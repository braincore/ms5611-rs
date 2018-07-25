//! A library for the MS5611 barometric pressure sensor.

extern crate byteorder;
use byteorder::{ByteOrder, BigEndian};
extern crate i2cdev;
use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use std::time;
use std::thread;

fn get_i2c_bus_path(i2c_bus: i32) -> String {
    format!("/dev/i2c-{}", i2c_bus)
}

/// Oversampling ratio
/// See datasheet for more information.
pub enum Osr {
    Opt256,
    Opt512,
    Opt1024,
    Opt2048,
    Opt4096,
}

impl Osr {
    fn get_delay(&self) -> u64 {
        match *self {
            Osr::Opt256 => 1,
            Osr::Opt512 => 2,
            Osr::Opt1024 => 3,
            Osr::Opt2048 => 5,
            Osr::Opt4096 => 10,
        }
    }

    fn addr_modifier(&self) -> u8 {
        match *self {
            Osr::Opt256 => 0,
            Osr::Opt512 => 2,
            Osr::Opt1024 => 4,
            Osr::Opt2048 => 6,
            Osr::Opt4096 => 8,
        }
    }
}

/// Pressure sensor
pub struct Ms5611 {
    i2c_dev: LinuxI2CDevice,
    prom: Prom,
}

enum Ms5611Reg {
    Reset,
    /// Digital pressure value
    D1,
    /// Digital temperature value
    D2,
    /// AdcRead command returns 24-bit result.
    AdcRead,
    /// Prom command returns 16-bit result.
    Prom,
}

impl Ms5611Reg {
    fn addr(&self) -> u8 {
        match *self {
            Ms5611Reg::Reset => 0x1e,
            Ms5611Reg::D1 => 0x40,
            Ms5611Reg::D2 => 0x50,
            Ms5611Reg::AdcRead => 0x00,
            // Valid from 0xa0 to 0xae
            Ms5611Reg::Prom => 0xa0,
        }
    }
}

/// Output from the MS5611.
#[derive(Debug)]
pub struct Ms5611Sample {
    /// Pressure measured in millibars.
    pub pressure_mbar: f32,
    /// Temperature in celsius.
    pub temperature_c: f32,
}

/// Factory calibrated data in device's ROM.
#[derive(Debug)]
struct Prom {
    /// From datasheet, C1.
    pub pressure_sensitivity: u16,
    /// From datasheet, C2.
    pub pressure_offset: u16,
    /// From datasheet, C3.
    pub temp_coef_pressure_sensitivity: u16,
    /// From datasheet, C4.
    pub temp_coef_pressure_offset: u16,
    /// From datasheet, C5.
    pub temp_ref: u16,
    /// From datasheet, C6.
    pub temp_coef_temp: u16,
}

impl Ms5611 {

    /// If i2c_addr is unspecified, 0x77 is used.
    /// The addr of the device is 0x77 if CSB is low / 0x76 if CSB is high.
    pub fn new(i2c_bus: i32, i2c_addr: Option<u16>)
            -> Result<Ms5611, LinuxI2CError> {
        let mut i2c_dev = LinuxI2CDevice::new(
            get_i2c_bus_path(i2c_bus), i2c_addr.unwrap_or(0x77))?;

        let prom = Self::read_prom(&mut i2c_dev)?;

        let ms = Ms5611 {
            i2c_dev,
            prom,
        };

        Ok(ms)
    }

    /// Triggers a hardware reset of the device.
    pub fn reset(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev.write(&[Ms5611Reg::Reset.addr()])?;
        // Haven't tested for the lower time bound necessary for the chip to
        // start functioning again. But, it does require some amount of sleep.
        thread::sleep(time::Duration::from_millis(50));
        Ok(())
    }

    fn read_prom(i2c_dev: &mut LinuxI2CDevice) -> Result<Prom, LinuxI2CError> {
        let mut crc_check = 0u16;

        // This is the CRC scheme in the MS5611 AN520 (Application Note)
        fn crc_accumulate_byte(crc_check: &mut u16, byte: u8) {
            *crc_check ^= byte as u16;
            for _ in 0..8 {
                if (*crc_check & 0x8000) > 0 {
                    *crc_check = (*crc_check << 1) ^ 0x3000;
                } else {
                    *crc_check = *crc_check << 1;
                }
            }
        }

        fn crc_accumulate_buf2(crc_check: &mut u16, buf: &[u8]) {
            crc_accumulate_byte(crc_check,buf[0]);
            crc_accumulate_byte(crc_check,buf[1]);
        }

        let mut buf: [u8; 2] = [0u8; 2];
        // Address reserved for manufacturer. We need it for the CRC.
        i2c_dev.write(&[Ms5611Reg::Prom.addr()])?;
        i2c_dev.read(&mut buf)?;
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c_dev.write(&[Ms5611Reg::Prom.addr() + 2])?;
        i2c_dev.read(&mut buf)?;
        let pressure_sensitivity = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c_dev.write(&[Ms5611Reg::Prom.addr() + 4])?;
        i2c_dev.read(&mut buf)?;
        let pressure_offset = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c_dev.write(&[Ms5611Reg::Prom.addr() + 6])?;
        i2c_dev.read(&mut buf)?;
        let temp_coef_pressure_sensitivity = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c_dev.write(&[Ms5611Reg::Prom.addr() + 8])?;
        i2c_dev.read(&mut buf)?;
        let temp_coef_pressure_offset = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c_dev.write(&[Ms5611Reg::Prom.addr() + 10])?;
        i2c_dev.read(&mut buf)?;
        let temp_ref = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c_dev.write(&[Ms5611Reg::Prom.addr() + 12])?;
        i2c_dev.read(&mut buf)?;
        let temp_coef_temp = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c_dev.write(&[Ms5611Reg::Prom.addr() + 14])?;
        i2c_dev.read(&mut buf)?;
        // CRC is only last 4 bits
        let crc = BigEndian::read_u16(&mut buf) & 0x000f;
        crc_accumulate_byte(&mut crc_check, buf[0]);
        crc_accumulate_byte(&mut crc_check, 0);

        crc_check = crc_check >> 12;

        if crc != crc_check {
            panic!("PROM CRC did not match: {} != {}", crc, crc_check);
        }

        Ok(Prom {
            pressure_sensitivity,
            pressure_offset,
            temp_coef_pressure_sensitivity,
            temp_coef_pressure_offset,
            temp_ref,
            temp_coef_temp,
        })
    }

    /// Based on oversampling ratio, function may block between 1ms (OSR=256)
    /// to 18ms (OSR=4096). To avoid blocking, consider invoking this function
    /// in a separate thread.
    pub fn read_sample(&mut self, osr: Osr) -> Result<Ms5611Sample, LinuxI2CError> {
        // Note: Variable names aren't pretty, but they're consistent with the
        // MS5611 datasheet.
        let mut buf = [0u8; 4];

        self.i2c_dev.write(&[Ms5611Reg::D1.addr() + osr.addr_modifier()])?;
        // If we don't delay, the read is all 0s.
        thread::sleep(time::Duration::from_millis(osr.get_delay()));
        self.i2c_dev.write(&[Ms5611Reg::AdcRead.addr()])?;
        self.i2c_dev.read(&mut buf[1 .. 4])?;

        // Raw digital pressure
        let d1 = BigEndian::read_i32(&mut buf);

        self.i2c_dev.write(&[Ms5611Reg::D2.addr() + osr.addr_modifier()])?;
        thread::sleep(time::Duration::from_millis(osr.get_delay()));
        self.i2c_dev.write(&[Ms5611Reg::AdcRead.addr()])?;
        self.i2c_dev.read(&mut buf[1 .. 4])?;

        // Raw digital temperature
        let d2 = BigEndian::read_i32(&mut buf) as i64;

        // Temperature difference from reference
        let dt = d2 - ((self.prom.temp_ref as i64) << 8);

        // Units: celcius * 100
        let mut temperature: i32 = 2000 +
            (((dt * (self.prom.temp_coef_temp as i64)) >> 23) as i32);

        let mut offset: i64 = ((self.prom.pressure_offset as i64) << 16)
            + ((dt * (self.prom.temp_coef_pressure_offset as i64)) >> 7);
        let mut sens: i64 = ((self.prom.pressure_sensitivity as i64) << 15)
            + ((dt * (self.prom.temp_coef_pressure_sensitivity as i64)) >> 8);

        let mut t2 = 0i32;
        let mut off2 = 0i64;
        let mut sens2 = 0i64;

        //
        // Second order temperature compensation
        //

        // Low temperature (< 20C)
        if temperature < 2000 {
            t2 = ((dt * dt) >> 31) as i32;
            off2 = ((5 * (temperature - 2000).pow(2)) >> 1) as i64;
            sens2 = off2 >> 1;
        }

        // Very low temperature (< -15)
        if temperature < -1500 {
            off2 += 7 * (temperature as i64 + 1500).pow(2);
            sens2 += ((11 * (temperature as i64 + 1500).pow(2)) >> 1) as i64;
        }

        temperature -= t2;
        offset -= off2;
        sens -= sens2;

        // Units: mbar * 100
        let pressure: i32 = (((((d1 as i64) * sens) >> 21) - offset) >> 15) as i32;

        Ok(Ms5611Sample {
            pressure_mbar: pressure as f32/100.0,
            temperature_c: temperature as f32/100.0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{Ms5611, Osr};
    use std::env;

    fn get_i2c_bus() -> i32 {
        match env::var("MS5611_I2C_BUS") {
            Ok(bus_string) => {
                bus_string.parse().expect(
                    "Could not convert MS5611_I2C_BUS env var to i32.")
            },
            Err(_) => 1,
        }
    }

    fn get_i2c_addr() -> Option<u16> {
        match env::var("MS5611_I2C_ADDR") {
            Ok(addr_string) => {
                Some(addr_string.parse().expect(
                    "Could not convert MS5611_I2C_ADDR env var to u16."))
            },
            Err(_) => None,
        }
    }

    #[test]
    fn basic() {
        let mut ms5611 = Ms5611::new(
            get_i2c_bus(), get_i2c_addr()).unwrap();
        ms5611.read_sample(Osr::Opt256).unwrap();
        ms5611.reset().unwrap();
    }
}
