use linux_embedded_hal::{Delay, I2cdev};
use ms5611::{Ms5611, Osr};
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

fn get_i2c_addr() -> Option<u8> {
    match env::var("MS5611_I2C_ADDR") {
        Ok(addr_string) => {
            Some(addr_string.parse().expect(
                "Could not convert MS5611_I2C_ADDR env var to u16."))
        },
        Err(_) => None,
    }
}


fn get_i2c_bus_path(i2c_bus: i32) -> String {
    format!("/dev/i2c-{}", i2c_bus)
}


#[test]
fn basic() {
    let dev = I2cdev::new(get_i2c_bus_path(get_i2c_bus())).unwrap();

    let mut ms5611 = Ms5611::new(dev, get_i2c_addr()).unwrap();

    let mut delay = Delay;

    ms5611.read_sample(Osr::Opt256, &mut delay).unwrap();
    ms5611.reset(&mut delay).unwrap();
}
