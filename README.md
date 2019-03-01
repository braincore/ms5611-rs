# MS5611 Library for Rust [![Latest Version]][crates.io] [![Documentation]][docs.rs]

[Latest Version]: https://img.shields.io/crates/v/ms5611.svg
[crates.io]: https://crates.io/crates/ms5611
[Documentation]: https://docs.rs/ms5611/badge.svg
[docs.rs]: https://docs.rs/ms5611

A library for the MS5611 barometric pressure sensor. Only supports the i2c
interface (no SPI).

## Features

* Per datasheet, computes the second order temperature compensation.
* Validates the PROM's checksum.

## Usage

See basic test in `tests/basic.rs`.

## Testing

By default, uses i2c bus=1, addr=0x77. To override, use these environment
variables:

```
MS5611_I2C_BUS2=1 MS5611_I2C_ADDR=119 cargo test
```
