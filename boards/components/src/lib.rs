// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

#![no_std]

pub mod adc;
pub mod adc_microphone;
pub mod aes;
pub mod air_quality;
pub mod alarm;
pub mod analog_comparator;
pub mod apds9960;
pub mod app_flash_driver;
pub mod app_loader;
pub mod appid;
pub mod atecc508a;
pub mod ble;
pub mod bme280;
pub mod bmm150;
pub mod bmp280;
pub mod bus;
pub mod button;
pub mod can;
pub mod ccs811;
pub mod cdc;
pub mod chirp_i2c_moisture;
pub mod console;
pub mod crc;
pub mod ctap;
pub mod dac;
pub mod date_time;
pub mod debug_writer;
pub mod dfrobot_rainfall_sensor;
pub mod dynamic_binary_storage;
pub mod eui64;
pub mod flash;
pub mod fm25cl;
pub mod ft6x06;
pub mod fxos8700;
pub mod gpio;
pub mod hd44780;
pub mod hmac;
pub mod hs3003;
pub mod hts221;
pub mod humidity;
pub mod i2c;
pub mod ieee802154;
pub mod isl29035;
pub mod isolated_nonvolatile_storage;
pub mod keyboard_hid;
pub mod kv;
pub mod l3gd20;
pub mod led;
pub mod led_matrix;
pub mod lldb;
pub mod loader;
pub mod lpm013m126;
pub mod lps22hb;
pub mod lps25hb;
pub mod lsm303agr;
pub mod lsm303dlhc;
pub mod lsm6dsox;
pub mod ltc294x;
pub mod mlx90614;
pub mod moisture;
pub mod mx25r6435f;
pub mod ninedof;
pub mod nonvolatile_storage;
pub mod nrf51822;
pub mod panic_button;
pub mod pressure;
pub mod process_array;
pub mod process_console;
pub mod process_info_driver;
pub mod process_printer;
pub mod proximity;
pub mod pwm;
pub mod rainfall;
pub mod rf233;
pub mod rng;
pub mod sched;
pub mod screen;
pub mod segger_rtt;
pub mod servo;
pub mod sh1106;
pub mod sha;
pub mod sht3x;
pub mod sht4x;
pub mod si7021;
pub mod signature_verify_in_memory_keys;
pub mod siphash;
pub mod sound_pressure;
pub mod spi;
pub mod ssd1306;
pub mod st77xx;
pub mod storage_permissions;
pub mod temperature;
pub mod temperature_rp2040;
pub mod temperature_stm;
pub mod test;
pub mod text_screen;
pub mod thread_network;
pub mod tickv;
pub mod touch;
pub mod udp_driver;
pub mod udp_mux;
pub mod usb;
