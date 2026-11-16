#![allow(dead_code)]
pub mod graphical_debug;
pub mod serial;
pub mod uart_16550;
pub mod uart_meson;
pub mod uart_pl011;
#[cfg(target_arch = "mips64")]
pub mod uart_z85c30;
