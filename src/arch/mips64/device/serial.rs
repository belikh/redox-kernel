use spin::Mutex;

use crate::devices::{serial::SerialKind, uart_z85c30::SerialPort};

/// IOC2 Z85C30 channel B control port (kseg1). Channel B is "Serial Port 1".
pub const SCC_CHANNEL_B: usize = 0xBFBD_9833;

pub static COM1: Mutex<SerialKind> = Mutex::new(SerialKind::NotPresent);

pub unsafe fn init() {
    let mut port = unsafe { SerialPort::new(SCC_CHANNEL_B) };
    port.init();
    *COM1.lock() = SerialKind::Z85c30(port);
}
