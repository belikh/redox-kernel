use spin::MutexGuard;

use crate::{arch::device::{newport, serial::COM1}, devices::serial::SerialKind};

pub struct Writer<'a> {
    serial: MutexGuard<'a, SerialKind>,
}

impl<'a> Writer<'a> {
    pub fn new() -> Writer<'a> {
        Writer {
            serial: COM1.lock(),
        }
    }

    pub fn write(&mut self, buf: &[u8]) {
        // The screen first: it is the console on a machine with no serial
        // cable. Serial Port 1 always gets a copy.
        newport::write(buf);
        self.serial.write(buf);
    }
}
