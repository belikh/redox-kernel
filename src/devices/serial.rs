use syscall::MmioPtr;
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
use syscall::Pio;

#[cfg(target_arch = "mips64")]
use crate::devices::uart_z85c30;
use crate::{
    devices::{uart_16550, uart_meson, uart_pl011},
    scheme::debug::{debug_input, debug_notify},
    sync::CleanLockToken,
};

#[allow(dead_code)]
pub enum SerialKind {
    NotPresent,
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    Ns16550Pio(uart_16550::SerialPort<Pio<u8>>),
    Ns16550u8(uart_16550::SerialPort<MmioPtr<u8>>),
    Ns16550u32(uart_16550::SerialPort<MmioPtr<u32>>),
    Meson(uart_meson::SerialPort),
    Pl011(uart_pl011::SerialPort),
    #[cfg(target_arch = "mips64")]
    Z85c30(uart_z85c30::SerialPort),
}

impl SerialKind {
    #[cfg(target_arch = "aarch64")]
    pub fn init_full(&mut self) -> Result<(), ()> {
        match self {
            Self::NotPresent => Err(()),
            Self::Meson(inner) => inner.init_full(),
            _ => Ok(()),
        }
    }

    #[cfg(target_arch = "aarch64")]
    pub fn enable_irq(&mut self) {
        //TODO: implement for NS16550
        match self {
            Self::NotPresent => {}
            #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
            Self::Ns16550Pio(_) => {}
            Self::Ns16550u8(_) => {}
            Self::Ns16550u32(_) => {}
            Self::Meson(inner) => inner.enable_irq(),
            Self::Pl011(inner) => inner.enable_irq(),
        }
    }

    /// Z85C30 only: see `uart_z85c30::SerialPort::enable_interrupts`.
    #[cfg(target_arch = "mips64")]
    pub fn enable_serial_interrupts(&mut self, queue_output: bool) {
        if let Self::Z85c30(inner) = self {
            inner.enable_interrupts(queue_output);
        }
    }

    pub fn receive(&mut self, token: &mut CleanLockToken) {
        //TODO: make PL011 receive work the same way as NS16550
        match self {
            Self::NotPresent => {}
            #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
            Self::Ns16550Pio(inner) => {
                while let Some(c) = inner.receive() {
                    debug_input(c, token);
                }
                debug_notify(token);
            }
            Self::Ns16550u8(inner) => {
                while let Some(c) = inner.receive() {
                    debug_input(c, token);
                }
                debug_notify(token);
            }
            Self::Ns16550u32(inner) => {
                while let Some(c) = inner.receive() {
                    debug_input(c, token);
                }
                debug_notify(token);
            }
            Self::Meson(inner) => inner.receive(token),
            Self::Pl011(inner) => inner.receive(token),
            #[cfg(target_arch = "mips64")]
            Self::Z85c30(inner) => {
                // Notify readers only when a byte arrived.
                let mut any = false;
                while let Some(c) = inner.receive() {
                    debug_input(c, token);
                    any = true;
                }
                inner.service();
                if any {
                    debug_notify(token);
                }
            }
        }
    }

    pub fn write(&mut self, buf: &[u8]) {
        match self {
            Self::NotPresent => {}
            #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
            Self::Ns16550Pio(inner) => inner.write(buf),
            Self::Ns16550u8(inner) => inner.write(buf),
            Self::Ns16550u32(inner) => inner.write(buf),
            Self::Meson(inner) => inner.write(buf),
            Self::Pl011(inner) => inner.write(buf),
            #[cfg(target_arch = "mips64")]
            Self::Z85c30(inner) => inner.write(buf),
        }
    }
}

// Required for use in static mutex
unsafe impl Send for SerialKind {}
