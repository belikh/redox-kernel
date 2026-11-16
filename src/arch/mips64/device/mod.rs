//! Indy devices the kernel itself drives: the console (Serial Port 1, or the
//! Newport screen with the IOC2 keyboard) and the CP0 timer. Everything else
//! (HPC3, SCSI, Seeq Ethernet) is a userspace scheme reached through fixed
//! kseg1 addresses; there is no bus enumeration.

pub mod arcs;
pub mod cpu;
pub mod kbd;
pub mod newport;
pub mod serial;

pub unsafe fn init() {
    unsafe {
        super::time::init();
        super::interrupt::irq::init();
    }
}

pub unsafe fn init_noncore() {}

/// Counters for the console's interrupts and for the once-a-second fallback
/// poll finding input an interrupt should have delivered. Read with kdbg.
pub static KEYBOARD_INTERRUPTS: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(0);
pub static SERIAL_INTERRUPTS: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(0);
pub static FALLBACK_INPUT: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(0);

/// Move what the keyboard controller holds, and the screen's replies to
/// terminal reports, to the console input. Returns the number of bytes
/// that came from the keyboard.
pub fn keyboard_input(token: &mut crate::sync::CleanLockToken) -> usize {
    let mut bytes = [0u8; 96];
    let mut len = {
        let mut reply = [0u8; 32];
        let n = newport::take_reply(&mut reply);
        bytes[..n].copy_from_slice(&reply[..n]);
        n
    };
    let replies = len;
    kbd::poll(&mut |byte| {
        if len < bytes.len() {
            bytes[len] = byte;
            len += 1;
        }
    });
    for &byte in &bytes[..len] {
        crate::scheme::debug::debug_input(byte, token);
    }
    if len != 0 {
        crate::scheme::debug::debug_notify(token);
    }
    len - replies
}

/// Serial Port 1: received bytes to the console input, queued output to
/// the chip.
pub fn serial_service(token: &mut crate::sync::CleanLockToken) {
    serial::COM1.lock().receive(token);
}

pub struct ArchPercpuMisc;

impl ArchPercpuMisc {
    pub const fn default() -> Self {
        Self
    }
}
