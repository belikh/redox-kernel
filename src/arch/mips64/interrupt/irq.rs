//! Device interrupts for the `irq:` scheme: the IOC2's INT3 controller.
//!
//! INT3 has two 8-bit local interrupt registers. Local 0 drives CPU IP2,
//! local 1 drives IP3. Each has a status and a mask register (a set mask bit
//! enables the line). IRQ numbers 0..8 are local 0 bits, 8..16 local 1 bits:
//!
//! | IRQ | Source |
//! |---|---|
//! | 1, 2 | SCSI 0, SCSI 1 |
//! | 3 | Ethernet (Seeq 8003 and its HPC3 DMA) |
//! | 5 | parallel port |
//! | 12 | HPC3 DMA done |
//!
//! A line that fires is masked and reported to its `irq:` handle; the
//! driver's acknowledgement unmasks it. A line nobody holds stays masked
//! after its first interrupt.
//!
//! INT3 also has eight mappable lines with their own status register and
//! two mask registers: mask 0 gathers lines into local 0 bit 7, mask 1 into
//! local 1 bit 3. The kernel's console owns two of them through mask 0 and
//! serves them itself: line 4, the keyboard controller, and line 5, the
//! serial ports. Both are level interrupts that drop when the device is
//! read, so they are never masked. Local 0 bit 7 is therefore not an `irq:`
//! line.

use core::ptr::{read_volatile, write_volatile};

use crate::{
    arch::cp0::{self, cause, status},
    cpu_set::LogicalCpuId,
    scheme::irq::irq_trigger,
    sync::CleanLockToken,
};

const L0_STAT: *mut u8 = 0xBFBD_9883 as *mut u8;
const L0_MASK: *mut u8 = 0xBFBD_9887 as *mut u8;
const L1_STAT: *mut u8 = 0xBFBD_988B as *mut u8;
const L1_MASK: *mut u8 = 0xBFBD_988F as *mut u8;
const MAP_STAT: *mut u8 = 0xBFBD_9893 as *mut u8;
const MAP_MASK0: *mut u8 = 0xBFBD_9897 as *mut u8;
const MAP_MASK1: *mut u8 = 0xBFBD_989B as *mut u8;

const MAP_KEYBOARD: u8 = 1 << 4;
const MAP_SERIAL: u8 = 1 << 5;
/// Local 0 bit 7: the mappable lines enabled in mask 0.
const L0_MAPPABLE: u8 = 1 << 7;

/// Enable every local line and the two CPU inputs they cascade into.
pub unsafe fn init() {
    unsafe {
        write_volatile(MAP_MASK1, 0);
        write_volatile(MAP_MASK0, MAP_KEYBOARD | MAP_SERIAL);
        // The serial port interrupts for received characters. With the
        // screen as the console its output is only a copy, sent from a
        // queue by the transmit interrupt.
        crate::arch::device::serial::COM1.lock().enable_serial_interrupts(
            crate::arch::device::newport::active(),
        );
        write_volatile(L0_MASK, 0xff);
        write_volatile(L1_MASK, 0xff);
        cp0::write_status(cp0::read_status() | status::IM2 | status::IM3);
    }
}

/// Called from the exception handler with the pending, enabled Cause bits.
pub unsafe fn handle(pending: u32, token: &mut CleanLockToken) {
    unsafe {
        if pending & cause::IP2 != 0 {
            dispatch(L0_STAT, L0_MASK, 0, token);
        }
        if pending & cause::IP3 != 0 {
            dispatch(L1_STAT, L1_MASK, 8, token);
        }
    }
}

unsafe fn dispatch(stat: *mut u8, mask: *mut u8, base: u8, token: &mut CleanLockToken) {
    unsafe {
        let enabled = read_volatile(mask);
        let mut active = read_volatile(stat) & enabled;
        if base == 0 && active & L0_MAPPABLE != 0 {
            active &= !L0_MAPPABLE;
            let lines = read_volatile(MAP_STAT) & read_volatile(MAP_MASK0);
            if lines & MAP_KEYBOARD != 0 {
                crate::arch::device::KEYBOARD_INTERRUPTS
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                crate::arch::device::keyboard_input(token);
            }
            if lines & MAP_SERIAL != 0 {
                crate::arch::device::SERIAL_INTERRUPTS
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                crate::arch::device::serial_service(token);
            }
            // A line nobody serves would interrupt for ever: drop it.
            let stray = lines & !(MAP_KEYBOARD | MAP_SERIAL);
            if stray != 0 {
                write_volatile(MAP_MASK0, read_volatile(MAP_MASK0) & !stray);
            }
        }
        if active == 0 {
            return;
        }
        // Mask what fired until its driver acknowledges it.
        write_volatile(mask, enabled & !active);
        for bit in 0..8u8 {
            if active & (1 << bit) != 0 {
                irq_trigger(base + bit, token);
            }
        }
    }
}

pub unsafe fn acknowledge(irq: usize) {
    unsafe {
        let (mask, bit) = match irq {
            // Bit 7 belongs to the console's mappable lines.
            7 => return,
            0..=6 => (L0_MASK, irq as u8),
            8..=15 => (L1_MASK, irq as u8 - 8),
            _ => return,
        };
        write_volatile(mask, read_volatile(mask) | (1 << bit));
    }
}

pub fn is_reserved(_cpu_id: LogicalCpuId, _index: u8) -> bool {
    false
}

pub fn set_reserved(_cpu_id: LogicalCpuId, _index: u8, _reserved: bool) {}

/// Only the sixteen INT3 lines exist; none is handed out dynamically.
pub fn available_irqs_iter(_cpu_id: LogicalCpuId) -> impl Iterator<Item = u8> + 'static {
    0..0
}
