use super::cp0;

#[macro_use]
pub mod handler;

pub mod exception;
pub mod irq;
pub mod syscall;
pub mod trace;

pub use handler::InterruptStack;

/// Clear interrupts
#[inline(always)]
pub unsafe fn disable() {
    unsafe { cp0::write_status(cp0::read_status() & !cp0::status::IE) }
}

/// Set interrupts and halt
/// This will atomically wait for the next interrupt
/// Performing enable followed by halt is not guaranteed to be atomic, use this instead!
#[inline(always)]
pub unsafe fn enable_and_halt() {
    // WAIT (COP0, CO=1, funct 0x20, word 0x42000020) stops the pipeline until
    // an interrupt is requested. It takes effect only while the SysAD bus is
    // idle, and otherwise does nothing, so the caller loops. An interrupt
    // already pending when IE is set is taken before WAIT runs.
    unsafe {
        cp0::write_status(cp0::read_status() | cp0::status::IE);
        core::arch::asm!(".word 0x42000020", "nop", options(nomem, nostack));
    }
}

/// Set interrupts and nop
/// This will enable interrupts and allow the IF flag to be processed
/// Simply enabling interrupts does not gurantee that they will trigger, use this instead!
#[inline(always)]
pub unsafe fn enable_and_nop() {
    unsafe {
        cp0::write_status(cp0::read_status() | cp0::status::IE);
        core::arch::asm!("nop", options(nomem, nostack));
    }
}

/// Halt instruction
#[inline(always)]
pub unsafe fn halt() {
    unsafe { core::arch::asm!("nop", options(nomem, nostack)) }
}
