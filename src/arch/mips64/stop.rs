use crate::sync::CleanLockToken;

/// Reset the machine through the memory controller: CPUCTRL0 bit 4 (SYSINIT)
/// pulses the system reset line, so the PROM starts from a real reset with
/// every device in its power-on state. Jumping to the PROM's entry instead
/// left the real machine with a blank screen.
pub unsafe fn kreset() -> ! {
    println!("kreset");
    unsafe { emergency_reset() }
}

pub unsafe fn emergency_reset() -> ! {
    const MC_CPUCTRL0: *mut u32 = 0xBFA0_0004 as *mut u32;
    const SYSINIT: u32 = 0x10;
    unsafe {
        super::interrupt::disable();
        // First choice: the firmware's own Reboot. On the real machine both
        // the SYSINIT reset and a jump to the PROM entry left the screen off
        // and the PROM not running.
        super::device::arcs::reboot();
        MC_CPUCTRL0.write_volatile(MC_CPUCTRL0.read_volatile() | SYSINIT);
        // The reset takes effect within a few bus cycles. If it does not,
        // fall back to the PROM's entry with its own vectors selected.
        for _ in 0..1_000_000 {
            core::hint::spin_loop();
        }
        core::arch::asm!(
            "li $8, 0x00400000", // Status: BEV only
            "mtc0 $8, $12",
            "li $8, 0xBFC00000",
            "jr $8",
            "nop",
            options(noreturn, nostack)
        );
    }
}

pub unsafe fn kstop(_token: &mut CleanLockToken) -> ! {
    // TODO: soft power off through the DS1386 / IOC2 power control register.
    println!("kstop: halting");
    loop {
        unsafe {
            super::interrupt::disable();
            super::interrupt::halt();
        }
    }
}
