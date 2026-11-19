use crate::{
    arch::{cp0, interrupt::InterruptStack},
    context::context::Kstack,
    memory::RmmA,
    percpu::PercpuBlock,
    syscall::FloatRegisters,
};
use crate::arch::interrupt::handler::sign_extend;
use core::{mem::offset_of, sync::atomic::AtomicBool};
use rmm::{Arch, VirtualAddress};
use spin::Once;
use syscall::{error::*, EnvRegisters};

pub const KFX_ALIGN: usize = 16;

/// Callee-saved state of a context that is not running. Registers are 64
/// bits on the R4400 even though n32 pointers are 32.
#[derive(Clone, Debug, Default)]
#[repr(C)]
pub struct Context {
    sp: u64,
    ra: u64,
    fp: u64,
    gp: u64,
    s0: u64,
    s1: u64,
    s2: u64,
    s3: u64,
    s4: u64,
    s5: u64,
    s6: u64,
    s7: u64,
    status: u64,
}

impl Context {
    pub fn new() -> Self {
        Self {
            // Kernel mode, timer unmasked, interrupts off until the context
            // enables them itself (run_userspace) or returns to user mode.
            status: crate::arch::cp0::status::IM_ENABLED as u64,
            ..Self::default()
        }
    }

    /// What `switch_to` saved, for the debugger: sp, ra, fp, gp, s0..s7,
    /// status.
    pub fn debug_regs(&self) -> [u64; 13] {
        [
            self.sp, self.ra, self.fp, self.gp, self.s0, self.s1, self.s2, self.s3, self.s4,
            self.s5, self.s6, self.s7, self.status,
        ]
    }

    fn set_stack(&mut self, address: usize) {
        self.sp = sign_extend(address);
    }

    fn set_ra(&mut self, address: usize) {
        self.ra = sign_extend(address);
    }

    fn set_s7(&mut self, address: usize) {
        self.s7 = sign_extend(address);
    }

    pub(crate) fn setup_initial_call(
        &mut self,
        stack: &Kstack,
        func: extern "C" fn(),
        userspace_allowed: bool,
    ) {
        let mut stack_top = stack.initial_top();

        const INT_REGS_SIZE: usize = size_of::<InterruptStack>();

        if userspace_allowed {
            unsafe {
                // Zero-initialize InterruptStack registers.
                stack_top = stack_top.sub(INT_REGS_SIZE);
                stack_top.write_bytes(0_u8, INT_REGS_SIZE);
                (&mut *stack_top.cast::<InterruptStack>()).init();
            }
        }

        self.set_ra(crate::arch::interrupt::syscall::enter_usermode as usize);
        self.set_s7(func as usize);

        self.set_stack(stack_top as usize);
    }
}

impl super::Context {
    /// The FPU registers of this context. Zero until its first FPU use.
    pub fn get_fx_regs(&self) -> FloatRegisters {
        if !frame_uses_fpu(self) {
            return FloatRegisters::default();
        }
        unsafe { self.kfx.as_ptr().cast::<FloatRegisters>().read_unaligned() }
    }

    /// Set the FPU registers of this context and give it the FPU: its frame
    /// gets `Status.CU1`, so it loads these on its next switch in.
    pub fn set_fx_regs(&mut self, new: FloatRegisters) {
        unsafe { self.kfx.as_mut_ptr().cast::<FloatRegisters>().write_unaligned(new) };
        if let Some(regs) = self.regs_mut() {
            regs.iret.status |= (cp0::status::CU1 | cp0::status::FR) as u64;
        }
    }

    pub fn current_syscall(&self) -> Option<[usize; 7]> {
        if !self.inside_syscall {
            return None;
        }
        let regs = self.regs()?;
        let r = &regs.registers;
        Some([
            r.v0 as usize,
            r.a0 as usize,
            r.a1 as usize,
            r.a2 as usize,
            r.a3 as usize,
            r.a4 as usize,
            r.a5 as usize,
        ])
    }

    pub(crate) fn write_current_env_regs(&mut self, regs: EnvRegisters) -> Result<()> {
        self.write_env_regs(regs)
    }

    pub(crate) fn write_env_regs(&mut self, regs: EnvRegisters) -> Result<()> {
        if RmmA::virt_is_valid(VirtualAddress::new(regs.tp)) {
            match self.regs_mut() {
                Some(stack) => {
                    stack.registers.gp = sign_extend(regs.tp);
                    Ok(())
                }
                None => Err(Error::new(ESRCH)),
            }
        } else {
            Err(Error::new(EINVAL))
        }
    }

    pub(crate) fn read_current_env_regs(&self) -> Result<EnvRegisters> {
        self.read_env_regs()
    }

    pub(crate) fn read_env_regs(&self) -> Result<EnvRegisters> {
        match self.regs() {
            Some(stack) => Ok(EnvRegisters {
                tp: stack.registers.gp as usize,
            }),
            None => Err(Error::new(ESRCH)),
        }
    }
    pub fn set_userspace_io_allowed(&mut self, _allowed: bool) {}
}

pub static EMPTY_CR3: Once<rmm::PhysicalAddress> = Once::new();

// SAFETY: EMPTY_CR3 must be initialized.
pub unsafe fn empty_cr3() -> rmm::PhysicalAddress {
    unsafe {
        debug_assert!(EMPTY_CR3.poll().is_some());
        *EMPTY_CR3.get_unchecked()
    }
}

/// Whether this context has used the FPU: its trap frame has `Status.CU1`.
/// Such a context keeps its FPU registers in `kfx`, and a context without
/// the bit has none, so its first FPU instruction traps and starts from zero.
fn frame_uses_fpu(context: &super::Context) -> bool {
    context
        .regs()
        .is_some_and(|regs| regs.iret.status as u32 & cp0::status::CU1 != 0)
}

/// Save the FPU registers of the current context into `out`. Works with
/// `CU1` off: it turns `CU1` on for the save, then restores the old Status.
/// FR is 1 (set with CU1 by the exception path): 32 doubleword registers.
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn fp_save(out: *mut FloatRegisters) {
    core::arch::naked_asm!(
        "
        .set noreorder
        .set noat
        .set hardfloat                  // the kernel is soft float; only these two use FPU instructions
        mfc0    $11, $12                // old Status
        lui     $10, 0x2400             // CU1 and FR (bit 26)
        or      $10, $11, $10
        mtc0    $10, $12
        nop
        nop
        nop
        nop
        sdc1    $f0, 0($4)
        sdc1    $f1, 8($4)
        sdc1    $f2, 16($4)
        sdc1    $f3, 24($4)
        sdc1    $f4, 32($4)
        sdc1    $f5, 40($4)
        sdc1    $f6, 48($4)
        sdc1    $f7, 56($4)
        sdc1    $f8, 64($4)
        sdc1    $f9, 72($4)
        sdc1    $f10, 80($4)
        sdc1    $f11, 88($4)
        sdc1    $f12, 96($4)
        sdc1    $f13, 104($4)
        sdc1    $f14, 112($4)
        sdc1    $f15, 120($4)
        sdc1    $f16, 128($4)
        sdc1    $f17, 136($4)
        sdc1    $f18, 144($4)
        sdc1    $f19, 152($4)
        sdc1    $f20, 160($4)
        sdc1    $f21, 168($4)
        sdc1    $f22, 176($4)
        sdc1    $f23, 184($4)
        sdc1    $f24, 192($4)
        sdc1    $f25, 200($4)
        sdc1    $f26, 208($4)
        sdc1    $f27, 216($4)
        sdc1    $f28, 224($4)
        sdc1    $f29, 232($4)
        sdc1    $f30, 240($4)
        sdc1    $f31, 248($4)
        cfc1    $10, $31
        nop
        sw      $10, {fcsr}($4)
        mtc0    $11, $12
        nop
        nop
        jr      $31
        nop
        .set softfloat
        .set at
        .set reorder
        ",
        fcsr = const offset_of!(FloatRegisters, fcsr),
    )
}

/// Load the FPU registers of the current context from `src`, as `fp_save`
/// stores them. The same `CU1` handling applies.
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn fp_load(src: *const FloatRegisters) {
    core::arch::naked_asm!(
        "
        .set noreorder
        .set noat
        .set hardfloat                  // the kernel is soft float; only these two use FPU instructions
        mfc0    $11, $12                // old Status
        lui     $10, 0x2400             // CU1 and FR (bit 26)
        or      $10, $11, $10
        mtc0    $10, $12
        nop
        nop
        nop
        nop
        ldc1    $f0, 0($4)
        ldc1    $f1, 8($4)
        ldc1    $f2, 16($4)
        ldc1    $f3, 24($4)
        ldc1    $f4, 32($4)
        ldc1    $f5, 40($4)
        ldc1    $f6, 48($4)
        ldc1    $f7, 56($4)
        ldc1    $f8, 64($4)
        ldc1    $f9, 72($4)
        ldc1    $f10, 80($4)
        ldc1    $f11, 88($4)
        ldc1    $f12, 96($4)
        ldc1    $f13, 104($4)
        ldc1    $f14, 112($4)
        ldc1    $f15, 120($4)
        ldc1    $f16, 128($4)
        ldc1    $f17, 136($4)
        ldc1    $f18, 144($4)
        ldc1    $f19, 152($4)
        ldc1    $f20, 160($4)
        ldc1    $f21, 168($4)
        ldc1    $f22, 176($4)
        ldc1    $f23, 184($4)
        ldc1    $f24, 192($4)
        ldc1    $f25, 200($4)
        ldc1    $f26, 208($4)
        ldc1    $f27, 216($4)
        ldc1    $f28, 224($4)
        ldc1    $f29, 232($4)
        ldc1    $f30, 240($4)
        ldc1    $f31, 248($4)
        lw      $10, {fcsr}($4)
        nop
        ctc1    $10, $31
        nop
        nop
        mtc0    $11, $12
        nop
        nop
        jr      $31
        nop
        .set softfloat
        .set at
        .set reorder
        ",
        fcsr = const offset_of!(FloatRegisters, fcsr),
    )
}

/// Switch to the next context by restoring its stack and registers
pub unsafe fn switch_to(prev: &mut super::Context, next: &mut super::Context) {
    unsafe {
        // FPU registers belong to the context that used them last. Save the
        // outgoing context's, and load the incoming one's if it has any.
        if frame_uses_fpu(prev) {
            fp_save(prev.kfx.as_mut_ptr().cast());
        }
        if frame_uses_fpu(next) {
            fp_load(next.kfx.as_ptr().cast());
        }

        PercpuBlock::current()
            .new_addrsp_tmp
            .set(next.addr_space.clone());

        crate::arch::cache::STAT_SWITCHES
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        crate::arch::interrupt::exception::timing_charge();
        switch_to_inner(&mut prev.arch, &mut next.arch);
        // This context runs again: its time is not known by trap code.
        if crate::arch::cache::TRAP_TIMING {
            crate::arch::interrupt::exception::timing_charge();
            crate::arch::interrupt::exception::TIMING_CODE
                .store(31, core::sync::atomic::Ordering::Relaxed);
        }
    }
}

#[unsafe(naked)]
unsafe extern "C" fn switch_to_inner(prev: &mut Context, next: &mut Context) {
    core::arch::naked_asm!(r#"
        .set noreorder
        sd      $16, {off_s0}($4)
        ld      $16, {off_s0}($5)
        sd      $17, {off_s1}($4)
        ld      $17, {off_s1}($5)
        sd      $18, {off_s2}($4)
        ld      $18, {off_s2}($5)
        sd      $19, {off_s3}($4)
        ld      $19, {off_s3}($5)
        sd      $20, {off_s4}($4)
        ld      $20, {off_s4}($5)
        sd      $21, {off_s5}($4)
        ld      $21, {off_s5}($5)
        sd      $22, {off_s6}($4)
        ld      $22, {off_s6}($5)
        sd      $23, {off_s7}($4)
        ld      $23, {off_s7}($5)
        sd      $28, {off_gp}($4)
        ld      $28, {off_gp}($5)
        sd      $30, {off_fp}($4)
        ld      $30, {off_fp}($5)
        sd      $29, {off_sp}($4)
        ld      $29, {off_sp}($5)
        sd      $31, {off_ra}($4)
        ld      $31, {off_ra}($5)

        mfc0    $8, $12
        sd      $8, {off_status}($4)
        ld      $8, {off_status}($5)
        mtc0    $8, $12
        nop
        nop

        la      $25, {switch_hook}
        jr      $25
        nop
        .set reorder
        "#,
        off_s0 = const(offset_of!(Context, s0)),
        off_s1 = const(offset_of!(Context, s1)),
        off_s2 = const(offset_of!(Context, s2)),
        off_s3 = const(offset_of!(Context, s3)),
        off_s4 = const(offset_of!(Context, s4)),
        off_s5 = const(offset_of!(Context, s5)),
        off_s6 = const(offset_of!(Context, s6)),
        off_s7 = const(offset_of!(Context, s7)),
        off_gp = const(offset_of!(Context, gp)),
        off_fp = const(offset_of!(Context, fp)),
        off_sp = const(offset_of!(Context, sp)),
        off_ra = const(offset_of!(Context, ra)),
        off_status = const(offset_of!(Context, status)),

        switch_hook = sym crate::context::switch_finish_hook,
    );
}
