use crate::{
    arch::interrupt::InterruptStack, context::context::Kstack, memory::RmmA, percpu::PercpuBlock,
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
    pub fn get_fx_regs(&self) -> FloatRegisters {
        // TODO: the FPU is disabled (Status.CU1 = 0); userspace is soft float.
        FloatRegisters::default()
    }

    pub fn set_fx_regs(&mut self, _new: FloatRegisters) {}

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

/// Switch to the next context by restoring its stack and registers
pub unsafe fn switch_to(prev: &mut super::Context, next: &mut super::Context) {
    unsafe {
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
