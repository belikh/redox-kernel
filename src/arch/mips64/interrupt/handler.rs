//! The trap frame. Every general register is saved as 64 bits, because n32
//! code keeps 64-bit values in registers even though pointers are 32 bits.
//!
//! Layout is shared with the assembly in `exception.rs`: register `$n` is at
//! byte offset `(n - 1) * 8`, followed by the CP0 state.

use crate::{memory::ArchIntCtx, panic, syscall::IntRegisters};

#[derive(Default)]
#[repr(C)]
pub struct Registers {
    pub at: u64, // $1
    pub v0: u64, // $2
    pub v1: u64, // $3
    pub a0: u64, // $4
    pub a1: u64, // $5
    pub a2: u64, // $6
    pub a3: u64, // $7
    pub a4: u64, // $8
    pub a5: u64, // $9
    pub a6: u64, // $10
    pub a7: u64, // $11
    pub t0: u64, // $12
    pub t1: u64, // $13
    pub t2: u64, // $14
    pub t3: u64, // $15
    pub s0: u64, // $16
    pub s1: u64, // $17
    pub s2: u64, // $18
    pub s3: u64, // $19
    pub s4: u64, // $20
    pub s5: u64, // $21
    pub s6: u64, // $22
    pub s7: u64, // $23
    pub t8: u64, // $24
    pub t9: u64, // $25
    pub k0: u64, // $26 (kernel scratch, not restored)
    pub k1: u64, // $27, preserved: the entry stashes it in ErrorEPC
    pub gp: u64, // $28, the thread pointer
    pub sp: u64, // $29
    pub fp: u64, // $30
    pub ra: u64, // $31
}

impl Registers {
    pub fn dump(&self) {
        println!("AT:  {:>016X}  V0:  {:>016X}  V1:  {:>016X}", { self.at }, { self.v0 }, { self.v1 });
        println!("A0:  {:>016X}  A1:  {:>016X}  A2:  {:>016X}", { self.a0 }, { self.a1 }, { self.a2 });
        println!("A3:  {:>016X}  A4:  {:>016X}  A5:  {:>016X}", { self.a3 }, { self.a4 }, { self.a5 });
        println!("A6:  {:>016X}  A7:  {:>016X}  T0:  {:>016X}", { self.a6 }, { self.a7 }, { self.t0 });
        println!("T1:  {:>016X}  T2:  {:>016X}  T3:  {:>016X}", { self.t1 }, { self.t2 }, { self.t3 });
        println!("S0:  {:>016X}  S1:  {:>016X}  S2:  {:>016X}", { self.s0 }, { self.s1 }, { self.s2 });
        println!("S3:  {:>016X}  S4:  {:>016X}  S5:  {:>016X}", { self.s3 }, { self.s4 }, { self.s5 });
        println!("S6:  {:>016X}  S7:  {:>016X}  T8:  {:>016X}", { self.s6 }, { self.s7 }, { self.t8 });
        println!("T9:  {:>016X}  GP:  {:>016X}  SP:  {:>016X}", { self.t9 }, { self.gp }, { self.sp });
        println!("FP:  {:>016X}  RA:  {:>016X}", { self.fp }, { self.ra });
    }
}

#[derive(Default)]
#[repr(C)]
pub struct IretRegisters {
    pub epc: u64,
    pub status: u64,
    pub cause: u64,
    pub badvaddr: u64,
    pub hi: u64,
    pub lo: u64,
    /// Keeps the frame a multiple of 16 bytes.
    pub pad: u64,
}

impl IretRegisters {
    pub fn dump(&self) {
        println!(
            "EPC: {:>016X}  STATUS: {:>08X}  CAUSE: {:>08X}  BADVADDR: {:>016X}",
            { self.epc },
            { self.status },
            { self.cause },
            { self.badvaddr }
        );
    }
}

// NOTE: Layout of this structure must be synced with assembly code in exception.rs
#[derive(Default)]
#[repr(C)]
pub struct InterruptStack {
    pub registers: Registers,
    pub iret: IretRegisters,
}

/// Byte size of the trap frame, as the assembly reserves it.
pub const FRAME_SIZE: usize = 38 * 8;

/// n32 keeps every 32-bit value sign-extended in its 64-bit register. A
/// value written into a register slot must follow that rule, or compiled
/// code comparing it against a sign-extended constant goes wrong.
#[inline(always)]
pub const fn sign_extend(value: usize) -> u64 {
    value as u32 as i32 as i64 as u64
}

impl InterruptStack {
    pub fn init(&mut self) {
        const {
            assert!(FRAME_SIZE == size_of::<InterruptStack>());
            assert!(FRAME_SIZE % 16 == 0);
        }
        // Return to user mode with interrupts enabled.
        // UX: n32 code uses 64-bit instructions (`sd`, `ld`, `daddu`), which
        // are reserved in user mode without it. With UX set, user TLB misses
        // arrive at the XTLB refill vector, which has the same stub.
        self.iret.status = (crate::arch::cp0::status::KSU_USER
            | crate::arch::cp0::status::UX
            | crate::arch::cp0::status::EXL
            | crate::arch::cp0::status::IE
            | crate::arch::cp0::status::IM_ENABLED) as u64;
    }
    pub fn frame_pointer(&self) -> usize {
        self.registers.fp as usize
    }
    pub fn stack_pointer(&self) -> usize {
        self.registers.sp as usize
    }
    pub fn set_stack_pointer(&mut self, sp: usize) {
        self.registers.sp = sign_extend(sp);
    }
    pub fn set_instr_pointer(&mut self, ip: usize) {
        self.iret.epc = sign_extend(ip);
    }
    pub fn instr_pointer(&self) -> usize {
        self.iret.epc as usize
    }
    pub fn sig_archdep_reg(&self) -> usize {
        self.registers.t0 as usize
    }

    pub fn set_syscall_ret_reg(&mut self, ret: usize) {
        self.registers.v0 = sign_extend(ret);
    }

    pub fn set_arg1(&mut self, arg_opt: Option<usize>) {
        if let Some(arg) = arg_opt {
            self.registers.a1 = sign_extend(arg);
        }
    }

    pub fn dump(&self) {
        self.iret.dump();
        self.registers.dump();
    }

    pub fn trace(&self) {
        self.dump();
        unsafe {
            panic::user_stack_trace(&self);
            panic::stack_trace();
        }
    }

    /// Saves all registers to a struct used by the proc:
    /// scheme to read/write registers.
    pub fn save(&self, all: &mut IntRegisters) {
        let r = &self.registers;
        all.pc = self.iret.epc as usize;
        all.at = r.at as usize;
        all.v0 = r.v0 as usize;
        all.v1 = r.v1 as usize;
        all.a0 = r.a0 as usize;
        all.a1 = r.a1 as usize;
        all.a2 = r.a2 as usize;
        all.a3 = r.a3 as usize;
        all.a4 = r.a4 as usize;
        all.a5 = r.a5 as usize;
        all.a6 = r.a6 as usize;
        all.a7 = r.a7 as usize;
        all.t0 = r.t0 as usize;
        all.t1 = r.t1 as usize;
        all.t2 = r.t2 as usize;
        all.t3 = r.t3 as usize;
        all.s0 = r.s0 as usize;
        all.s1 = r.s1 as usize;
        all.s2 = r.s2 as usize;
        all.s3 = r.s3 as usize;
        all.s4 = r.s4 as usize;
        all.s5 = r.s5 as usize;
        all.s6 = r.s6 as usize;
        all.s7 = r.s7 as usize;
        all.t8 = r.t8 as usize;
        all.t9 = r.t9 as usize;
        all.sp = r.sp as usize;
        all.fp = r.fp as usize;
        all.ra = r.ra as usize;
        all.hi = self.iret.hi as usize;
        all.lo = self.iret.lo as usize;
    }

    /// Loads all registers from a struct used by the proc:
    /// scheme to read/write registers.
    pub fn load(&mut self, all: &IntRegisters) {
        let r = &mut self.registers;
        self.iret.epc = sign_extend(all.pc);
        r.at = sign_extend(all.at);
        r.v0 = sign_extend(all.v0);
        r.v1 = sign_extend(all.v1);
        r.a0 = sign_extend(all.a0);
        r.a1 = sign_extend(all.a1);
        r.a2 = sign_extend(all.a2);
        r.a3 = sign_extend(all.a3);
        r.a4 = sign_extend(all.a4);
        r.a5 = sign_extend(all.a5);
        r.a6 = sign_extend(all.a6);
        r.a7 = sign_extend(all.a7);
        r.t0 = sign_extend(all.t0);
        r.t1 = sign_extend(all.t1);
        r.t2 = sign_extend(all.t2);
        r.t3 = sign_extend(all.t3);
        r.s0 = sign_extend(all.s0);
        r.s1 = sign_extend(all.s1);
        r.s2 = sign_extend(all.s2);
        r.s3 = sign_extend(all.s3);
        r.s4 = sign_extend(all.s4);
        r.s5 = sign_extend(all.s5);
        r.s6 = sign_extend(all.s6);
        r.s7 = sign_extend(all.s7);
        r.t8 = sign_extend(all.t8);
        r.t9 = sign_extend(all.t9);
        r.sp = sign_extend(all.sp);
        r.fp = sign_extend(all.fp);
        r.ra = sign_extend(all.ra);
        self.iret.hi = sign_extend(all.hi);
        self.iret.lo = sign_extend(all.lo);
    }

    //TODO: single step through the Watch registers or a breakpoint at EPC + 4
    pub fn is_singlestep(&self) -> bool {
        false
    }
    pub fn set_singlestep(&mut self, _singlestep: bool) {}
}

impl ArchIntCtx for InterruptStack {
    fn ip(&self) -> usize {
        self.iret.epc as usize
    }
    fn recover_and_efault(&mut self) {
        // Set the return value to nonzero to indicate usercopy failure (EFAULT), and emulate the
        // return instruction by setting the return pointer to the saved RA value.
        self.iret.epc = self.registers.ra;
        self.registers.v0 = 1;
    }
}

/// Store all general registers except `$sp` (kept by the caller) into the
/// frame at `$sp`, then the CP0 state. Expects `$sp` already lowered by
/// `FRAME_SIZE`. `$k1` holds the original stack pointer.
#[macro_export]
macro_rules! push_registers {
    () => {
        "
    sd      $1, 0($sp)
    sd      $2, 8($sp)
    sd      $3, 16($sp)
    sd      $4, 24($sp)
    sd      $5, 32($sp)
    sd      $6, 40($sp)
    sd      $7, 48($sp)
    sd      $8, 56($sp)
    sd      $9, 64($sp)
    sd      $10, 72($sp)
    sd      $11, 80($sp)
    sd      $12, 88($sp)
    sd      $13, 96($sp)
    sd      $14, 104($sp)
    sd      $15, 112($sp)
    sd      $16, 120($sp)
    sd      $17, 128($sp)
    sd      $18, 136($sp)
    sd      $19, 144($sp)
    sd      $20, 152($sp)
    sd      $21, 160($sp)
    sd      $22, 168($sp)
    sd      $23, 176($sp)
    sd      $24, 184($sp)
    sd      $25, 192($sp)
    sd      $zero, 200($sp)
    dmfc0   $8, $30
    sd      $8, 208($sp)
    sd      $28, 216($sp)
    sd      $27, 224($sp)
    sd      $30, 232($sp)
    sd      $31, 240($sp)
    dmfc0   $8, $14
    sd      $8, 248($sp)
    mfc0    $8, $12
    sd      $8, 256($sp)
    mfc0    $8, $13
    sd      $8, 264($sp)
    dmfc0   $8, $8
    sd      $8, 272($sp)
    mfhi    $8
    sd      $8, 280($sp)
    mflo    $8
    sd      $8, 288($sp)
    "
    };
}

/// Restore CP0 state and every general register from the frame at `$sp`,
/// loading `$sp` itself last.
#[macro_export]
macro_rules! pop_registers {
    () => {
        "
    ld      $8, 280($sp)
    mthi    $8
    ld      $8, 288($sp)
    mtlo    $8
    ld      $8, 248($sp)
    dmtc0   $8, $14
    // Status takes effect three instructions late (interrupts sample it at
    // stage 3). Write it with IE clear, wait out the window, then write IE
    // back. EXL is already set by the first write, so no interrupt can
    // arrive with IE set and EXL not yet.
    ld      $8, 256($sp)
    addiu   $9, $zero, -2
    and     $9, $8, $9
    mtc0    $9, $12
    nop
    nop
    nop
    mtc0    $8, $12
    nop
    nop
    ld      $1, 0($sp)
    ld      $2, 8($sp)
    ld      $3, 16($sp)
    ld      $4, 24($sp)
    ld      $5, 32($sp)
    ld      $6, 40($sp)
    ld      $7, 48($sp)
    ld      $8, 56($sp)
    ld      $9, 64($sp)
    ld      $10, 72($sp)
    ld      $11, 80($sp)
    ld      $12, 88($sp)
    ld      $13, 96($sp)
    ld      $14, 104($sp)
    ld      $15, 112($sp)
    ld      $16, 120($sp)
    ld      $17, 128($sp)
    ld      $18, 136($sp)
    ld      $19, 144($sp)
    ld      $20, 152($sp)
    ld      $21, 160($sp)
    ld      $22, 168($sp)
    ld      $23, 176($sp)
    ld      $24, 184($sp)
    ld      $25, 192($sp)
    ld      $27, 208($sp)
    ld      $28, 216($sp)
    ld      $30, 232($sp)
    ld      $31, 240($sp)
    ld      $29, 224($sp)
    "
    };
}

/// Enter user mode for the first time on this context's kernel stack.
///
/// Reached from `switch_to_inner` with `$ra` = this function and `$s7` =
/// the kernel function to run first (`setup_initial_call`). The trap frame
/// prepared below `Kstack::initial_top` is then restored and `eret` enters
/// user mode.
#[unsafe(naked)]
pub unsafe extern "C" fn enter_usermode() -> ! {
    core::arch::naked_asm!(
        "
        .set noreorder
        .set noat
        move    $25, $23
        jalr    $25
        nop
        // First entry to user mode: its code was written through other
        // addresses, so start with empty caches.
        la      $25, {flush}
        jalr    $25
        nop
        // Record this context's kernel stack top for the next user trap.
        la      $8, PERCPU_PTR
        lw      $8, 0($8)
        addiu   $9, $sp, {frame_size}
        sw      $9, 4($8)
        ",
        pop_registers!(),
        "
        eret
        nop
        .set at
        .set reorder
        ",
        frame_size = const FRAME_SIZE,
        flush = sym crate::arch::cache::flush_for_user,
    )
}
