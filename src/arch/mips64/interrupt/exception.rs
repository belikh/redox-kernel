//! Exception vectors and the common handler.
//!
//! The R4400 takes every exception at one of three kseg0 vectors when
//! `Status.BEV` is clear: TLB refill at 0x8000_0000, extended refill at
//! 0x8000_0080 and the general vector at 0x8000_0180. On the Indy physical
//! 0 aliases the first 512 KiB of RAM, so the vectors are RAM. Each vector
//! is a jump to `exception_entry`, which saves the frame and calls
//! `exception_handler_inner`.

use ::syscall::Exception;
use core::arch::naked_asm;
use rmm::{Arch, VirtualAddress};

use crate::{
    arch::cp0::{self, cause},
    context::signal::excp_handler,
    memory::{GenericPfFlags, RmmA},
    ptrace,
    sync::CleanLockToken,
    syscall::{self, flag::*},
};

use super::{handler::FRAME_SIZE, InterruptStack};

const VECTOR_TLB_REFILL: usize = 0x8000_0000;
const VECTOR_XTLB_REFILL: usize = 0x8000_0080;
const VECTOR_GENERAL: usize = 0x8000_0180;

/// Copy the vector stub to the three vector addresses.
///
/// The stub is written through kseg1, so RAM holds it at once and no data
/// cache geometry matters. Before the write, any cached copy of the line is
/// written back and dropped (`Hit_Writeback_Inv_D`), so a stale dirty line
/// cannot land on top of the stub later. After it, `Hit_Invalidate_I` makes
/// the next fetch read RAM. Hit operations match by address, so they are
/// right for the R4400 (16-byte lines, direct mapped) and for the R4600 and
/// R5000 (32-byte lines, two ways). The load before the write-back is the
/// workaround early R4600 revisions need for that operation.
pub unsafe fn install_vectors() {
    unsafe {
        for vector in [VECTOR_TLB_REFILL, VECTOR_XTLB_REFILL, VECTOR_GENERAL] {
            // Refills go to the assembly handler, everything else to the
            // general entry.
            let stub = if vector == VECTOR_GENERAL {
                vector_stub as usize
            } else {
                refill_stub as usize
            } as *const u32;
            let _ = (vector as *const u32).read_volatile();
            core::arch::asm!(
                "cache 21, 0({0})", // Hit_Writeback_Inv_D
                in(reg) vector,
                options(nostack)
            );
            let dst = (vector | 0x2000_0000) as *mut u32;
            for i in 0..VECTOR_STUB_WORDS {
                dst.add(i).write_volatile(stub.add(i).read_volatile());
            }
            core::arch::asm!(
                "cache 16, 0({0})", // Hit_Invalidate_I
                in(reg) vector,
                options(nostack)
            );
        }
    }
}

/// Instruction words in `vector_stub`.
const VECTOR_STUB_WORDS: usize = 4;

/// Position-independent: `la` expands to lui/ori with the absolute address.
#[unsafe(naked)]
unsafe extern "C" fn vector_stub() {
    naked_asm!(
        "
        .set noreorder
        .set noat
        la      $26, {entry}
        jr      $26
        nop
        .set at
        .set reorder
        ",
        entry = sym exception_entry,
    )
}

#[unsafe(naked)]
unsafe extern "C" fn refill_stub() {
    naked_asm!(
        "
        .set noreorder
        .set noat
        la      $26, {entry}
        jr      $26
        nop
        .set at
        .set reorder
        ",
        entry = sym refill_entry,
    )
}

/// `$1`..`$4` of the interrupted code while `refill_entry` runs. The handler
/// runs at exception level on one CPU, so one copy is enough.
static REFILL_SCRATCH: core::cell::SyncUnsafeCell<[u64; 4]> =
    core::cell::SyncUnsafeCell::new([0; 4]);

/// TLB refill in assembly, for the entries the slow path has marked as
/// needing no cache work (rmm::mips::PTE_FAST_*). It walks the two-level
/// table through kseg0, checks both halves of the pair and writes a random
/// entry. A refill exception means no entry matches, so no probe is needed.
/// Anything else goes to `exception_entry` with every register as it came.
///
/// A half is taken when it is not present (EntryLo 0), or present, cached,
/// and one of: a kernel address with FAST_DATA; a user page (USER bit) on
/// its frame's colour with NO_EXEC and FAST_DATA; the same with execute
/// permission, not writable, and FAST_TEXT.
#[unsafe(naked)]
unsafe extern "C" fn refill_entry() {
    naked_asm!(
        "
        .set noreorder
        .set noat
        dmtc0   $27, $30                // keep the user's k1 (see exception_entry)
        la      $26, {scratch}
        sd      $1, 0($26)
        sd      $2, 8($26)
        sd      $3, 16($26)
        sd      $4, 24($26)

        mfc0    $1, $8                  // BadVAddr
        la      $27, {table}
        lw      $27, 0($27)             // directory, physical
        srl     $2, $1, 22
        sll     $2, $2, 2
        addu    $27, $27, $2
        lui     $3, 0x8000
        or      $27, $27, $3
        lw      $27, 0($27)             // directory entry
        andi    $2, $27, 2
        beqz    $2, 9f
        lui     $2, 0x03ff
        ori     $2, $2, 0xffc0
        and     $27, $27, $2
        sll     $27, $27, 6
        or      $27, $27, $3            // table, kseg0
        srl     $2, $1, 13
        andi    $2, $2, 0x1ff
        sll     $2, $2, 3
        addu    $27, $27, $2            // the even entry of the pair
        la      $26, {colour_mask}
        lw      $26, 0($26)

        // Even half.
        lw      $2, 0($27)
        andi    $3, $2, 2
        beqz    $3, 2f
        move    $4, $zero               // not present: EntryLo 0
        lui     $3, 0x2000              // uncached pages take the slow path
        and     $3, $3, $2
        bnez    $3, 9f
        nop
        bltz    $1, 1f                  // kernel address: FAST_DATA is enough
        nop
        bgez    $2, 9f                  // user address without the USER bit
        srl     $3, $1, 13
        sll     $3, $3, 1               // page number, even half
        srl     $4, $2, 6
        xor     $3, $3, $4
        and     $3, $3, $26
        bnez    $3, 9f                  // off colour
        sll     $3, $2, 1
        bltz    $3, 1f                  // NO_EXEC: a data page
        andi    $3, $2, 4
        bnez    $3, 9f                  // writable and executable
        lui     $3, 0x0800
        and     $3, $3, $2
        beqz    $3, 9f                  // text not marked FAST_TEXT
        nop
        b       3f
        nop
    1:  lui     $3, 0x0400
        and     $3, $3, $2
        beqz    $3, 9f                  // not marked FAST_DATA
        nop
    3:  lui     $3, 0x03ff
        ori     $3, $3, 0xffc6
        and     $4, $2, $3
        ori     $4, $4, 0x18            // cacheable, noncoherent
    2:  srl     $3, $1, 31              // G for a kernel address
        or      $4, $4, $3
        mtc0    $4, $2                  // EntryLo0

        // Odd half.
        lw      $2, 4($27)
        andi    $3, $2, 2
        beqz    $3, 2f
        move    $4, $zero
        lui     $3, 0x2000
        and     $3, $3, $2
        bnez    $3, 9f
        nop
        bltz    $1, 1f
        nop
        bgez    $2, 9f
        srl     $3, $1, 13
        sll     $3, $3, 1
        ori     $3, $3, 1               // page number, odd half
        srl     $4, $2, 6
        xor     $3, $3, $4
        and     $3, $3, $26
        bnez    $3, 9f
        sll     $3, $2, 1
        bltz    $3, 1f
        andi    $3, $2, 4
        bnez    $3, 9f
        lui     $3, 0x0800
        and     $3, $3, $2
        beqz    $3, 9f
        nop
        b       3f
        nop
    1:  lui     $3, 0x0400
        and     $3, $3, $2
        beqz    $3, 9f
        nop
    3:  lui     $3, 0x03ff
        ori     $3, $3, 0xffc6
        and     $4, $2, $3
        ori     $4, $4, 0x18
    2:  srl     $3, $1, 31
        or      $4, $4, $3
        mtc0    $4, $3                  // EntryLo1
        la      $26, {fast_count}
        lw      $2, 0($26)
        addiu   $2, $2, 1
        sw      $2, 0($26)
        tlbwr
        nop
        la      $26, {scratch}
        ld      $1, 0($26)
        ld      $2, 8($26)
        ld      $3, 16($26)
        ld      $4, 24($26)
        dmfc0   $27, $30
        eret
        nop

        // The slow path, with every register restored.
    9:  la      $26, {scratch}
        ld      $1, 0($26)
        ld      $2, 8($26)
        ld      $3, 16($26)
        ld      $4, 24($26)
        la      $26, {slow}
        jr      $26
        dmfc0   $27, $30
        .set at
        .set reorder
        ",
        scratch = sym REFILL_SCRATCH,
        table = sym rmm::mips::CURRENT_TABLE,
        colour_mask = sym crate::arch::cache::COLOUR_MASK,
        fast_count = sym crate::arch::cache::STAT_FAST_REFILLS,
        slow = sym exception_entry,
    )
}

#[unsafe(naked)]
pub unsafe extern "C" fn exception_entry() {
    naked_asm!(
        "
        .set noreorder
        .set noat
        // $27 (k1) is preserved for user code: relibc's signal trampoline
        // needs one register no compiled code uses, to hold its last jump
        // target. Stash it in ErrorEPC, which only reset, NMI and cache
        // errors write; push_registers moves it into the frame before EXL
        // is cleared. $26 (k0) stays kernel scratch.
        dmtc0   $27, $30
        // Virtual coherency exceptions (R4000/R4400 with a secondary cache):
        // the page was last cached under another virtual colour. Drop the
        // primary line at this index, write the secondary line back and
        // retry. Only $26/$27 are used, so no frame is needed.
        mfc0    $26, $13
        andi    $26, $26, 0x7c
        xori    $27, $26, 0x7c          // 31: VCED
        beqz    $27, 8f
        xori    $27, $26, 0x38          // 14: VCEI
        beqz    $27, 9f
        nop
        move    $27, $sp
        mfc0    $26, $12
        andi    $26, $26, 0x18
        beqz    $26, 1f
        nop
        // From user mode: take this context's kernel stack.
        la      $26, PERCPU_PTR
        lw      $26, 0($26)
        lw      $sp, 4($26)
    1:
        addiu   $sp, $sp, -{frame_size}
        ",
        push_registers!(),
        "
        // Leave exception level so a nested TLB miss updates EPC; stay in
        // kernel mode with interrupts off. The frame keeps the real Status.
        mfc0    $8, $12
        li      $9, ~0x1b
        and     $8, $8, $9
        mtc0    $8, $12
        nop
        nop
        move    $4, $sp
        la      $25, {handler}
        jalr    $25
        nop
        // Returning to user mode: remember the kernel stack top for the next trap.
        ld      $8, 256($sp)
        andi    $8, $8, 0x18
        beqz    $8, 2f
        nop
        la      $8, PERCPU_PTR
        lw      $8, 0($8)
        addiu   $9, $sp, {frame_size}
        sw      $9, 4($8)
    2:
        ",
        pop_registers!(),
        "
        eret
        nop
    8:
        mfc0    $26, $8
        li      $27, -4
        and     $26, $26, $27
        mtc0    $zero, $28
        nop
        cache   9, 0($26)               // Index_Store_Tag_D
        cache   0x17, 0($26)            // Hit_Writeback_Inv_SD
        dmfc0   $27, $30
        eret
        nop
    9:
        mfc0    $26, $8
        nop
        cache   0x17, 0($26)            // Hit_Writeback_Inv_SD
        dmfc0   $27, $30
        eret
        nop
        .set at
        .set reorder
        ",
        frame_size = const FRAME_SIZE,
        handler = sym exception_handler_inner,
    )
}

unsafe extern "C" fn exception_handler_inner(regs: &mut InterruptStack) {
    unsafe {
        let cause_reg = regs.iret.cause as u32;
        let exc_code = (cause_reg & cause::EXC_CODE_MASK) >> cause::EXC_CODE_SHIFT;
        let user_mode = regs.iret.status as u32 & cp0::status::KSU_MASK != 0;
        let _timing = TrapTiming::start(exc_code as usize, user_mode);

        // See arch::cache: frames are about to be touched, except by a refill
        // served from the page table.
        let is_tlb = matches!(exc_code, cause::EXC_MOD | cause::EXC_TLBL | cause::EXC_TLBS);
        if is_tlb {
            match tlb_fault(regs, exc_code, user_mode) {
                TlbOutcome::Refilled => return,
                TlbOutcome::Handled => {}
                TlbOutcome::Failed if user_mode => handle_user_exception(regs, exc_code),
                TlbOutcome::Failed => handle_system_exception(regs, exc_code),
            }
        } else {
            crate::arch::cache::before_frames();
            match exc_code {
                cause::EXC_INT => handle_interrupt(regs, cause_reg),
                _ if user_mode => handle_user_exception(regs, exc_code),
                _ => handle_system_exception(regs, exc_code),
            }
        }
    }
}

/// Read one word of the current address space through the page table and
/// kseg0, without touching the TLB. RAM is current: the data cache was
/// written back on the way in.
fn user_peek(addr: usize) -> Option<u32> {
    unsafe {
        let root = RmmA::table(rmm::TableKind::User);
        let directory = RmmA::phys_to_virt(root).data() as *const usize;
        let pde = directory.add(addr >> 22).read_volatile();
        if pde & RmmA::ENTRY_FLAG_PRESENT == 0 {
            return None;
        }
        let table_phys =
            ((pde >> RmmA::ENTRY_ADDRESS_SHIFT) & RmmA::ENTRY_ADDRESS_MASK) << RmmA::PAGE_SHIFT;
        let table =
            RmmA::phys_to_virt(rmm::PhysicalAddress::new(table_phys)).data() as *const usize;
        let pte = table.add((addr >> 12) & 0x3ff).read_volatile();
        if pte & RmmA::ENTRY_FLAG_PRESENT == 0 {
            return None;
        }
        let frame =
            ((pte >> RmmA::ENTRY_ADDRESS_SHIFT) & RmmA::ENTRY_ADDRESS_MASK) << RmmA::PAGE_SHIFT;
        let virt = RmmA::phys_to_virt(rmm::PhysicalAddress::new(frame + (addr & 0xffc))).data();
        Some((virt as *const u32).read_volatile())
    }
}

/// Print `count` words of user memory, eight to a line, for a fault report.
fn dump_user_words(what: &str, start: usize, count: usize) {
    if start >= crate::USER_END_OFFSET {
        return;
    }
    println!("{} ({:#x}):", what, start);
    for row in 0..count.div_ceil(8) {
        for col in 0..8 {
            let index = row * 8 + col;
            if index >= count {
                break;
            }
            match user_peek(start + index * 4) {
                Some(word) => print!(" {:08X}", word),
                None => print!(" --------"),
            }
        }
        println!();
    }
}

/// Kernel time by trap code, in CP0 Count ticks, for tuning. Time is
/// charged from a stamp: set at trap entry and when a context is switched
/// in, charged at trap exit and at switch-out. Bucket 31 is time after a
/// switch-in, when the code of the resumed trap is not known here.
pub static TIMING_STAMP: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static TIMING_CODE: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(31);

pub fn timing_charge() {
    use core::sync::atomic::Ordering;
    if !crate::arch::cache::TRAP_TIMING {
        return;
    }
    let now = cp0::read_count();
    let ticks = now.wrapping_sub(TIMING_STAMP.swap(now, Ordering::Relaxed)) as usize;
    let code = TIMING_CODE.load(Ordering::Relaxed) % 288;
    crate::arch::cache::STAT_TICKS[code].fetch_add(ticks, Ordering::Relaxed);
}

struct TrapTiming {
    outer: usize,
}

impl TrapTiming {
    fn start(code: usize, from_user: bool) -> Self {
        use core::sync::atomic::Ordering;
        if !crate::arch::cache::TRAP_TIMING {
            return TrapTiming { outer: 0 };
        }
        if from_user {
            TIMING_STAMP.store(cp0::read_count(), Ordering::Relaxed);
        } else {
            // A trap taken in kernel mode interrupts another trap's time.
            timing_charge();
        }
        crate::arch::cache::STAT_BY_CODE[code & 31].fetch_add(1, Ordering::Relaxed);
        TrapTiming {
            outer: TIMING_CODE.swap(code, Ordering::Relaxed),
        }
    }
}

impl Drop for TrapTiming {
    fn drop(&mut self) {
        use core::sync::atomic::Ordering;
        if !crate::arch::cache::TRAP_TIMING {
            return;
        }
        timing_charge();
        TIMING_CODE.store(self.outer, Ordering::Relaxed);
    }
}

enum TlbOutcome {
    /// The pair was in the page table; only the TLB was written.
    Refilled,
    /// The generic page fault handler changed the mapping.
    Handled,
    Failed,
}

fn handle_interrupt(regs: &mut InterruptStack, cause_reg: u32) {
    let pending = cause_reg & regs.iret.status as u32 & cause::IP_MASK;
    let mut token = unsafe { CleanLockToken::new() };
    if pending & (cause::IP2 | cause::IP3) != 0 {
        unsafe { super::irq::handle(pending, &mut token) };
    }
    if pending & cause::IP7 != 0 {
        crate::arch::time::profile_sample(
            regs.iret.epc as usize,
            regs.iret.status as u32 & cp0::status::KSU_MASK != 0,
        );
        crate::arch::time::tick();
        // The keyboard and the serial port interrupt (irq.rs). The tick
        // delivers the screen's replies to terminal reports, and once a
        // second looks at both devices in case an interrupt was lost;
        // FALLBACK_INPUT counts what that finds.
        static CONSOLE_TICK: core::sync::atomic::AtomicU32 =
            core::sync::atomic::AtomicU32::new(0);
        let tick = CONSOLE_TICK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        if tick % 100 == 0 {
            let found = crate::arch::device::keyboard_input(&mut token);
            crate::arch::device::FALLBACK_INPUT
                .fetch_add(found, core::sync::atomic::Ordering::Relaxed);
            crate::arch::device::serial_service(&mut token);
        } else if crate::arch::device::newport::active() {
            let mut reply = [0u8; 32];
            let n = crate::arch::device::newport::take_reply(&mut reply);
            for &byte in &reply[..n] {
                crate::scheme::debug::debug_input(byte, &mut token);
            }
            if n != 0 {
                crate::scheme::debug::debug_notify(&mut token);
            }
        }
        crate::context::timeout::trigger(&mut token);
        crate::context::switch::tick(&mut token);
    }
}

unsafe fn handle_system_exception(regs: &InterruptStack, exc_code: u32) {
    error!(
        "Kernel exception {} at {:#x}, BadVAddr {:#x}",
        exc_code,
        { regs.iret.epc },
        { regs.iret.badvaddr }
    );
    regs.dump();
    // Leave the report on the screen for about a minute, then restart
    // through the PROM: a machine nobody is sitting at comes back by itself
    // with whatever image the boot server holds by then.
    println!("Indy: restarting through the PROM in about a minute");
    let start = crate::arch::cp0::read_count();
    let per_second = crate::arch::time::count_hz() as u32;
    for _ in 0..60 {
        while crate::arch::cp0::read_count().wrapping_sub(start) % per_second
            < per_second / 2
        {}
        while crate::arch::cp0::read_count().wrapping_sub(start) % per_second
            >= per_second / 2
        {}
    }
    unsafe { crate::arch::stop::emergency_reset() }
}

unsafe fn handle_user_exception(regs: &mut InterruptStack, exc_code: u32) {
    unsafe {
        let mut token = CleanLockToken::new();

        if exc_code == cause::EXC_SYS {
            let r = &mut regs.registers;
            // Skip the syscall instruction. LLVM never places it in a branch delay slot.
            regs.iret.epc += 4;
            // Kernel time per syscall number (buckets 32..96 of the stats).
            // One bucket per calling program, syscall number and first
            // argument (the file descriptor of a read or a write).
            let number = r.v0 as usize;
            if crate::arch::cache::TRAP_TIMING {
                let fd = r.a0 as usize & 0xff;
                let tag = crate::arch::cache::current_tag() as usize;
                let key = number ^ fd.wrapping_mul(0x9e37) ^ tag.wrapping_mul(0x01000193);
                let bucket = 32 + ((key ^ (key >> 8) ^ (key >> 16) ^ (key >> 24)) & 255);
                timing_charge();
                TIMING_CODE.store(bucket, core::sync::atomic::Ordering::Relaxed);
                crate::arch::cache::STAT_SYSCALL[bucket - 32]
                    .store(number, core::sync::atomic::Ordering::Relaxed);
                crate::arch::cache::STAT_WHO[bucket - 32]
                    .store((tag << 8) | fd, core::sync::atomic::Ordering::Relaxed);
                crate::arch::cache::STAT_BY_CODE[bucket]
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            let ret = syscall::syscall(
                r.v0 as usize,
                r.a0 as usize,
                r.a1 as usize,
                r.a2 as usize,
                r.a3 as usize,
                r.a4 as usize,
                r.a5 as usize,
                &mut token,
            );
            r.v0 = super::handler::sign_extend(ret);
            if crate::arch::cache::TRAP_TIMING {
                timing_charge();
                TIMING_CODE.store(8, core::sync::atomic::Ordering::Relaxed);
            }
            return;
        }

        if exc_code == cause::EXC_BP
            && ptrace::breakpoint_callback(PTRACE_STOP_BREAKPOINT, None, &mut token).is_some()
        {
            return;
        }

        // `rdhwr rt, $29` reads the user-local register, which the R4400 lacks:
        // LLVM uses it for every thread-local access, so emulate it with the
        // thread pointer ($gp). The instruction is SPECIAL3 (0x1f), function
        // RDHWR (0x3b), rd = 29.
        if exc_code == cause::EXC_RI
            && let Some(insn) = read_user_insn(regs.iret.epc as usize)
            && insn & 0xffe0_ffff == 0x7c00_e83b
        {
            let rt = ((insn >> 16) & 0x1f) as usize;
            let tp = regs.registers.gp;
            if rt != 0 && rt != 26 && rt != 27 {
                unsafe {
                    (&raw mut regs.registers as *mut u64).add(rt - 1).write(tp);
                }
            }
            regs.iret.epc += 4;
            return;
        }

        info!(
            "User exception {} at {:#x}, BadVAddr {:#x}",
            exc_code,
            { regs.iret.epc },
            { regs.iret.badvaddr }
        );
        regs.dump();
        dump_user_words("code at EPC-16", (regs.iret.epc as usize).wrapping_sub(16) & !3, 12);
        dump_user_words("code at RA-16", (regs.registers.ra as usize).wrapping_sub(16) & !3, 8);
        dump_user_words("stack at SP", regs.registers.sp as usize & !3, 32);

        excp_handler(Exception {
            code: exc_code as usize,
            address: regs.iret.badvaddr as usize,
        });
    }
}

/// Fetch the user instruction at `epc` through the user copy path, so a bad
/// address faults safely instead of taking the kernel down.
fn read_user_insn(epc: usize) -> Option<u32> {
    if epc & 3 != 0 || epc >= crate::arch::consts::USER_END_OFFSET {
        return None;
    }
    let mut word = [0u8; 4];
    let failed = unsafe {
        crate::arch::arch_copy_from_user(word.as_mut_ptr() as usize, epc, 4)
    };
    if failed != 0 {
        return None;
    }
    Some(u32::from_be_bytes(word))
}

/// Serve a TLB exception from the page table; fall back to the generic page
/// fault handler. Returns true when execution can resume at EPC.
unsafe fn tlb_fault(regs: &mut InterruptStack, exc_code: u32, user_mode: bool) -> TlbOutcome {
    unsafe {
        let badvaddr = regs.iret.badvaddr as usize;
        let page = badvaddr & !(RmmA::PAGE_SIZE - 1);
        let write = exc_code == cause::EXC_TLBS || exc_code == cause::EXC_MOD;

        // Walk the active table.
        let root = RmmA::table(rmm::TableKind::User);
        let directory = RmmA::phys_to_virt(root).data() as *const usize;
        let pde = directory.add(page >> 22).read_volatile();
        let pte = if pde & RmmA::ENTRY_FLAG_PRESENT != 0 {
            let table_phys = ((pde >> RmmA::ENTRY_ADDRESS_SHIFT) & RmmA::ENTRY_ADDRESS_MASK)
                << RmmA::PAGE_SHIFT;
            let table = RmmA::phys_to_virt(rmm::PhysicalAddress::new(table_phys)).data()
                as *const usize;
            Some(table.add((page >> 12) & 0x3ff).read_volatile())
        } else {
            None
        };

        if let Some(pte) = pte
            && pte & RmmA::ENTRY_FLAG_PRESENT != 0
            && (!user_mode || pte & RmmA::ENTRY_FLAG_PAGE_USER != 0)
            && (!write || pte & RmmA::ENTRY_FLAG_READWRITE != 0)
            && exc_code != cause::EXC_MOD
        {
            // Present in the table, missing from the TLB: refill the pair.
            let table_phys = ((pde >> RmmA::ENTRY_ADDRESS_SHIFT) & RmmA::ENTRY_ADDRESS_MASK)
                << RmmA::PAGE_SHIFT;
            let table = RmmA::phys_to_virt(rmm::PhysicalAddress::new(table_phys)).data()
                as *const usize;
            let even = (page >> 12) & 0x3fe;
            let pte0 = table.add(even).read_volatile();
            let pte1 = table.add(even + 1).read_volatile();
            let lo0 = pte_to_entry_lo(pte0, user_mode);
            let lo1 = pte_to_entry_lo(pte1, user_mode);
            // Both halves become usable: bring each one's caches in order.
            let pair = page & !0x1fff;
            if lo0 != 0 {
                let fast = crate::arch::cache::installing(pair, pte0);
                if fast != 0 && pte0 & fast == 0 {
                    (table.add(even) as *mut usize).write_volatile(pte0 | fast);
                }
            }
            if lo1 != 0 {
                let fast = crate::arch::cache::installing(pair + 0x1000, pte1);
                if fast != 0 && pte1 & fast == 0 {
                    (table.add(even + 1) as *mut usize).write_volatile(pte1 | fast);
                }
            }
            rmm::mips::write_random(page, lo0, lo1);
            return TlbOutcome::Refilled;
        }

        let present = pte.is_some_and(|pte| pte & RmmA::ENTRY_FLAG_PRESENT != 0);
        let mut flags = GenericPfFlags::empty();
        flags.set(GenericPfFlags::PRESENT, present);
        flags.set(GenericPfFlags::INVOLVED_WRITE, write);
        flags.set(GenericPfFlags::USER_NOT_SUPERVISOR, user_mode);
        flags.set(
            GenericPfFlags::INSTR_NOT_DATA,
            regs.iret.epc as usize & !3 == badvaddr & !3,
        );

        // The handler reads and fills frames through kseg0; a frame it
        // allocates should have the colour of the faulting page.
        crate::arch::cache::before_frames();
        crate::arch::cache::set_alloc_colour(Some(badvaddr));
        let ok =
            crate::memory::page_fault_handler(regs, flags, VirtualAddress::new(badvaddr)).is_ok();
        crate::arch::cache::set_alloc_colour(None);
        if ok {
            // The handler changed the table; drop any stale TLB entry so the
            // retried access refills from the table.
            RmmA::invalidate(VirtualAddress::new(page));
            TlbOutcome::Handled
        } else {
            TlbOutcome::Failed
        }
    }
}

/// A page the user may not touch is invisible to it: both halves of a pair
/// are written, so a kernel-only page next to a user page must not leak.
fn pte_to_entry_lo(pte: usize, user_mode: bool) -> usize {
    if pte & RmmA::ENTRY_FLAG_PRESENT == 0 {
        return 0;
    }
    if user_mode && pte & RmmA::ENTRY_FLAG_PAGE_USER == 0 {
        return 0;
    }
    rmm::mips::entry_lo(pte)
}
