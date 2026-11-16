//! SGI Indy (IP24): MIPS R4400, n32 kernel with 32-bit addresses, big-endian.
//!
//! Address space: useg (0..0x8000_0000) is user and TLB mapped; kseg0
//! (0x8000_0000) is the cached direct map the kernel runs in; kseg1
//! (0xA000_0000) is the uncached direct map used for device registers; kseg2
//! (0xC000_0000) holds TLB-mapped kernel memory: the heap and kernel stacks.

/// Constants like memory locations
pub mod consts;

/// CP0 register access
pub mod cache;
pub mod cp0;

/// Debugging support
pub mod debug;

/// Devices
pub mod device;

/// Interrupt instructions
#[macro_use]
pub mod interrupt;

/// Inter-processor interrupts (one CPU: no-ops)
pub mod ipi;

/// Per-CPU data
pub mod misc;

/// Paging
pub mod paging;

/// Initialization and start function
pub mod start;

/// Stop function
pub mod stop;

pub mod time;

pub use ::rmm::mips::MipsArch as CurrentRmmArch;
use core::arch::naked_asm;

pub use arch_copy_to_user as arch_copy_from_user;

/// Copy `len` bytes from `src` to `dst`, one of them in userspace, in 64-bit
/// words once the destination is aligned. A leaf that leaves `$ra` and `$sp`
/// alone: the fault path returns through `$ra`. A fault
/// inside `__usercopy_start..__usercopy_end` is turned into a non-zero
/// return by the page fault handler (`recover_and_efault`).
#[unsafe(naked)]
pub unsafe extern "C" fn arch_copy_to_user(dst: usize, src: usize, len: usize) -> u8 {
    naked_asm!(
        "
        .set noreorder
    .global __usercopy_start
    __usercopy_start:
        sltiu   $8, $6, 16
        bnez    $8, 7f                  // short: bytes only
        nop
        // Bytes until the destination is 8-byte aligned.
        negu    $8, $4
        andi    $8, $8, 7
        beqz    $8, 2f
        subu    $6, $6, $8
    1:  lbu     $9, 0($5)
        addiu   $5, $5, 1
        addiu   $8, $8, -1
        sb      $9, 0($4)
        bnez    $8, 1b
        addiu   $4, $4, 1
    2:  srl     $10, $6, 3              // whole 64-bit words
        andi    $9, $5, 7
        bnez    $9, 5f                  // source not aligned
        andi    $6, $6, 7               // bytes after the words
        srl     $11, $10, 2             // rounds of four words
        beqz    $11, 4f
        andi    $10, $10, 3
    3:  ld      $8, 0($5)
        ld      $9, 8($5)
        ld      $12, 16($5)
        ld      $13, 24($5)
        addiu   $5, $5, 32
        sd      $8, 0($4)
        sd      $9, 8($4)
        sd      $12, 16($4)
        sd      $13, 24($4)
        addiu   $11, $11, -1
        bnez    $11, 3b
        addiu   $4, $4, 32
    4:  beqz    $10, 7f
        nop
    41: ld      $8, 0($5)
        addiu   $5, $5, 8
        addiu   $10, $10, -1
        sd      $8, 0($4)
        bnez    $10, 41b
        addiu   $4, $4, 8
        b       7f
        nop
    5:  beqz    $10, 7f
        nop
    6:  ldl     $8, 0($5)               // big-endian unaligned load
        ldr     $8, 7($5)
        addiu   $5, $5, 8
        addiu   $10, $10, -1
        sd      $8, 0($4)
        bnez    $10, 6b
        addiu   $4, $4, 8
    7:  beqz    $6, 9f
        nop
    8:  lbu     $8, 0($5)
        addiu   $5, $5, 1
        addiu   $6, $6, -1
        sb      $8, 0($4)
        bnez    $6, 8b
        addiu   $4, $4, 1
    9:  jr      $31
        move    $2, $zero
    .global __usercopy_end
    __usercopy_end:
        .set reorder
    "
    )
}

pub const KFX_SIZE: usize = 512;

// This function exists as the KFX size is dynamic on x86_64.
pub fn kfx_size() -> usize {
    KFX_SIZE
}

#[inline]
pub fn current_stack_ptr() -> *mut u8 {
    let ret: *mut u8;
    unsafe {
        core::arch::asm!(
            "move {}, $sp",
            out(reg) ret,
            options(nomem, preserves_flags, nostack),
        );
    }
    ret
}
