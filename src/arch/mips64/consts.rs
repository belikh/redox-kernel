//! Fixed virtual addresses. Physical RAM starts at 0x0800_0000 and is reached
//! through kseg0 (`PHYS_OFFSET` 0x8000_0000) and kseg1 for devices.

/// Offset to kernel heap: the start of kseg2, TLB mapped. The heap is a fixed
/// 1 MiB (`allocator::KERNEL_HEAP_SIZE`).
#[inline(always)]
pub fn kernel_heap_offset() -> usize {
    0xC000_0000
}

/// End of user mappings, 64 KiB below the end of useg. The thread pointer is
/// the TLS block plus 0x7000 and is used with negative offsets; with
/// Status.UX set the CPU adds base and offset in 64 bits, so a thread
/// pointer at or above 0x8000_0000 (sign-extended in its register) gives an
/// address error. Nothing may be mapped where that sum could cross.
pub const USER_END_OFFSET: usize = 0x7fff_0000;

/// Physical base of DRAM on the Indy.
pub const RAM_PHYS_BASE: usize = 0x0800_0000;

/// Physical bytes below the kernel reserved for the PROM, the exception
/// vectors (the first 512 KiB of RAM is aliased at physical 0) and the ARCS
/// parameter block.
pub const LOW_RESERVED_BYTES: usize = 0x0004_0000;
