//! MIPS R4000-class paging for the SGI Indy (R4400, n32 kernel, 32-bit addresses).
//!
//! The TLB is refilled by software. `rmm` keeps a two-level table of 4-byte
//! entries (1024 directory entries, 1024 page entries each, 4 KiB pages) and
//! the kernel's exception handler walks it. A leaf entry holds the physical
//! frame number at bits 6..25, the EntryLo `V` (present) and `D` (writable)
//! bits at bits 1 and 2, and software bits above the frame number: 29
//! uncached, 30 no-execute, 31 user. The refill handler builds EntryLo from
//! the frame number and the cache bit; the other software bits never reach
//! the hardware.
//!
//! There is one CPU and no page-table base register, so the active table is a
//! static. All mappings use ASID 0 and the whole TLB is flushed on a table
//! switch.

use core::{
    arch::asm,
    sync::atomic::{AtomicUsize, Ordering},
};

use crate::{Arch, PhysicalAddress, TableKind, VirtualAddress};

#[derive(Clone, Copy)]
pub struct MipsArch;

/// Whole-TLB flushes since the last statistics read.
pub static TLB_FLUSHES: AtomicUsize = AtomicUsize::new(0);

/// Physical address of the table the refill handler walks.
pub static CURRENT_TABLE: AtomicUsize = AtomicUsize::new(0);

/// Number of TLB entries on the R4400.
pub const TLB_ENTRIES: usize = 48;

/// Software bit: build the EntryLo with the uncached cache coherency code.
pub const PTE_UNCACHED: usize = 1 << 29;
/// Software bits the slow refill path sets on a leaf entry to tell the
/// assembly refill handler that installing it needs no cache work. Each is
/// only honoured with the protection it was decided for: `PTE_FAST_DATA`
/// on a no-execute entry, `PTE_FAST_TEXT` on an executable read-only one.
/// A changed protection therefore sends the entry back to the slow path.
pub const PTE_FAST_DATA: usize = 1 << 26;
pub const PTE_FAST_TEXT: usize = 1 << 27;
/// The bits of a leaf entry the hardware understands: frame number, D, V.
pub const PTE_HARDWARE_MASK: usize = 0x03ff_ffc6;
/// EntryLo coherency field: cacheable, noncoherent.
pub const ENTRYLO_CACHEABLE: usize = 3 << 3;
/// EntryLo coherency field: uncached.
pub const ENTRYLO_UNCACHED: usize = 2 << 3;

/// Bring-up experiment: map every page uncached. Together with an uncached
/// kseg0 (Config.K0, set by the kernel) this takes the primary caches out of
/// the picture, to tell a cache fault from a translation fault on real
/// hardware. Slow.
pub const FORCE_UNCACHED: bool = false;

/// Convert a leaf entry to the EntryLo value the refill handler writes.
#[inline(always)]
pub fn entry_lo(pte: usize) -> usize {
    let cache = if FORCE_UNCACHED || pte & PTE_UNCACHED != 0 {
        ENTRYLO_UNCACHED
    } else {
        ENTRYLO_CACHEABLE
    };
    (pte & PTE_HARDWARE_MASK) | cache
}

/// An EntryHi that can never match: a kseg0 address with ASID 0. kseg0 is not
/// TLB mapped, so an entry tagged with it is as good as empty.
#[inline(always)]
fn unmapped_entry_hi(index: usize) -> usize {
    0x8000_0000 + index * 0x2000
}

/// Write one TLB entry at `index`.
///
/// # Safety
/// Changes address translation for the whole CPU.
pub unsafe fn write_indexed(index: usize, entry_hi: usize, entry_lo0: usize, entry_lo1: usize) {
    unsafe {
        asm!(
            "mtc0 {index}, $0",
            "mtc0 {hi}, $10",
            "mtc0 {lo0}, $2",
            "mtc0 {lo1}, $3",
            "nop",
            "nop",
            "tlbwi",
            "nop",
            "nop",
            index = in(reg) index,
            hi = in(reg) entry_hi,
            lo0 = in(reg) entry_lo0,
            lo1 = in(reg) entry_lo1,
            options(nostack),
        );
    }
}

/// Write the TLB entry for the pair of pages containing `virt`.
///
/// The TLB is probed first. A pair can already be present with this half
/// invalid (its neighbour was refilled before this page was mapped), and a
/// second entry for the same pair is a duplicate match: real R4x00 CPUs give
/// undefined translations or shut the TLB down. A present entry is replaced
/// in place; otherwise a random one is taken.
///
/// # Safety
/// Changes address translation for the whole CPU.
pub unsafe fn write_random(virt: usize, entry_lo0: usize, entry_lo1: usize) {
    // Kernel pages (kseg2) are the same in every address space: the G bit in
    // both halves makes the entry match under any ASID.
    let global = usize::from(virt >= 0x8000_0000) * ENTRYLO_GLOBAL;
    unsafe {
        asm!(
            ".set push",
            ".set noreorder",
            "mtc0 {hi}, $10",
            "nop",
            "nop",
            "tlbp",
            "nop",
            "nop",
            "mfc0 {probe}, $0",
            "mtc0 {lo0}, $2",
            "mtc0 {lo1}, $3",
            "nop",
            "bltz {probe}, 2f",
            "nop",
            "tlbwi",
            "b 3f",
            "nop",
            "2:",
            "tlbwr",
            "3:",
            "nop",
            "nop",
            ".set pop",
            hi = in(reg) (virt & !0x1fff) | CURRENT_ASID.load(Ordering::Relaxed),
            lo0 = in(reg) entry_lo0 | global,
            lo1 = in(reg) entry_lo1 | global,
            probe = out(reg) _,
            options(nostack),
        );
    }
}

/// EntryLo G: with it set in both halves the entry ignores the ASID.
pub const ENTRYLO_GLOBAL: usize = 1;

/// The address space identifier in EntryHi: TLB entries of user pages carry
/// the one they were written under and match only under it, so a context
/// switch changes this register and leaves the TLB alone. 0 is the kernel's
/// own (no user mappings); the kernel hands out 1..=255 (arch::paging).
pub static CURRENT_ASID: AtomicUsize = AtomicUsize::new(0);

/// Make `table` the table refills walk and `asid` the current identifier.
///
/// # Safety
/// Changes address translation for the whole CPU.
pub unsafe fn switch_table(table: usize, asid: usize) {
    CURRENT_TABLE.store(table, Ordering::Release);
    CURRENT_ASID.store(asid, Ordering::Relaxed);
    unsafe {
        asm!("mtc0 {asid}, $10", "nop", "nop", "nop", asid = in(reg) asid, options(nostack));
    }
}

/// Drop every entry written under `asid` (global entries stay).
///
/// # Safety
/// Changes address translation for the whole CPU.
pub unsafe fn flush_asid(asid: usize) {
    unsafe {
        for index in 0..TLB_ENTRIES {
            let (hi, lo0, lo1): (usize, usize, usize);
            asm!(
                "mtc0 {index}, $0",
                "nop",
                "nop",
                "tlbr",
                "nop",
                "nop",
                "mfc0 {hi}, $10",
                "mfc0 {lo0}, $2",
                "mfc0 {lo1}, $3",
                index = in(reg) index,
                hi = out(reg) hi,
                lo0 = out(reg) lo0,
                lo1 = out(reg) lo1,
                options(nostack),
            );
            if hi & 0xff == asid && lo0 & lo1 & ENTRYLO_GLOBAL == 0 {
                write_indexed(index, unmapped_entry_hi(index), 0, 0);
            }
        }
        // `tlbr` and the writes left another identifier in EntryHi.
        asm!(
            "mtc0 {asid}, $10",
            "nop",
            "nop",
            "nop",
            asid = in(reg) CURRENT_ASID.load(Ordering::Relaxed),
            options(nostack)
        );
    }
}

/// Clear every TLB entry and set PageMask to 4 KiB pages, Wired to 0.
///
/// # Safety
/// Changes address translation for the whole CPU.
pub unsafe fn flush_all() {
    TLB_FLUSHES.fetch_add(1, Ordering::Relaxed);
    unsafe {
        asm!(
            "mtc0 $zero, $5", // PageMask: 4 KiB
            "mtc0 $zero, $6", // Wired
            "nop",
            options(nostack)
        );
        for index in 0..TLB_ENTRIES {
            write_indexed(index, unmapped_entry_hi(index), 0, 0);
        }
        asm!(
            "mtc0 {asid}, $10",
            "nop",
            "nop",
            "nop",
            asid = in(reg) CURRENT_ASID.load(Ordering::Relaxed),
            options(nostack)
        );
    }
}

impl Arch for MipsArch {
    const KERNEL_SEPARATE_TABLE: bool = false;

    const PAGE_SHIFT: usize = 12; // 4096 bytes
    const PAGE_ENTRY_SHIFT: usize = 10; // 1024 entries, 4 bytes each
    const PAGE_LEVELS: usize = 2; // directory, table

    const ENTRY_ADDRESS_WIDTH: usize = 20; // 32-bit physical addresses
    const ENTRY_ADDRESS_SHIFT: usize = 6; // EntryLo frame number position
    const ENTRY_FLAG_DEFAULT_PAGE: usize = Self::ENTRY_FLAG_PRESENT;
    const ENTRY_FLAG_DEFAULT_TABLE: usize = Self::ENTRY_FLAG_PRESENT | Self::ENTRY_FLAG_READWRITE;
    const ENTRY_FLAG_PRESENT: usize = 1 << 1; // EntryLo V
    const ENTRY_FLAG_READONLY: usize = 0;
    const ENTRY_FLAG_READWRITE: usize = 1 << 2; // EntryLo D
    const ENTRY_FLAG_PAGE_USER: usize = 1 << 31; // software
    const ENTRY_FLAG_NO_EXEC: usize = 1 << 30; // software; the R4400 has no execute permission
    const ENTRY_FLAG_EXEC: usize = 0;
    const ENTRY_FLAG_GLOBAL: usize = 0; // kernel entries get G when written (write_random)
    const ENTRY_FLAG_NO_GLOBAL: usize = 0;
    const ENTRY_FLAG_DEVICE_MEMORY: usize = PTE_UNCACHED;
    const ENTRY_FLAG_UNCACHEABLE: usize = PTE_UNCACHED;
    const ENTRY_FLAG_WRITE_COMBINING: usize = PTE_UNCACHED;

    /// kseg0: the cached direct map of the low 512 MiB of physical memory.
    const PHYS_OFFSET: usize = 0x8000_0000;

    #[inline(always)]
    fn invalidate(address: VirtualAddress) {
        unsafe {
            let index: isize;
            asm!(
                "mtc0 {hi}, $10",
                "nop",
                "nop",
                "tlbp",
                "nop",
                "nop",
                "mfc0 {index}, $0",
                hi = in(reg) (address.data() & !0x1fff) | CURRENT_ASID.load(Ordering::Relaxed),
                index = out(reg) index,
                options(nostack),
            );
            if index >= 0 {
                let index = index as usize;
                write_indexed(index, unmapped_entry_hi(index), 0, 0);
                asm!(
                    "mtc0 {asid}, $10",
                    "nop",
                    "nop",
                    "nop",
                    asid = in(reg) CURRENT_ASID.load(Ordering::Relaxed),
                    options(nostack)
                );
            }
        }
    }

    #[inline(always)]
    fn invalidate_all() {
        unsafe { flush_all() }
    }

    #[inline(always)]
    fn table(_table_kind: TableKind) -> PhysicalAddress {
        PhysicalAddress::new(CURRENT_TABLE.load(Ordering::Acquire))
    }

    #[inline(always)]
    unsafe fn set_table(_table_kind: TableKind, address: PhysicalAddress) {
        // The general switch: no identifier is known for this table, so the
        // TLB is emptied and identifier 0 is used. The kernel's context
        // switch goes through `switch_table` with an identifier instead.
        unsafe {
            switch_table(address.data(), 0);
            flush_all()
        }
    }

    fn virt_is_valid(_address: VirtualAddress) -> bool {
        // Every 32-bit address is a valid MIPS virtual address.
        true
    }
}

const _: () = {
    assert!(MipsArch::PAGE_SIZE == 4096);
    assert!(MipsArch::PAGE_OFFSET_MASK == 0xFFF);
    assert!(MipsArch::PAGE_ADDRESS_SHIFT == 32);
    assert!(MipsArch::PAGE_ADDRESS_SIZE == 0x0000_0001_0000_0000);
    assert!(MipsArch::PAGE_ADDRESS_MASK == 0xFFFF_F000);
    assert!(MipsArch::PAGE_ENTRY_SIZE == 4);
    assert!(MipsArch::PAGE_ENTRIES == 1024);
    assert!(MipsArch::PAGE_ENTRY_MASK == 0x3FF);
    assert!(MipsArch::ENTRY_ADDRESS_MASK == 0x000F_FFFF);
    assert!(MipsArch::ENTRY_FLAGS_MASK == 0xFC00_003F);
    assert!(MipsArch::PHYS_OFFSET == 0x8000_0000);
};
