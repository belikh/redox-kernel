//! Primary cache coherence.
//!
//! The R4x00 and R5000 primary caches are indexed by virtual address, and a
//! way is larger than a 4 KiB page: 8 KiB on the R4600 (two colours), 16 KiB
//! on the R4400 and R5000 (four). A frame reached through two addresses of
//! different colour has two independent cached copies. The instruction cache
//! fills from RAM and never sees a store. Only an R4000/R4400 with a
//! secondary cache traps on the first problem (VCED/VCEI).
//!
//! The kernel reaches frames through kseg0, so its colour is the frame's
//! own. The design keeps user mappings on that colour and pays only for the
//! ones that are not:
//!
//! - A frame allocated in a page fault is picked to have the colour of the
//!   faulting address (`recolour`), and a mapping borrowed from another
//!   address space is placed on the colour of its source
//!   (`Grants::find_free_near`). Such mappings share their cache lines with
//!   kseg0 and need nothing.
//! - A mapping on another colour is handled where its TLB entry is written
//!   (`installing`): the frame's kseg0 lines are written back first, and a
//!   user page is put on a short list. On the next trap that can touch
//!   frames (`before_frames`), the listed pages' lines are written back and
//!   their TLB entries dropped, so the next use comes through `installing`
//!   again. A refill served from the page table touches no frame and does
//!   neither.
//! - Instruction fetch: one bit per frame says the instruction cache holds
//!   nothing stale for it. A refill that maps the frame executable with the
//!   bit clear writes the frame's data lines back and invalidates its
//!   instruction lines under every colour, then sets the bit. A refill that
//!   maps it writable, and a new allocation, clear the bit.
//!
//! Limits: a frame mapped under two colours in one address space and used
//! through both without a trap in between is not kept coherent, and code
//! written through a mapping whose TLB entry predates an executable mapping
//! of the same frame is not noticed.

use core::{
    cell::SyncUnsafeCell,
    sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering},
};

use rmm::Arch;

use crate::memory::{Frame, RmmA};

use super::cp0;

static DCACHE_BYTES: AtomicUsize = AtomicUsize::new(0);
static DCACHE_LINE: AtomicUsize = AtomicUsize::new(16);
static ICACHE_BYTES: AtomicUsize = AtomicUsize::new(0);
static ICACHE_LINE: AtomicUsize = AtomicUsize::new(16);

/// Index operations select the line (and the way, with the top index bit)
/// from the address, so any kseg0 range of the cache's size reaches it all.
const BASE: usize = 0x8000_0000;

/// Read the primary cache geometry from Config: IC bits 11..9 and DC bits
/// 8..6 give 4 KiB << n, IB bit 5 and DB bit 4 give 16- or 32-byte lines.
pub fn init() {
    let config = cp0::read_config();
    let icache = 4096usize << ((config >> 9) & 7);
    let dcache = 4096usize << ((config >> 6) & 7);
    let iline = if config & (1 << 5) != 0 { 32 } else { 16 };
    let dline = if config & (1 << 4) != 0 { 32 } else { 16 };
    ICACHE_BYTES.store(icache, Ordering::Relaxed);
    ICACHE_LINE.store(iline, Ordering::Relaxed);
    DCACHE_BYTES.store(dcache, Ordering::Relaxed);
    DCACHE_LINE.store(dline, Ordering::Relaxed);
    // Two ways on the R4600, R4700, R4650 and R5000 (PRId implementation
    // 0x20..0x23); the R4000 and R4400 are direct mapped.
    let ways = match (cp0::read_prid() >> 8) & 0xff {
        0x20..=0x23 | 0x28 => 2,
        _ => 1,
    };
    WAYS.store(ways, Ordering::Relaxed);
    DCACHE_WAY.store(dcache / ways, Ordering::Relaxed);
    COLOUR_MASK.store((dcache / ways / 4096).max(1) - 1, Ordering::Relaxed);
    ICACHE_WAY.store(icache / ways, Ordering::Relaxed);
    info!(
        "Indy: primary caches I {} KiB/{} B lines, D {} KiB/{} B lines, {} way(s), {} colours",
        icache >> 10,
        iline,
        dcache >> 10,
        dline,
        ways,
        colours()
    );
}

static WAYS: AtomicUsize = AtomicUsize::new(1);
static DCACHE_WAY: AtomicUsize = AtomicUsize::new(4096);
static ICACHE_WAY: AtomicUsize = AtomicUsize::new(4096);

const PAGE: usize = 4096;

/// Page colours of the data cache: pages per way.
pub fn colours() -> usize {
    (DCACHE_WAY.load(Ordering::Relaxed) / PAGE).max(1)
}

/// The data cache colour of an address, virtual or physical.
pub fn colour(addr: usize) -> usize {
    (addr / PAGE) & (colours() - 1)
}

/// Write back and invalidate the data cache lines a page at `addr` can
/// occupy, in every way. Index operations: no TLB entry is needed and they
/// do not depend on what the lines hold.
fn flush_data_page(addr: usize) {
    let line = DCACHE_LINE.load(Ordering::Relaxed);
    let way = DCACHE_WAY.load(Ordering::Relaxed);
    let offset = addr & (way - 1) & !(PAGE - 1);
    for w in 0..WAYS.load(Ordering::Relaxed) {
        let start = BASE + w * way + offset;
        let mut at = start;
        while at < start + PAGE {
            unsafe {
                core::arch::asm!("cache 1, 0({0})", in(reg) at, options(nostack));
            }
            at += line;
        }
    }
}

/// One byte per frame of the first 64 MiB of RAM. Bit 0: the instruction
/// cache holds nothing stale for the frame; bit 1 below. Frames above that have no
/// state and are flushed every time.
const TRACKED_FRAMES: usize = 64 * 1024 * 1024 / PAGE;
static FRAME_STATE: [AtomicU8; TRACKED_FRAMES] = [const { AtomicU8::new(0) }; TRACKED_FRAMES];
const ICACHE_CLEAN: u8 = 1;
/// Bit 1: a kernel page's kseg0 lines were written back after allocation.
const KSEG0_SETTLED: u8 = 2;

fn frame_state(phys: usize) -> Option<&'static AtomicU8> {
    let index = phys.checked_sub(super::consts::RAM_PHYS_BASE)? / PAGE;
    FRAME_STATE.get(index)
}

/// Frames went back to the allocator: forget what is known about them.
pub fn forget(first: Frame, order: u32) {
    for index in 0..1usize << order {
        if let Some(state) = frame_state(first.base().data() + index * PAGE) {
            state.store(0, Ordering::Relaxed);
        }
    }
}

/// User pages mapped on another colour than their frame since the last
/// `before_frames`. Single CPU, and only touched with interrupts off.
const LIST_LEN: usize = 24;
static LIST: SyncUnsafeCell<[usize; LIST_LEN]> = SyncUnsafeCell::new([0; LIST_LEN]);
static LIST_USED: AtomicUsize = AtomicUsize::new(0);
static LIST_OVERFLOW: AtomicBool = AtomicBool::new(false);

/// Counters for `dmesg`-side inspection through the info line at boot and
/// for tuning: refills seen, off-colour installs, executable flushes.
pub static STAT_OFF_COLOUR: AtomicUsize = AtomicUsize::new(0);
pub static STAT_EXEC_FLUSH: AtomicUsize = AtomicUsize::new(0);
pub static STAT_ENTRY_PAGES: AtomicUsize = AtomicUsize::new(0);
pub static STAT_OVERFLOW: AtomicUsize = AtomicUsize::new(0);

/// A TLB entry for `page` -> `pte` is about to be written. Returns the fast
/// bit to set on the entry when installing it again will need no work here
/// (rmm::mips::PTE_FAST_*), or 0.
pub fn installing(page: usize, pte: usize) -> usize {
    STAT_REFILLS.fetch_add(1, Ordering::Relaxed);
    let phys = ((pte >> RmmA::ENTRY_ADDRESS_SHIFT) & RmmA::ENTRY_ADDRESS_MASK) << RmmA::PAGE_SHIFT;
    let writable = pte & RmmA::ENTRY_FLAG_READWRITE != 0;
    let executable = pte & RmmA::ENTRY_FLAG_NO_EXEC == 0;
    let user = page < crate::USER_END_OFFSET;
    let mut data_flushed = false;

    let state = frame_state(phys);
    if pte & rmm::mips::PTE_UNCACHED != 0 {
        // Device registers and DMA memory: the mapping uses no cache, so it
        // has no colour and needs no place on the list. RAM (DMA memory)
        // may still have lines in kseg0 from when the kernel cleared it;
        // they are written back once, or a later eviction would overwrite
        // what the device stored.
        if let Some(state) = state
            && state.load(Ordering::Relaxed) & KSEG0_SETTLED == 0
        {
            flush_data_page(phys);
            state.fetch_or(KSEG0_SETTLED, Ordering::Relaxed);
        }
        return 0;
    }
    if colour(page) != colour(phys) {
        // A kernel (kseg2) page is only ever used through this mapping once
        // it is mapped, so its kseg0 lines need writing back once.
        let settled =
            !user && state.is_some_and(|s| s.load(Ordering::Relaxed) & KSEG0_SETTLED != 0);
        if !settled {
            STAT_OFF_COLOUR.fetch_add(1, Ordering::Relaxed);
            note_off_colour(page, writable, executable);
            flush_data_page(phys);
            data_flushed = true;
            if let (false, Some(state)) = (user, state) {
                state.fetch_or(KSEG0_SETTLED, Ordering::Relaxed);
            }
        }
        if user {
            let used = LIST_USED.load(Ordering::Relaxed);
            let list = unsafe { &mut *LIST.get() };
            if !list[..used].contains(&page) {
                if used < LIST_LEN {
                    list[used] = page;
                    LIST_USED.store(used + 1, Ordering::Relaxed);
                } else {
                    LIST_OVERFLOW.store(true, Ordering::Relaxed);
                }
            }
        }
    }

    if user && executable {
        let clean = state.is_some_and(|s| s.load(Ordering::Relaxed) & ICACHE_CLEAN != 0);
        if !clean || writable {
            STAT_EXEC_FLUSH.fetch_add(1, Ordering::Relaxed);
            if !data_flushed {
                flush_data_page(phys);
            }
            flush_instruction_page(phys);
        }
    }
    if let Some(state) = state {
        if writable {
            state.fetch_and(!ICACHE_CLEAN, Ordering::Relaxed);
        } else if user && executable {
            state.fetch_or(ICACHE_CLEAN, Ordering::Relaxed);
        }
    }

    let on_colour = colour(page) == colour(phys);
    if !user {
        // Kernel pages: on colour, or off colour with kseg0 settled.
        let settled = state.is_some_and(|s| s.load(Ordering::Relaxed) & KSEG0_SETTLED != 0);
        return if on_colour || settled { rmm::mips::PTE_FAST_DATA } else { 0 };
    }
    if !on_colour || pte & RmmA::ENTRY_FLAG_PAGE_USER == 0 {
        return 0;
    }
    if !executable {
        rmm::mips::PTE_FAST_DATA
    } else if !writable && state.is_some() {
        rmm::mips::PTE_FAST_TEXT
    } else {
        0
    }
}

/// Colours minus one, for the assembly refill handler.
pub static COLOUR_MASK: AtomicUsize = AtomicUsize::new(0);

/// Invalidate the instruction cache lines of one page offset under every
/// colour and way.
fn flush_instruction_page(phys: usize) {
    let line = ICACHE_LINE.load(Ordering::Relaxed);
    let way = ICACHE_WAY.load(Ordering::Relaxed);
    let _ = phys;
    for w in 0..WAYS.load(Ordering::Relaxed) {
        let mut colour_base = BASE + w * way;
        let end = colour_base + way;
        while colour_base < end {
            let mut at = colour_base;
            while at < colour_base + PAGE {
                unsafe {
                    core::arch::asm!("cache 0, 0({0})", in(reg) at, options(nostack));
                }
                at += line;
            }
            colour_base += PAGE;
        }
    }
}

/// The kernel is about to touch frames on behalf of a trap: bring the
/// off-colour user pages back to RAM and make their next use a refill.
pub fn before_frames() {
    STAT_TRAPS.fetch_add(1, Ordering::Relaxed);
    if LIST_OVERFLOW.swap(false, Ordering::Relaxed) {
        STAT_OVERFLOW.fetch_add(1, Ordering::Relaxed);
        LIST_USED.store(0, Ordering::Relaxed);
        flush_data();
        RmmA::invalidate_all();
        return;
    }
    let used = LIST_USED.swap(0, Ordering::Relaxed);
    if used == 0 {
        return;
    }
    STAT_ENTRY_PAGES.fetch_add(used, Ordering::Relaxed);
    let list = unsafe { &*LIST.get() };
    for &page in &list[..used] {
        flush_data_page(page);
        RmmA::invalidate(rmm::VirtualAddress::new(page));
    }
}

/// Colour wanted for frames allocated right now: the faulting address's,
/// while the page fault handler runs. `usize::MAX` is no preference.
static ALLOC_COLOUR: AtomicUsize = AtomicUsize::new(usize::MAX);

pub fn set_alloc_colour(addr: Option<usize>) {
    ALLOC_COLOUR.store(addr.map_or(usize::MAX, colour), Ordering::Relaxed);
}

/// Called with every newly allocated single frame. Resets the frame's
/// state, and swaps the frame for one of the wanted colour when the first
/// is not. Returns the frame to use.
pub fn recolour(first: Frame, must_be_zero: bool) -> Frame {
    let reset = |frame: Frame| {
        if let Some(state) = frame_state(frame.base().data()) {
            state.store(0, Ordering::Relaxed);
        }
    };
    let want = ALLOC_COLOUR.load(Ordering::Relaxed);
    if want == usize::MAX || colour(first.base().data()) == want {
        reset(first);
        return first;
    }
    // The buddy allocator hands out neighbours, so a few tries reach every
    // colour. The rejects go back afterwards.
    STAT_RECOLOUR.fetch_add(1, Ordering::Relaxed);
    let mut rejects: [Option<Frame>; 6] = [None; 6];
    let mut chosen = None;
    for slot in rejects.iter_mut() {
        let Some(frame) = crate::memory::allocate_p2frame(0, must_be_zero) else {
            break;
        };
        if colour(frame.base().data()) == want {
            chosen = Some(frame);
            break;
        }
        *slot = Some(frame);
    }
    let (keep, spare) = match chosen {
        Some(frame) => (frame, Some(first)),
        // The free lists held no frame of this colour near their heads:
        // cut one from a block that holds every colour.
        None => match crate::memory::allocate_frame_of_colour(want, colours(), must_be_zero) {
            Some(frame) => (frame, Some(first)),
            None => {
                STAT_RECOLOUR_FAILED.fetch_add(1, Ordering::Relaxed);
                (first, None)
            }
        },
    };
    for frame in rejects.into_iter().flatten().chain(spare) {
        unsafe { crate::memory::deallocate_frame(frame) };
    }
    reset(keep);
    keep
}

/// Write back and invalidate every data cache line.
pub fn flush_data() {
    let line = DCACHE_LINE.load(Ordering::Relaxed);
    let end = BASE + DCACHE_BYTES.load(Ordering::Relaxed);
    let mut addr = BASE;
    while addr < end {
        unsafe {
            core::arch::asm!(
                "cache 1, 0({0})", // Index_Writeback_Inv_D
                in(reg) addr,
                options(nostack)
            );
        }
        addr += line;
    }
}

/// Invalidate every instruction cache line.
pub fn flush_instructions() {
    let line = ICACHE_LINE.load(Ordering::Relaxed);
    let end = BASE + ICACHE_BYTES.load(Ordering::Relaxed);
    let mut addr = BASE;
    while addr < end {
        unsafe {
            core::arch::asm!(
                "cache 0, 0({0})", // Index_Invalidate_I
                in(reg) addr,
                options(nostack)
            );
        }
        addr += line;
    }
}

/// Both caches, before user code runs.
pub extern "C" fn flush_for_user() {
    flush_data();
    flush_instructions();
}

/// The Indy's R4600SC and R5000SC modules carry a 512 KiB secondary cache
/// outside the CPU, which the R4600 does not know about (its Config has no
/// SC bit). The module watches the bus: it answers block reads it holds and
/// is controlled by stores to physical 0x8000_0000 through the uncached
/// 64-bit segment, with KX set: a byte store switches it on, a halfword
/// store switches it off, and a word store at an offset invalidates the
/// line of that offset (Linux arch/mips/mm/sc-ip22.c, NetBSD
/// sgimips/ip22_cache.c). The PROM hands the machine over with it off.
///
/// `enable` != 0: walk the code once without storing, so every line of it
/// is in the primary cache before the secondary cache can answer a fetch,
/// then switch on and invalidate all 16384 lines. `enable` == 0: switch
/// off. Runs with interrupts off.
#[unsafe(naked)]
unsafe extern "C" fn scache_control(enable: usize) {
    core::arch::naked_asm!(
        "
        .set noreorder
        .set noat
        .balign 32
        mfc0    $8, $12
        li      $9, 0x80                // KX only: 64-bit addresses, interrupts off
        mtc0    $9, $12
        nop
        nop
        nop
        nop
        lui     $9, 0x9000
        dsll    $9, $9, 16
        ori     $9, $9, 0x8000
        dsll    $9, $9, 16              // 0x9000_0000_8000_0000
        beqz    $4, 5f
        li      $12, 1                  // first pass stores nothing
    1:  move    $10, $9
        lui     $11, 0x8                // 512 KiB
        daddu   $11, $11, $9
    2:  bnez    $12, 3f
        nop
        sw      $0, 0($10)              // invalidate one line
    3:  daddiu  $10, $10, 32
        bne     $10, $11, 2b
        nop
        beqz    $12, 6f                 // second pass done
        nop
        bnez    $4, 4f
        move    $12, $0
        // The store and the branch after it share one cache line, fetched
        // while the secondary cache is still off.
        .balign 32
    4:  sb      $0, 0($9)               // on
        b       1b
        nop
    5:  sh      $0, 0($9)               // off
    6:  mtc0    $8, $12
        nop
        nop
        nop
        nop
        jr      $31
        nop
        .set at
        .set reorder
        "
    )
}

static SCACHE_ON: AtomicBool = AtomicBool::new(false);

/// Switch the module's secondary cache on or off. Only when the PROM's
/// inventory lists one of 512 KiB with 32-byte lines.
pub fn scache(enable: bool) -> bool {
    if crate::arch::device::arcs::secondary_cache_key() != 0x0105_0007 {
        return false;
    }
    unsafe { scache_control(usize::from(enable)) };
    SCACHE_ON.store(enable, Ordering::Relaxed);
    info!("Indy: secondary cache (512 KiB) {}", if enable { "on" } else { "off" });
    true
}

/// Around a firmware call: the PROM expects the cache as it left it, off.
/// Returns whether it was on, for `scache_resume`.
pub fn scache_pause() -> bool {
    let was_on = SCACHE_ON.swap(false, Ordering::Relaxed);
    if was_on {
        unsafe { scache_control(0) };
    }
    was_on
}

pub fn scache_resume(was_on: bool) {
    if was_on {
        unsafe { scache_control(1) };
        SCACHE_ON.store(true, Ordering::Relaxed);
    }
}

/// Counters since boot, for `echo cachestat > /scheme/sys/kstop`.
pub fn log_stats() {
    log_notes();
    let mut line = alloc::string::String::new();
    for code in 0..32 {
        let count = STAT_BY_CODE[code].swap(0, Ordering::Relaxed);
        let ticks = STAT_TICKS[code].swap(0, Ordering::Relaxed);
        if count != 0 || ticks != 0 {
            use core::fmt::Write;
            let _ = write!(line, " code{}:{}x{}k", code, count, ticks / 1000);
        }
    }
    // Syscalls: the 28 buckets with the most time, as
    // program:number/first argument, calls x thousands of Count ticks.
    let mut top = [(0usize, 0usize, 0usize); 28];
    for bucket in 0..256 {
        let count = STAT_BY_CODE[32 + bucket].swap(0, Ordering::Relaxed);
        let ticks = STAT_TICKS[32 + bucket].swap(0, Ordering::Relaxed);
        if ticks > top[27].0 {
            top[27] = (ticks, count, bucket);
            top.sort_unstable_by(|a, b| b.0.cmp(&a.0));
        }
    }
    let mut calls = alloc::string::String::new();
    for (ticks, count, bucket) in top {
        if ticks == 0 {
            continue;
        }
        use core::fmt::Write;
        let who = STAT_WHO[bucket].load(Ordering::Relaxed);
        let tag = ((who >> 8) as u32).to_be_bytes();
        let _ = write!(
            calls,
            " {}:{:#x}/{}:{}x{}k",
            core::str::from_utf8(&tag).unwrap_or("?").trim(),
            STAT_SYSCALL[bucket].load(Ordering::Relaxed),
            who & 0xff,
            count,
            ticks / 1000
        );
    }
    info!("Indy: syscalls:{}", calls);
    info!(
        "Indy: secondary cache {}, PRId {:#x}, Config {:#x}, PROM inventory (class/type/key):{}",
        if SCACHE_ON.load(Ordering::Relaxed) { "on" } else { "off" },
        cp0::read_prid(),
        cp0::read_config(),
        crate::arch::device::arcs::inventory()
    );
    info!(
        "Indy: console: {} keyboard interrupts, {} serial interrupts, {} keyboard bytes found by the fallback poll",
        crate::arch::device::KEYBOARD_INTERRUPTS.load(Ordering::Relaxed),
        crate::arch::device::SERIAL_INTERRUPTS.load(Ordering::Relaxed),
        crate::arch::device::FALLBACK_INPUT.load(Ordering::Relaxed)
    );
    info!(
        "Indy: events: {} switches, {} fast refills, {} TLB flushes;{}",
        STAT_SWITCHES.swap(0, Ordering::Relaxed),
        STAT_FAST_REFILLS.swap(0, Ordering::Relaxed),
        rmm::mips::TLB_FLUSHES.swap(0, Ordering::Relaxed),
        line
    );
    info!(
        "Indy: cache: {} off-colour installs, {} executable flushes, {} pages written back at traps, {} list overflows, {} traps with frames, {} refills, {} recolours tried, {} failed",
        STAT_OFF_COLOUR.load(Ordering::Relaxed),
        STAT_EXEC_FLUSH.load(Ordering::Relaxed),
        STAT_ENTRY_PAGES.load(Ordering::Relaxed),
        STAT_OVERFLOW.load(Ordering::Relaxed),
        STAT_TRAPS.load(Ordering::Relaxed),
        STAT_REFILLS.load(Ordering::Relaxed),
        STAT_RECOLOUR.load(Ordering::Relaxed),
        STAT_RECOLOUR_FAILED.load(Ordering::Relaxed),
    );
}
pub static STAT_TRAPS: AtomicUsize = AtomicUsize::new(0);
pub static STAT_REFILLS: AtomicUsize = AtomicUsize::new(0);
pub static STAT_RECOLOUR: AtomicUsize = AtomicUsize::new(0);
pub static STAT_RECOLOUR_FAILED: AtomicUsize = AtomicUsize::new(0);

/// The off-colour pages seen most, for tuning: (page | flags, the last four
/// characters of the running context's name, count).
const NOTE_LEN: usize = 48;
static NOTES: SyncUnsafeCell<[(usize, u32, u32); NOTE_LEN]> =
    SyncUnsafeCell::new([(0, 0, 0); NOTE_LEN]);

/// The end of the running context's name, read without its lock: one CPU,
/// interrupts off, and the result is only printed.
pub fn current_tag() -> u32 {
    let mut tag = [b' '; 4];
    let current = crate::context::try_current();
    if let Some(context) = current.as_ref() {
        let name = unsafe { &(*context.as_mut_ptr()).name };
        let bytes = name.as_bytes();
        let tail = &bytes[bytes.len().saturating_sub(4)..];
        tag[..tail.len()].copy_from_slice(tail);
    }
    u32::from_be_bytes(tag)
}

fn note_off_colour(page: usize, writable: bool, executable: bool) {
    let key = page | usize::from(writable) | (usize::from(executable) << 1);
    let tag = if page < crate::USER_END_OFFSET { current_tag() } else { 0 };
    let notes = unsafe { &mut *NOTES.get() };
    let mut weakest = 0;
    for (i, note) in notes.iter_mut().enumerate() {
        if note.0 == key && note.1 == tag {
            note.2 = note.2.saturating_add(1);
            return;
        }
        if note.2 < unsafe { (*NOTES.get())[weakest].2 } {
            weakest = i;
        }
    }
    notes[weakest] = (key, tag, 1);
}

pub fn log_notes() {
    let notes = unsafe { &*NOTES.get() };
    let mut sorted = *notes;
    sorted.sort_unstable_by(|a, b| b.2.cmp(&a.2));
    for chunk in sorted.chunks(6) {
        let mut line = alloc::string::String::new();
        for (key, tag, count) in chunk {
            use core::fmt::Write;
            let tag = tag.to_be_bytes();
            let _ = write!(
                line,
                " {}:{:#x}{}{}x{}",
                core::str::from_utf8(&tag).unwrap_or("?").trim(),
                key & !0xfff,
                if key & 1 != 0 { "w" } else { "" },
                if key & 2 != 0 { "x" } else { "" },
                count
            );
        }
        info!("Indy: off-colour:{}", line);
    }
}

/// Charge kernel time to trap codes and syscall numbers (the `events` line
/// of the statistics). Costs two Count reads and a few stores per trap, so
/// it is off unless a kernel is being tuned.
pub const TRAP_TIMING: bool = false;

/// Event counts for tuning, logged with the cache counters.
pub static STAT_SWITCHES: AtomicUsize = AtomicUsize::new(0);
pub static STAT_FAST_REFILLS: AtomicUsize = AtomicUsize::new(0);
pub static STAT_BY_CODE: [AtomicUsize; 288] = [const { AtomicUsize::new(0) }; 288];
/// The syscall number behind each of buckets 32..96.
pub static STAT_SYSCALL: [AtomicUsize; 256] = [const { AtomicUsize::new(0) }; 256];
/// The calling program's tag (high bits) and first argument (low byte).
pub static STAT_WHO: [AtomicUsize; 256] = [const { AtomicUsize::new(0) }; 256];
/// CP0 Count ticks spent between a trap's entry and its return, by code.
/// A trap that blocks counts the time until its context runs again.
pub static STAT_TICKS: [AtomicUsize; 288] = [const { AtomicUsize::new(0) }; 288];
pub static STAT_TLB_FLUSHES: AtomicUsize = AtomicUsize::new(0);

