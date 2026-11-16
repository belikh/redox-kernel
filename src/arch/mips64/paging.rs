//! TLB setup: 4 KiB pages, nothing wired, ASID 0, every entry cleared, and
//! the exception vectors in place so refills can be served.
//!
//! Address space identifiers: the R4600's EntryHi has an 8-bit ASID, and a
//! user TLB entry matches only under the one it was written with. Each
//! address space gets one of 1..=255 from its key (the address of its
//! `AddrSpaceWrapper::used_by`, which the switch code and the `Flusher`
//! both have). A context switch then changes EntryHi and keeps the TLB;
//! kernel entries are global. Two spaces whose keys fall on one identifier
//! share it in turn: the newcomer drops the other's entries first.

use super::cp0;

#[cold]
pub unsafe fn init() {
    unsafe {
        // Each step reports while the PROM's vectors are still in charge, so
        // a hang on real hardware names the step it happened in.
        info!(
            "Indy: PRId {:#x}, Config {:#x}, Status {:#x}",
            cp0::read_prid(),
            cp0::read_config(),
            cp0::read_status()
        );
        super::cache::init();
        cp0::write_entry_hi(0); // ASID 0
        rmm::mips::flush_all();
        info!("Indy: TLB cleared");
        super::interrupt::exception::install_vectors();
        info!("Indy: vectors written");
        // Vectors live in kseg0 now: leave the PROM's bootstrap vectors.
        cp0::write_status(cp0::read_status() & !cp0::status::BEV);
    }
}

use core::sync::atomic::{AtomicUsize, Ordering};

/// The key of the address space that last ran under each identifier.
static OWNER: [AtomicUsize; 256] = [const { AtomicUsize::new(0) }; 256];

fn identifier(key: usize) -> usize {
    ((key >> 4) ^ (key >> 12)) % 255 + 1
}

/// Make the address space `key`, with page table `table`, the current one.
pub unsafe fn switch_to(key: usize, table: usize) {
    // Off-colour pages on the cache list belong to the space being left:
    // their entries can only be found under its identifier.
    super::cache::before_frames();
    let asid = identifier(key);
    unsafe {
        if OWNER[asid].swap(key, Ordering::Relaxed) != key {
            rmm::mips::flush_asid(asid);
        }
        rmm::mips::switch_table(table, asid);
    }
}

/// Mappings of the address space `key` changed: drop its TLB entries,
/// whether or not it is the current one.
pub fn flush_space(key: usize) {
    let asid = identifier(key);
    if OWNER[asid].load(Ordering::Relaxed) == key {
        unsafe { rmm::mips::flush_asid(asid) };
    }
}

/// A new address space takes `key`: drop what an earlier one left under it.
pub fn retire(key: usize) {
    let asid = identifier(key);
    if OWNER[asid].load(Ordering::Relaxed) == key {
        unsafe { rmm::mips::flush_asid(asid) };
        OWNER[asid].store(0, Ordering::Relaxed);
    }
}
