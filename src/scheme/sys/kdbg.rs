//! Read-only inspection of the running kernel for a remote debugger.
//!
//! `tools/kdbg` on the Indy speaks the GDB remote protocol on a TCP port and
//! answers from two files here. The kernel keeps running: this shows state,
//! it does not stop anything.
//!
//! - `kdbg_ctx`: one line per context, all numbers hexadecimal:
//!   `id pid running table kernel-registers(13) user(0|1) [user-registers(37)] name`.
//!   Kernel registers are what `switch_to` saved: sp, ra, fp, gp, s0..s7,
//!   status. They are stale for a context that is running. User registers
//!   are `$1`..`$31`, then pc, status, cause, badvaddr, hi, lo from the trap
//!   frame.
//! - `kdbg_mem`: write `table address length`, then read up to that many
//!   bytes. `table` is the physical address of a context's page table for a
//!   user address, or 0. The read stops at the first page that is not
//!   mapped RAM.

use alloc::{string::String, vec::Vec};
use core::{
    fmt::Write,
    sync::atomic::{AtomicUsize, Ordering},
};

use rmm::Arch;

use crate::{
    context::contexts,
    memory::RmmA,
    sync::CleanLockToken,
    syscall::error::{Error, Result, EINVAL},
};

static REQUEST: [AtomicUsize; 3] = [const { AtomicUsize::new(0) }; 3];
const MAX_READ: usize = 4096;

pub fn contexts_resource(token: &mut CleanLockToken) -> Result<Vec<u8>> {
    let mut out = String::new();
    let mut contexts = contexts(token.downgrade());
    let (contexts, mut token) = contexts.token_split();
    for context_ref in contexts.iter() {
        let context = context_ref.read(token.token());
        let addr_space = context.addr_space().cloned();
        let _ = write!(
            out,
            "{:x} {:x} {:x} ",
            context.debug_id,
            context.pid,
            u8::from(context.running)
        );
        let kernel = context.arch.debug_regs();
        let user = context.regs().map(|stack| {
            let r = &stack.registers;
            [
                r.at, r.v0, r.v1, r.a0, r.a1, r.a2, r.a3, r.a4, r.a5, r.a6, r.a7, r.t0, r.t1,
                r.t2, r.t3, r.s0, r.s1, r.s2, r.s3, r.s4, r.s5, r.s6, r.s7, r.t8, r.t9, r.k0,
                r.k1, r.gp, r.sp, r.fp, r.ra, stack.iret.epc, stack.iret.status,
                stack.iret.cause, stack.iret.badvaddr, stack.iret.hi, stack.iret.lo,
            ]
        });
        let name = context.name;
        drop(context);

        let table = match addr_space {
            Ok(addr_space) => {
                let guard = addr_space.acquire_read(token.downgrade());
                guard.table.utable.table().phys().data()
            }
            Err(_) => 0,
        };
        let _ = write!(out, "{:x}", table);
        for value in kernel {
            let _ = write!(out, " {:x}", value);
        }
        match user {
            Some(regs) => {
                out.push_str(" 1");
                for value in regs {
                    let _ = write!(out, " {:x}", value);
                }
            }
            None => out.push_str(" 0"),
        }
        let _ = writeln!(out, " {}", name);
    }
    Ok(out.into_bytes())
}

pub fn mem_request(arg: &[u8], _token: &mut CleanLockToken) -> Result<usize> {
    let text = core::str::from_utf8(arg).map_err(|_| Error::new(EINVAL))?;
    let mut fields = text.split_ascii_whitespace();
    for slot in &REQUEST {
        let field = fields.next().ok_or(Error::new(EINVAL))?;
        let value = usize::from_str_radix(field, 16).map_err(|_| Error::new(EINVAL))?;
        slot.store(value, Ordering::Relaxed);
    }
    Ok(arg.len())
}

pub fn mem_resource(_token: &mut CleanLockToken) -> Result<Vec<u8>> {
    let table = REQUEST[0].load(Ordering::Relaxed);
    let mut addr = REQUEST[1].load(Ordering::Relaxed);
    let mut left = REQUEST[2].load(Ordering::Relaxed).min(MAX_READ);
    let mut out = Vec::with_capacity(left);
    while left > 0 {
        let Some(source) = resolve(table, addr) else {
            break;
        };
        let chunk = left.min(RmmA::PAGE_SIZE - (addr & (RmmA::PAGE_SIZE - 1)));
        for i in 0..chunk {
            out.push(unsafe { (source as *const u8).add(i).read_volatile() });
        }
        addr = addr.wrapping_add(chunk);
        left -= chunk;
    }
    Ok(out)
}

/// A kernel address through which the byte at `addr` can be read safely, or
/// `None` when nothing readable is there.
fn resolve(table: usize, addr: usize) -> Option<usize> {
    const KSEG0: usize = 0x8000_0000;
    const KSEG1: usize = 0xA000_0000;
    const KSEG2: usize = 0xC000_0000;
    const PROM: core::ops::Range<usize> = 0xBFC0_0000..0xBFC8_0000;
    let ram = crate::arch::consts::RAM_PHYS_BASE
        ..crate::arch::consts::RAM_PHYS_BASE + crate::arch::start::ram_size();
    if (KSEG0..KSEG1).contains(&addr) {
        let phys = addr - KSEG0;
        return (phys < 0x8_0000 || ram.contains(&phys)).then_some(addr);
    }
    if (KSEG1..KSEG2).contains(&addr) {
        // Devices have read side effects; only the PROM is offered.
        return PROM.contains(&addr).then_some(addr);
    }
    // TLB-mapped: walk the table by hand, through kseg0.
    let root = if table != 0 && addr < KSEG0 {
        table
    } else {
        RmmA::table(rmm::TableKind::User).data()
    };
    if !ram.contains(&root) {
        return None;
    }
    unsafe {
        let directory = (root + KSEG0) as *const usize;
        let pde = directory.add(addr >> 22).read_volatile();
        if pde & RmmA::ENTRY_FLAG_PRESENT == 0 {
            return None;
        }
        let table_phys =
            ((pde >> RmmA::ENTRY_ADDRESS_SHIFT) & RmmA::ENTRY_ADDRESS_MASK) << RmmA::PAGE_SHIFT;
        if !ram.contains(&table_phys) {
            return None;
        }
        let pte = ((table_phys + KSEG0) as *const usize)
            .add((addr >> 12) & 0x3ff)
            .read_volatile();
        if pte & RmmA::ENTRY_FLAG_PRESENT == 0 || pte & rmm::mips::PTE_UNCACHED != 0 {
            return None;
        }
        let frame =
            ((pte >> RmmA::ENTRY_ADDRESS_SHIFT) & RmmA::ENTRY_ADDRESS_MASK) << RmmA::PAGE_SHIFT;
        ram.contains(&frame).then_some(frame + KSEG0 + (addr & 0xfff))
    }
}
