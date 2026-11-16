//! CP0 (system control coprocessor) registers of the R4400.

use core::arch::asm;

pub const INDEX: u32 = 0;
pub const ENTRY_LO0: u32 = 2;
pub const ENTRY_LO1: u32 = 3;
pub const CONTEXT: u32 = 4;
pub const PAGE_MASK: u32 = 5;
pub const WIRED: u32 = 6;
pub const BAD_VADDR: u32 = 8;
pub const COUNT: u32 = 9;
pub const ENTRY_HI: u32 = 10;
pub const COMPARE: u32 = 11;
pub const STATUS: u32 = 12;
pub const CAUSE: u32 = 13;
pub const EPC: u32 = 14;
pub const PRID: u32 = 15;
pub const CONFIG: u32 = 16;

/// Status register bits.
pub mod status {
    pub const IE: u32 = 1 << 0;
    pub const EXL: u32 = 1 << 1;
    pub const ERL: u32 = 1 << 2;
    /// Kernel/supervisor/user mode field: 0 kernel, 2 user.
    pub const KSU_MASK: u32 = 0x18;
    pub const KSU_USER: u32 = 0x10;
    pub const UX: u32 = 1 << 5;
    pub const SX: u32 = 1 << 6;
    pub const KX: u32 = 1 << 7;
    pub const IM_MASK: u32 = 0xff00;
    /// INT3 local 0 (IP2) and local 1 (IP3).
    pub const IM2: u32 = 1 << 10;
    pub const IM3: u32 = 1 << 11;
    /// Timer interrupt (IP7).
    pub const IM7: u32 = 1 << 15;
    /// Interrupt inputs every context runs with: timer and both INT3 cascades.
    pub const IM_ENABLED: u32 = IM7 | IM2 | IM3;
    /// Bootstrap exception vectors (PROM) when set.
    pub const BEV: u32 = 1 << 22;
    pub const CU1: u32 = 1 << 29;
}

/// Cause register bits.
pub mod cause {
    pub const EXC_CODE_MASK: u32 = 0x7c;
    pub const EXC_CODE_SHIFT: u32 = 2;
    pub const IP_MASK: u32 = 0xff00;
    pub const IP2: u32 = 1 << 10;
    pub const IP3: u32 = 1 << 11;
    pub const IP7: u32 = 1 << 15;
    /// Exception in a branch delay slot.
    pub const BD: u32 = 1 << 31;

    pub const EXC_INT: u32 = 0;
    pub const EXC_MOD: u32 = 1;
    pub const EXC_TLBL: u32 = 2;
    pub const EXC_TLBS: u32 = 3;
    pub const EXC_ADEL: u32 = 4;
    pub const EXC_ADES: u32 = 5;
    pub const EXC_IBE: u32 = 6;
    pub const EXC_DBE: u32 = 7;
    pub const EXC_SYS: u32 = 8;
    pub const EXC_BP: u32 = 9;
    pub const EXC_RI: u32 = 10;
    pub const EXC_CPU: u32 = 11;
    pub const EXC_OV: u32 = 12;
    pub const EXC_TR: u32 = 13;
    pub const EXC_FPE: u32 = 15;
}

macro_rules! cp0_access {
    ($read:ident, $write:ident, $reg:expr) => {
        #[inline(always)]
        pub fn $read() -> u32 {
            let value: u32;
            unsafe {
                asm!(concat!("mfc0 {}, $", $reg), out(reg) value, options(nomem, nostack));
            }
            value
        }

        /// # Safety
        /// Writing CP0 changes CPU state: translation, interrupts or timing.
        #[inline(always)]
        pub unsafe fn $write(value: u32) {
            unsafe {
                asm!(
                    concat!("mtc0 {}, $", $reg),
                    "nop",
                    "nop",
                    in(reg) value,
                    options(nomem, nostack)
                );
            }
        }
    };
}

cp0_access!(read_status, write_status, 12);
cp0_access!(read_cause, write_cause, 13);
cp0_access!(read_epc, write_epc, 14);
cp0_access!(read_count, write_count, 9);
cp0_access!(read_compare, write_compare, 11);
cp0_access!(read_bad_vaddr, write_bad_vaddr, 8);
cp0_access!(read_prid, write_prid, 15);
cp0_access!(read_config, write_config, 16);
cp0_access!(read_entry_hi, write_entry_hi, 10);
cp0_access!(read_wired, write_wired, 6);
cp0_access!(read_page_mask, write_page_mask, 5);
cp0_access!(read_context, write_context, 4);
