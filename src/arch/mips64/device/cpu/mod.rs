use core::fmt::{Result, Write};

use crate::arch::cp0;

pub fn cpu_info<W: Write>(w: &mut W) -> Result {
    let prid = cp0::read_prid();
    writeln!(w, "Implementation: 0x{:02x}", (prid >> 8) & 0xff)?;
    writeln!(w, "Revision: 0x{:02x}", prid & 0xff)?;
    writeln!(w, "Config: 0x{:08x}", cp0::read_config())?;
    Ok(())
}
