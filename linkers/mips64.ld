/* SGI Indy (IP24). The PROM loads the ECOFF image at kseg0 0x8804_0000
 * (physical 0x0804_0000) and jumps to kstart. Virtual and load addresses are
 * equal: kseg0 is the cached direct map, so there is no AT() offset. .bss is
 * kept separate and zeroed by kstart; the PROM does not promise to clear it. */
ENTRY(kstart)
OUTPUT_FORMAT("elf32-ntradbigmips")
OUTPUT_ARCH(mips)

KERNEL_OFFSET = 0x88040000;

SECTIONS {
    . = KERNEL_OFFSET;

    .text : {
        __text_start = .;
        KEEP(*(.text.kstart))
        *(.text*)
        . = ALIGN(4096);
        __text_end = .;
    }

    .rodata : {
        __rodata_start = .;
        *(.rodata*)
        . = ALIGN(4096);
        __rodata_end = .;
    }

    .data : {
        *(.data*)
        *(.sdata*)
        *(.got*)
        . = ALIGN(4096);
    }

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(.sbss*)
        *(COMMON)
        . = ALIGN(4096);
        __bss_end = .;
    }

    __end = .;

    /DISCARD/ : {
        *(.comment*)
        *(.eh_frame*)
        *(.gcc_except_table*)
        *(.note*)
        *(.rel.eh_frame*)
        *(.MIPS.abiflags)
        *(.MIPS.options)
        *(.reginfo)
        *(.pdr)
        *(.mdebug*)
        *(.gnu.attributes)
    }
}
