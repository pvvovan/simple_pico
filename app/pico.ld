MEMORY
{
    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 2048k
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 256k
    SCRATCH_X(rwx) : ORIGIN = 0x20040000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20041000, LENGTH = 4k
}

ENTRY(_reset_handler)

SECTIONS
{
    /* Second stage bootloader is prepended to the image. It must be 256 bytes big
       and checksummed. It is usually built by the boot_stage2 target
       in the Raspberry Pi Pico SDK
    */

    .boot2 : {
        __boot2_start__ = .;
        KEEP (*(.boot2))
        __boot2_end__ = .;
    } > FLASH

    ASSERT(__boot2_end__ - __boot2_start__ == 256,
        "ERROR: Pico second stage bootloader must be 256 bytes in size")

    /* The second stage will always enter the image at the start of .text.
       The debugger will use the ELF entry point, which is the _entry_point
       symbol if present, otherwise defaults to start of .text.
       This can be used to transfer control back to the bootrom on debugger
       launches only, to perform proper flash setup.
    */

    .text : {
        KEEP (*(.vectors))
        . += ALIGN(4);
        *(.text)
        *(.text*)
        . += ALIGN(4);
    } > FLASH

    .rodata : {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . += ALIGN(4);
    } > FLASH

    .data : {
        . = ALIGN(4);
        __SYM_databegin__ = .;
        *(.data)
        *(.data*)
        . += ALIGN(4);
        __SYM_dataend__ = .;
    } > RAM AT > FLASH
    __SYM_loaddatabegin__ = LOADADDR(.data);

    .bss : {
        . = ALIGN(4);
        __SYM_bssbegin__ = .;
        *(.bss)
        *(.bss*)
        . += ALIGN(4);
        __SYM_bssend__ = .;
    } > RAM

    __StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
}
