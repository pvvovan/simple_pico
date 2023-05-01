#define SIO_BASE 0xd0000000
#define SIO_CPUID_OFFSET 0x00000000
#define PPB_BASE 0xe0000000
#define M0PLUS_VTOR_OFFSET 0x0000ed08


// Reset handler:
// - initialises .data
// - clears .bss
// - calls runtime_init
// - calls main
// - calls exit (which should eventually hang the processor via _exit)

.type _reset_handler,%function
.thumb_func
.global _reset_handler
_reset_handler:
    // Only core 0 should run the C runtime startup code; core 1 is normally
    // sleeping in the bootrom at this point but check to be sure
    ldr r0, =SIO_BASE + SIO_CPUID_OFFSET
    ldr r0, [r0]
    cmp r0, #0
    bne hold_non_core0_in_bootrom
    b myloop

myloop:
    bl startup
    bkpt #0
    nop
    nop
    nop
    nop
    b myloop
    bkpt #1
    bkpt #2
    bkpt #3


.section .vectors, "ax"
.align 2

.global __vectors, __VECTOR_TABLE
__VECTOR_TABLE:
__vectors:
.word __StackTop
.word _reset_handler

// ----------------------------------------------------------------------------
// If core 1 somehow gets into crt0 due to a spectacular VTOR mishap, we need to
// catch it and send back to the sleep-and-launch code in the bootrom. Shouldn't
// happen (it should sleep in the ROM until given an entry point via the
// cross-core FIFOs) but it's good to be defensive.

hold_non_core0_in_bootrom:
    ldr r0, = 'W' | ('V' << 8)
    bl rom_func_lookup
    bx r0
