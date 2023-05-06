#include <stdint.h>
#include <stddef.h>
#include "core1.h"


typedef volatile uint32_t io_rw_32;
typedef const volatile uint32_t io_ro_32;
typedef volatile uint32_t io_wo_32;

#define SIO_FIFO_ST_VLD_BITS	0x00000001uL
#define SIO_FIFO_ST_RDY_BITS	0x00000002uL


typedef struct {
//     _REG_(SIO_INTERP0_ACCUM0_OFFSET) // SIO_INTERP0_ACCUM0
    // (Description copied from array index 0 register SIO_INTERP0_ACCUM0 applies similarly to other array indexes)
    //
    // Read/write access to accumulator 0
    io_rw_32 accum[2];

//     _REG_(SIO_INTERP0_BASE0_OFFSET) // SIO_INTERP0_BASE0
    // (Description copied from array index 0 register SIO_INTERP0_BASE0 applies similarly to other array indexes)
    //
    // Read/write access to BASE0 register
    io_rw_32 base[3];

//     _REG_(SIO_INTERP0_POP_LANE0_OFFSET) // SIO_INTERP0_POP_LANE0
    // (Description copied from array index 0 register SIO_INTERP0_POP_LANE0 applies similarly to other array indexes)
    //
    // Read LANE0 result, and simultaneously write lane results to both accumulators (POP)
    io_ro_32 pop[3];

//     _REG_(SIO_INTERP0_PEEK_LANE0_OFFSET) // SIO_INTERP0_PEEK_LANE0
    // (Description copied from array index 0 register SIO_INTERP0_PEEK_LANE0 applies similarly to other array indexes)
    //
    // Read LANE0 result, without altering any internal state (PEEK)
    io_ro_32 peek[3];

//     _REG_(SIO_INTERP0_CTRL_LANE0_OFFSET) // SIO_INTERP0_CTRL_LANE0
    // (Description copied from array index 0 register SIO_INTERP0_CTRL_LANE0 applies similarly to other array indexes)
    //
    // Control register for lane 0
    // 0x02000000 [25]    : OVERF (0): Set if either OVERF0 or OVERF1 is set
    // 0x01000000 [24]    : OVERF1 (0): Indicates if any masked-off MSBs in ACCUM1 are set
    // 0x00800000 [23]    : OVERF0 (0): Indicates if any masked-off MSBs in ACCUM0 are set
    // 0x00200000 [21]    : BLEND (0): Only present on INTERP0 on each core
    // 0x00180000 [20:19] : FORCE_MSB (0): ORed into bits 29:28 of the lane result presented to the processor on the bus
    // 0x00040000 [18]    : ADD_RAW (0): If 1, mask + shift is bypassed for LANE0 result
    // 0x00020000 [17]    : CROSS_RESULT (0): If 1, feed the opposite lane's result into this lane's accumulator on POP
    // 0x00010000 [16]    : CROSS_INPUT (0): If 1, feed the opposite lane's accumulator into this lane's shift + mask hardware
    // 0x00008000 [15]    : SIGNED (0): If SIGNED is set, the shifted and masked accumulator value is sign-extended to 32 bits
    // 0x00007c00 [14:10] : MASK_MSB (0): The most-significant bit allowed to pass by the mask (inclusive)
    // 0x000003e0 [9:5]   : MASK_LSB (0): The least-significant bit allowed to pass by the mask (inclusive)
    // 0x0000001f [4:0]   : SHIFT (0): Logical right-shift applied to accumulator before masking
    io_rw_32 ctrl[2];

//     _REG_(SIO_INTERP0_ACCUM0_ADD_OFFSET) // SIO_INTERP0_ACCUM0_ADD
    // (Description copied from array index 0 register SIO_INTERP0_ACCUM0_ADD applies similarly to other array indexes)
    //
    // Values written here are atomically added to ACCUM0
    // 0x00ffffff [23:0]  : INTERP0_ACCUM0_ADD (0)
    io_rw_32 add_raw[2];

//     _REG_(SIO_INTERP0_BASE_1AND0_OFFSET) // SIO_INTERP0_BASE_1AND0
    // On write, the lower 16 bits go to BASE0, upper bits to BASE1 simultaneously
    io_wo_32 base01;
} interp_hw_t;

#define SIO_BASE	0xd0000000uL
typedef struct {
//     _REG_(SIO_CPUID_OFFSET) // SIO_CPUID
    // Processor core identifier
    io_ro_32 cpuid;

//     _REG_(SIO_GPIO_IN_OFFSET) // SIO_GPIO_IN
    // Input value for GPIO pins
    // 0x3fffffff [29:0]  : GPIO_IN (0): Input value for GPIO0
    io_ro_32 gpio_in;

//     _REG_(SIO_GPIO_HI_IN_OFFSET) // SIO_GPIO_HI_IN
    // Input value for QSPI pins
    // 0x0000003f [5:0]   : GPIO_HI_IN (0): Input value on QSPI IO in order 0
    io_ro_32 gpio_hi_in;

    uint32_t _pad0;

//     _REG_(SIO_GPIO_OUT_OFFSET) // SIO_GPIO_OUT
    // GPIO output value
    // 0x3fffffff [29:0]  : GPIO_OUT (0): Set output level (1/0 -> high/low) for GPIO0
    io_rw_32 gpio_out;

//     _REG_(SIO_GPIO_OUT_SET_OFFSET) // SIO_GPIO_OUT_SET
    // GPIO output value set
    // 0x3fffffff [29:0]  : GPIO_OUT_SET (0): Perform an atomic bit-set on GPIO_OUT, i
    io_wo_32 gpio_set;

//     _REG_(SIO_GPIO_OUT_CLR_OFFSET) // SIO_GPIO_OUT_CLR
    // GPIO output value clear
    // 0x3fffffff [29:0]  : GPIO_OUT_CLR (0): Perform an atomic bit-clear on GPIO_OUT, i
    io_wo_32 gpio_clr;

//     _REG_(SIO_GPIO_OUT_XOR_OFFSET) // SIO_GPIO_OUT_XOR
    // GPIO output value XOR
    // 0x3fffffff [29:0]  : GPIO_OUT_XOR (0): Perform an atomic bitwise XOR on GPIO_OUT, i
    io_wo_32 gpio_togl;

//     _REG_(SIO_GPIO_OE_OFFSET) // SIO_GPIO_OE
    // GPIO output enable
    // 0x3fffffff [29:0]  : GPIO_OE (0): Set output enable (1/0 -> output/input) for GPIO0
    io_rw_32 gpio_oe;

//     _REG_(SIO_GPIO_OE_SET_OFFSET) // SIO_GPIO_OE_SET
    // GPIO output enable set
    // 0x3fffffff [29:0]  : GPIO_OE_SET (0): Perform an atomic bit-set on GPIO_OE, i
    io_wo_32 gpio_oe_set;

//     _REG_(SIO_GPIO_OE_CLR_OFFSET) // SIO_GPIO_OE_CLR
    // GPIO output enable clear
    // 0x3fffffff [29:0]  : GPIO_OE_CLR (0): Perform an atomic bit-clear on GPIO_OE, i
    io_wo_32 gpio_oe_clr;

//     _REG_(SIO_GPIO_OE_XOR_OFFSET) // SIO_GPIO_OE_XOR
    // GPIO output enable XOR
    // 0x3fffffff [29:0]  : GPIO_OE_XOR (0): Perform an atomic bitwise XOR on GPIO_OE, i
    io_wo_32 gpio_oe_togl;

//     _REG_(SIO_GPIO_HI_OUT_OFFSET) // SIO_GPIO_HI_OUT
    // QSPI output value
    // 0x0000003f [5:0]   : GPIO_HI_OUT (0): Set output level (1/0 -> high/low) for QSPI IO0
    io_rw_32 gpio_hi_out;

//     _REG_(SIO_GPIO_HI_OUT_SET_OFFSET) // SIO_GPIO_HI_OUT_SET
    // QSPI output value set
    // 0x0000003f [5:0]   : GPIO_HI_OUT_SET (0): Perform an atomic bit-set on GPIO_HI_OUT, i
    io_wo_32 gpio_hi_set;

//     _REG_(SIO_GPIO_HI_OUT_CLR_OFFSET) // SIO_GPIO_HI_OUT_CLR
    // QSPI output value clear
    // 0x0000003f [5:0]   : GPIO_HI_OUT_CLR (0): Perform an atomic bit-clear on GPIO_HI_OUT, i
    io_wo_32 gpio_hi_clr;

//     _REG_(SIO_GPIO_HI_OUT_XOR_OFFSET) // SIO_GPIO_HI_OUT_XOR
    // QSPI output value XOR
    // 0x0000003f [5:0]   : GPIO_HI_OUT_XOR (0): Perform an atomic bitwise XOR on GPIO_HI_OUT, i
    io_wo_32 gpio_hi_togl;

//     _REG_(SIO_GPIO_HI_OE_OFFSET) // SIO_GPIO_HI_OE
    // QSPI output enable
    // 0x0000003f [5:0]   : GPIO_HI_OE (0): Set output enable (1/0 -> output/input) for QSPI IO0
    io_rw_32 gpio_hi_oe;

//     _REG_(SIO_GPIO_HI_OE_SET_OFFSET) // SIO_GPIO_HI_OE_SET
    // QSPI output enable set
    // 0x0000003f [5:0]   : GPIO_HI_OE_SET (0): Perform an atomic bit-set on GPIO_HI_OE, i
    io_wo_32 gpio_hi_oe_set;

//     _REG_(SIO_GPIO_HI_OE_CLR_OFFSET) // SIO_GPIO_HI_OE_CLR
    // QSPI output enable clear
    // 0x0000003f [5:0]   : GPIO_HI_OE_CLR (0): Perform an atomic bit-clear on GPIO_HI_OE, i
    io_wo_32 gpio_hi_oe_clr;

//     _REG_(SIO_GPIO_HI_OE_XOR_OFFSET) // SIO_GPIO_HI_OE_XOR
    // QSPI output enable XOR
    // 0x0000003f [5:0]   : GPIO_HI_OE_XOR (0): Perform an atomic bitwise XOR on GPIO_HI_OE, i
    io_wo_32 gpio_hi_oe_togl;

//     _REG_(SIO_FIFO_ST_OFFSET) // SIO_FIFO_ST
    // Status register for inter-core FIFOs (mailboxes)
    // 0x00000008 [3]     : ROE (0): Sticky flag indicating the RX FIFO was read when empty
    // 0x00000004 [2]     : WOF (0): Sticky flag indicating the TX FIFO was written when full
    // 0x00000002 [1]     : RDY (1): Value is 1 if this core's TX FIFO is not full (i
    // 0x00000001 [0]     : VLD (0): Value is 1 if this core's RX FIFO is not empty (i
    io_rw_32 fifo_st;

//     _REG_(SIO_FIFO_WR_OFFSET) // SIO_FIFO_WR
    // Write access to this core's TX FIFO
    io_wo_32 fifo_wr;

//     _REG_(SIO_FIFO_RD_OFFSET) // SIO_FIFO_RD
    // Read access to this core's RX FIFO
    io_ro_32 fifo_rd;

//     _REG_(SIO_SPINLOCK_ST_OFFSET) // SIO_SPINLOCK_ST
    // Spinlock state
    io_ro_32 spinlock_st;

//     _REG_(SIO_DIV_UDIVIDEND_OFFSET) // SIO_DIV_UDIVIDEND
    // Divider unsigned dividend
    io_rw_32 div_udividend;

//     _REG_(SIO_DIV_UDIVISOR_OFFSET) // SIO_DIV_UDIVISOR
    // Divider unsigned divisor
    io_rw_32 div_udivisor;

//     _REG_(SIO_DIV_SDIVIDEND_OFFSET) // SIO_DIV_SDIVIDEND
    // Divider signed dividend
    io_rw_32 div_sdividend;

//     _REG_(SIO_DIV_SDIVISOR_OFFSET) // SIO_DIV_SDIVISOR
    // Divider signed divisor
    io_rw_32 div_sdivisor;

//     _REG_(SIO_DIV_QUOTIENT_OFFSET) // SIO_DIV_QUOTIENT
    // Divider result quotient
    io_rw_32 div_quotient;

//     _REG_(SIO_DIV_REMAINDER_OFFSET) // SIO_DIV_REMAINDER
    // Divider result remainder
    io_rw_32 div_remainder;

//     _REG_(SIO_DIV_CSR_OFFSET) // SIO_DIV_CSR
    // Control and status register for divider
    // 0x00000002 [1]     : DIRTY (0): Changes to 1 when any register is written, and back to 0 when QUOTIENT is read
    // 0x00000001 [0]     : READY (1): Reads as 0 when a calculation is in progress, 1 otherwise
    io_ro_32 div_csr;
    uint32_t _pad1;
    interp_hw_t interp[2];
} sio_hw_t;

#define sio_hw ((sio_hw_t *)SIO_BASE)

static inline int multicore_fifo_rvalid(void) {
    return !!(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS);
}

static inline int multicore_fifo_wready(void) {
    return !!(sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS);
}

static inline void multicore_fifo_drain(void) {
    while (multicore_fifo_rvalid())
        (void) sio_hw->fifo_rd;
}

static void tight_loop_contents(void) {}

static inline void multicore_fifo_push_blocking_inline(uint32_t data) {
    // We wait for the fifo to have some space
    while (!multicore_fifo_wready())
        tight_loop_contents();

    sio_hw->fifo_wr = data;

    // Fire off an event to the other core
//     __sev();
    asm volatile ("sev" : : : "memory");
}

static void multicore_fifo_push_blocking(uint32_t data) {
    multicore_fifo_push_blocking_inline(data);
}

static inline uint32_t multicore_fifo_pop_blocking_inline(void) {
    // If nothing there yet, we wait for an event first,
    // to try and avoid too much busy waiting
    while (!multicore_fifo_rvalid())
        // __wfe();
	asm volatile ("wfe" : : : "memory");

    return sio_hw->fifo_rd;
}

static uint32_t multicore_fifo_pop_blocking() {
    return multicore_fifo_pop_blocking_inline();
}

#define count_of(a) (sizeof(a)/sizeof((a)[0]))

static void multicore_launch_core1_raw(void (*entry)(void), uint32_t *sp, uint32_t vector_table) {
    // Allow for the fact that the caller may have already enabled the FIFO IRQ for their
    // own purposes (expecting FIFO content after core 1 is launched). We must disable
    // the IRQ during the handshake, then restore afterwards.
//     bool enabled = irq_is_enabled(SIO_IRQ_PROC0);
//     irq_set_enabled(SIO_IRQ_PROC0, false);

    // Values to be sent in order over the FIFO from core 0 to core 1
    //
    // vector_table is value for VTOR register
    // sp is initial stack pointer (SP)
    // entry is the initial program counter (PC) (don't forget to set the thumb bit!)
    const uint32_t cmd_sequence[] =
            {0, 0, 1, (uintptr_t) vector_table, (uintptr_t) sp, (uintptr_t) entry};

    uint32_t seq = 0;
    do {
        uint32_t cmd = cmd_sequence[seq];
        // Always drain the READ FIFO (from core 1) before sending a 0
        if (!cmd) {
            multicore_fifo_drain();
            // Execute a SEV as core 1 may be waiting for FIFO space via WFE
        //     __sev();
	    asm volatile ("sev" : : : "memory");
        }
        multicore_fifo_push_blocking(cmd);
        uint32_t response = multicore_fifo_pop_blocking();
        // Move to next state on correct response (echo-d value) otherwise start over
        seq = cmd == response ? seq + 1 : 0;
    } while (seq < count_of(cmd_sequence));

//     irq_set_enabled(SIO_IRQ_PROC0, enabled);
}

#define PPB_BASE 0xe0000000uL
#define M0PLUS_CPUID_OFFSET 0x0000ed00uL

typedef struct {
//     _REG_(M0PLUS_CPUID_OFFSET) // M0PLUS_CPUID
    // Read the CPU ID Base Register to determine: the ID number of the processor core, the version number of the processor...
    // 0xff000000 [31:24] : IMPLEMENTER (0x41): Implementor code: 0x41 = ARM
    // 0x00f00000 [23:20] : VARIANT (0): Major revision number n in the rnpm revision status:
    // 0x000f0000 [19:16] : ARCHITECTURE (0xc): Constant that defines the architecture of the processor:
    // 0x0000fff0 [15:4]  : PARTNO (0xc60): Number of processor within family: 0xC60 = Cortex-M0+
    // 0x0000000f [3:0]   : REVISION (1): Minor revision number m in the rnpm revision status:
    io_ro_32 cpuid;

//     _REG_(M0PLUS_ICSR_OFFSET) // M0PLUS_ICSR
    // Use the Interrupt Control State Register to set a pending Non-Maskable Interrupt (NMI), set or clear a pending...
    // 0x80000000 [31]    : NMIPENDSET (0): Setting this bit will activate an NMI
    // 0x10000000 [28]    : PENDSVSET (0): PendSV set-pending bit
    // 0x08000000 [27]    : PENDSVCLR (0): PendSV clear-pending bit
    // 0x04000000 [26]    : PENDSTSET (0): SysTick exception set-pending bit
    // 0x02000000 [25]    : PENDSTCLR (0): SysTick exception clear-pending bit
    // 0x00800000 [23]    : ISRPREEMPT (0): The system can only access this bit when the core is halted
    // 0x00400000 [22]    : ISRPENDING (0): External interrupt pending flag
    // 0x001ff000 [20:12] : VECTPENDING (0): Indicates the exception number for the highest priority pending exception: 0 =...
    // 0x000001ff [8:0]   : VECTACTIVE (0): Active exception number field
    io_rw_32 icsr;

//     _REG_(M0PLUS_VTOR_OFFSET) // M0PLUS_VTOR
    // The VTOR holds the vector table offset address
    // 0xffffff00 [31:8]  : TBLOFF (0): Bits [31:8] of the indicate the vector table offset address
    io_rw_32 vtor;

//     _REG_(M0PLUS_AIRCR_OFFSET) // M0PLUS_AIRCR
    // Use the Application Interrupt and Reset Control Register to: determine data endianness, clear all active state...
    // 0xffff0000 [31:16] : VECTKEY (0): Register key:
    // 0x00008000 [15]    : ENDIANESS (0): Data endianness implemented:
    // 0x00000004 [2]     : SYSRESETREQ (0): Writing 1 to this bit causes the SYSRESETREQ signal to the outer system to be...
    // 0x00000002 [1]     : VECTCLRACTIVE (0): Clears all active state information for fixed and configurable exceptions
    io_rw_32 aircr;

//     _REG_(M0PLUS_SCR_OFFSET) // M0PLUS_SCR
    // System Control Register
    // 0x00000010 [4]     : SEVONPEND (0): Send Event on Pending bit:
    // 0x00000004 [2]     : SLEEPDEEP (0): Controls whether the processor uses sleep or deep sleep as its low power mode:
    // 0x00000002 [1]     : SLEEPONEXIT (0): Indicates sleep-on-exit when returning from Handler mode to Thread mode:
    io_rw_32 scr;
} armv6m_scb_t;

#define scb_hw ((armv6m_scb_t *)(PPB_BASE + M0PLUS_CPUID_OFFSET))

static void __attribute__ ((naked)) core1_trampoline(void) {
    __asm("pop {r0, r1, pc}");
}

static int core1_wrapper(int (*entry)(void), void *stack_base) {
	(void)stack_base;
// #if PICO_USE_STACK_GUARDS
//     // install core1 stack guard
//     runtime_install_stack_guard(stack_base);
// #else
//     __unused void *ignore = stack_base;
// #endif
//     irq_init_priorities();
	return (*entry)();
}

static void multicore_launch_core1_with_stack(void (*entry)(void), uint32_t *stack_bottom, size_t stack_size_bytes) {
//     assert(!(stack_size_bytes & 3u));
    uint32_t *stack_ptr = stack_bottom + stack_size_bytes / sizeof(uint32_t);
    // push 2 values onto top of stack for core1_trampoline
    stack_ptr -= 3;
    stack_ptr[0] = (uintptr_t) entry;
    stack_ptr[1] = (uintptr_t) stack_bottom;
    stack_ptr[2] = (uintptr_t) core1_wrapper;
// #if PICO_VTABLE_PER_CORE
// #warning PICO_VTABLE_PER_CORE==1 is not currently supported in pico_multicore
//     panic_unsupported();
// #endif
    multicore_launch_core1_raw(core1_trampoline, stack_ptr, scb_hw->vtor);
}

#define PICO_CORE1_STACK_SIZE 8192
// Default stack for core1
static uint32_t __attribute__((section(".stack1"))) core1_stack[PICO_CORE1_STACK_SIZE / sizeof(uint32_t)];

void multicore_launch_core1(void (*entry)(void)) {
	// extern uint32_t __StackOneBottom;
	// uint32_t *stack_limit = (uint32_t *) &__StackOneBottom;
	// hack to reference core1_stack although that pointer is wrong.... core1_stack should always be <= stack_limit, if not boom!
	// uint32_t *stack = core1_stack <= stack_limit ? stack_limit : (uint32_t *) -1;
	uint32_t *stack = &core1_stack[0];
	multicore_launch_core1_with_stack(entry, stack, sizeof(core1_stack));
}
