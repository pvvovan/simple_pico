#include <stdint.h>


// Bootrom function: rom_table_lookup
// Returns the 32 bit pointer into the ROM if found or NULL otherwise.
typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);

/*!
 * \brief Lookup a bootrom function by code. This method is forcibly inlined into the caller for FLASH/RAM sensitive code usage
 * \ingroup pico_bootrom
 * \param code the code
 * \return a pointer to the function, or NULL if the code does not match any bootrom function
 */
static inline void *rom_func_lookup_inline(uint32_t code) {
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn)0x18;
    uint16_t *func_table = (uint16_t *)0x14;
    return rom_table_lookup(func_table, code);
}

void *rom_func_lookup(uint32_t code) {
    return rom_func_lookup_inline(code);
}

void isr_hardfault(void)
{
	for ( ; ; ) { }
}
