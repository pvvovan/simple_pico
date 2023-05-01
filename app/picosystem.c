#include <stdint.h>

int main(void);

extern const void *SYM_loaddatabegin;
extern void *SYM_databegin;
extern void *SYM_dataend;
extern void *SYM_bssbegin;
extern void *SYM_bssend;

static void initram(void)
{
	volatile long *dest = (volatile long *)&SYM_databegin;
	const long *src = (const long *)&SYM_loaddatabegin;
	while (dest < (volatile long *)&SYM_dataend) {
		*dest = *src;
		src++;
		dest++;
	}

	dest = (volatile long *)&SYM_bssbegin;
	while (dest < (volatile long *)&SYM_bssend) {
		*dest = 0;
		dest++;
	}
}

void startup(void)
{
	initram();
	main();
	while (1) { }
}


// Bootrom function: rom_table_lookup
// Returns the 32 bit pointer into the ROM if found or NULL otherwise.
typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);


#if defined(__GNUC__) && (__GNUC__ >= 12)
// Convert a 16 bit pointer stored at the given rom address into a 32 bit pointer
static inline void *rom_hword_as_ptr(uint16_t rom_address) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
    return (void *)(uintptr_t)*(uint16_t *)(uintptr_t)rom_address;
#pragma GCC diagnostic pop
}
#else
// Convert a 16 bit pointer stored at the given rom address into a 32 bit pointer
#define rom_hword_as_ptr(rom_address) (void *)(uintptr_t)(*(uint16_t *)(uintptr_t)(rom_address))
#endif


/*!
 * \brief Lookup a bootrom function by code. This method is forcibly inlined into the caller for FLASH/RAM sensitive code usage
 * \ingroup pico_bootrom
 * \param code the code
 * \return a pointer to the function, or NULL if the code does not match any bootrom function
 */
static inline void *rom_func_lookup_inline(uint32_t code) {
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) rom_hword_as_ptr(0x18);
    uint16_t *func_table = (uint16_t *) rom_hword_as_ptr(0x14);
    return rom_table_lookup(func_table, code);
}

void *rom_func_lookup(uint32_t code) {
    return rom_func_lookup_inline(code);
}

void isr_hardfault(void)
{
	for ( ; ; ) { }
}
