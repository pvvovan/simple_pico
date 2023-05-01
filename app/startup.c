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
