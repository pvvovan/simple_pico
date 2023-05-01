// ELF entry point:
.type _entry_point,%function
.thumb_func
.global _entry_point
_entry_point:
	bkpt #0
	bkpt #1
	bkpt #2
	bkpt #3
