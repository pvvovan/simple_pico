volatile long g_val = 123;
volatile long g_valu;
volatile const long c_val = 321;

int main()
{
	for ( ; ; ) {
		for (volatile long long i = 0; i < 1000000; i++) { }
		g_val++;
		g_valu += c_val;
	}

	return 0;
}
