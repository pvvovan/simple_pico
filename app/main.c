volatile long g_val = 123;
volatile long g_valu[2];
volatile const long c_val = 321;

int main()
{
	for ( ; ; ) {
		for (volatile long long i = 0; i < 100000; i++) { }
		g_val++;
		g_valu[1] += c_val;
	}

	return 0;
}
