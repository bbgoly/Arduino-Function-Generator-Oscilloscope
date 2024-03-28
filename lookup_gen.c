#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define LOOKUP_SIZE 256

int16_t sine_lut[LOOKUP_SIZE];

int main() {
	printf("int16_t sine_lut[LOOKUP_SIZE] = {");
	for (int i = 0; i < LOOKUP_SIZE; i++) {
		sine_lut[i] = (int16_t)(2047.0 * (sin(2.f * M_PI * (float)i / LOOKUP_SIZE) + 1.f));
		printf("%d,", sine_lut[i]);
	}
	printf("};\n\n");
	
	int in = 0;
	char esc = 'x';
	do {
	    printf("value at index (x to escape): ");
	    scanf("%d", &in);
	    printf("%d\n", sine_lut[in]);
	} while (getchar() != esc);
	return 0;
}