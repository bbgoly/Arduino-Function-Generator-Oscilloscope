#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define LOOKUP_SIZE 256

uint16_t lut[LOOKUP_SIZE];

//int test = (int)(1023.0 * sin(2 * M_PI* (float)i/LOOKUP_SIZE)*2047.0);

int main(void) {
	printf("uint16_t sine_lut[LOOKUP_SIZE] = {");
	for (int i = 0; i < LOOKUP_SIZE; i++) {
		lut[i] = (uint16_t)(2047.0 * (sin(2.f * M_PI * (float)i / LOOKUP_SIZE) + 1.f));
		printf("%d,", lut[i]);
	}
	printf("};\n\n");
	
	// used to more easily see value stored at specific index of lookup table
	// to confirm if correct
	int in = 0;
	char esc = 'x';
	do {
	    printf("value at index (x to escape): ");
	    scanf("%d", &in);
	    printf("%d\n", lut[in]);
	} while (getchar() != esc);
	return 0;
}