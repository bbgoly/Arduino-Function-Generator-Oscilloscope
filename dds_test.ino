#include <stdint.h>
#include <math.h>
#include "circ_buffer.h"

// probably use 32 bit phase accumulator for smaller frequency resolution (SAMPLING_RATE / 2^32 = 1e-5 s = 10 uS)
// phase accumulator is 8 bits, so frequency resolution of SAMPLING_RATE / 2^8 = 3906.25 Hz lol
#define LOOKUP_SIZE 256
#define SAMPLING_RATE 44100.0

float outputFreq;

uint32_t tuningWord;
volatile uint32_t phaseIncrement;
volatile uint32_t phaseAccumulator;

uint8_t tableAddrWidth;
uint8_t pollBufferCounter;

uint16_t sine_lut[LOOKUP_SIZE] = {2047,2097,2147,2197,2247,2297,2347,2396,2446,2495,2544,2592,2641,2689,2736,2783,2830,2876,2922,2967,3011,3055,3099,3142,3184,3225,3266,3306,3345,3384,3421,3458,3494,3529,3563,3597,3629,3660,3691,3720,3749,3776,3802,3828,3852,3875,3897,3918,3938,3956,3974,3990,4005,4019,4032,4044,4054,4063,4071,4078,4084,4088,4091,4093,4094,4093,4091,4088,4084,4078,4071,4063,4054,4044,4032,4019,4005,3990,3974,3956,3938,3918,3897,3875,3852,3828,3802,3776,3749,3720,3691,3660,3629,3597,3563,3529,3494,3458,3421,3384,3345,3306,3266,3225,3184,3142,3099,3055,3011,2967,2922,2876,2830,2783,2736,2689,2641,2592,2544,2495,2446,2396,2347,2297,2247,2197,2147,2097,2047,1996,1946,1896,1846,1796,1746,1697,1647,1598,1549,1501,1452,1404,1357,1310,1263,1217,1171,1126,1082,1038,994,951,909,868,827,787,748,709,672,635,599,564,530,496,464,433,402,373,344,317,291,265,241,218,196,175,155,137,119,103,88,74,61,49,39,30,22,15,9,5,2,0,0,0,2,5,9,15,22,30,39,49,61,74,88,103,119,137,155,175,196,218,241,265,291,317,344,373,402,433,464,496,530,564,599,635,672,709,748,787,827,868,909,951,994,1038,1082,1126,1171,1217,1263,1310,1357,1404,1452,1501,1549,1598,1647,1697,1746,1796,1846,1896,1946,1996};

volatile bool printFlag;

CIRC_BUFFER_DEF(pollBuffer, LOOKUP_SIZE / 4);

void setup() {
    Serial.begin(9600);
    outputFreq = 1000.0;
    tableAddrWidth = log2(LOOKUP_SIZE);
    tuningWord = pow(2, 32) * outputFreq / SAMPLING_RATE;
       
    analogReadResolution(12);
    pinMode(A0, INPUT);

    analogWriteResolution(12);

    setupTC();
    
    analogWrite(DAC0, 0);
    // testFrequencies();
}

void testFrequencies() {
    delay(2000);
    const float initialFreq = outputFreq, inc = initialFreq / 10;
    for (int i = initialFreq; i <= 10*initialFreq; i += inc) {
        outputFreq = i;
        tuningWord = pow(2, 32) * outputFreq / SAMPLING_RATE;
        Serial.println(outputFreq);
        delay(1000);
    }
}

void setupTC() {
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk(ID_TC4);
    // pmc_enable_periph_clk(DACC_INTERFACE_ID);

    TC_Configure(TC1, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
    // TC_SetRA(TC1, 1, VARIANT_MCK / 8 / SAMPLING_RATE / 2);
    TC_SetRC(TC1, 1, VARIANT_MCK / 8 / SAMPLING_RATE);
    TC_Start(TC1, 1);

    TC1 -> TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
    TC1 -> TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;

    NVIC_EnableIRQ(TC4_IRQn);
}

void loop() {
    uint16_t data;
    if (printFlag && circ_buffer_pop(&pollBuffer, &data) == 0 && data != 0) {
        printFlag = false;
        Serial.println(data);
        delay(1000.0 / SAMPLING_RATE);
    }
}

void TC4_Handler() {
    TC_GetStatus(TC1, 1);

    phaseAccumulator += tuningWord;
    phaseIncrement = phaseAccumulator >> (32 - tableAddrWidth);
    
    dacc_write_conversion_data(DACC_INTERFACE, sine_lut[phaseIncrement]);
    // analogWrite(DAC0, sine_lut[phaseIncrement]);
    circ_buffer_push(&pollBuffer, sine_lut[phaseIncrement]);
    printFlag = true;
}