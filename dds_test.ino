#include <stdint.h>
#include <math.h>
#include "circ_buffer.h"

// probably use 32 bit phase accumulator for smaller frequency resolution (SAMPLING_RATE / 2^32 = 1e-5 s = 10 uS)
// phase accumulator is 8 bits, so frequency resolution of SAMPLING_RATE / 2^8 = 3906.25 Hz lol
#define LOOKUP_SIZE 512
#define SAMPLING_RATE 100000.0

// Macros for testing functionalities
#define TEST_DAC 0

float outputFreq;

uint32_t tuningWord;
volatile uint32_t phaseIncrement;
volatile uint32_t phaseAccumulator;

uint8_t tableAddrWidth;
uint8_t pollBufferCounter;

uint16_t sine_lut[LOOKUP_SIZE] = {2047,2072,2097,2122,2147,2172,2197,2222,2247,2272,2297,2322,2347,2372,2396,2421,2446,2470,2495,2519,2544,2568,2592,2617,2641,2665,2689,2712,2736,2760,2783,2807,2830,2853,2876,2899,2922,2944,2967,2989,3011,3034,3055,3077,3099,3120,3142,3163,3184,3205,3225,3246,3266,3286,3306,3326,3345,3364,3384,3402,3421,3440,3458,3476,3494,3512,3529,3546,3563,3580,3597,3613,3629,3645,3660,3676,3691,3706,3720,3734,3749,3762,3776,3789,3802,3815,3828,3840,3852,3863,3875,3886,3897,3908,3918,3928,3938,3947,3956,3965,3974,3982,3990,3998,4005,4013,4019,4026,4032,4038,4044,4049,4054,4059,4063,4068,4071,4075,4078,4081,4084,4086,4088,4090,4091,4092,4093,4093,4094,4093,4093,4092,4091,4090,4088,4086,4084,4081,4078,4075,4071,4068,4063,4059,4054,4049,4044,4038,4032,4026,4019,4013,4005,3998,3990,3982,3974,3965,3956,3947,3938,3928,3918,3908,3897,3886,3875,3863,3852,3840,3828,3815,3802,3789,3776,3762,3749,3734,3720,3706,3691,3676,3660,3645,3629,3613,3597,3580,3563,3546,3529,3512,3494,3476,3458,3440,3421,3402,3384,3364,3345,3326,3306,3286,3266,3246,3225,3205,3184,3163,3142,3120,3099,3077,3055,3034,3011,2989,2967,2944,2922,2899,2876,2853,2830,2807,2783,2760,2736,2712,2689,2665,2641,2617,2592,2568,2544,2519,2495,2470,2446,2421,2396,2372,2347,2322,2297,2272,2247,2222,2197,2172,2147,2122,2097,2072,2047,2021,1996,1971,1946,1921,1896,1871,1846,1821,1796,1771,1746,1721,1697,1672,1647,1623,1598,1574,1549,1525,1501,1476,1452,1428,1404,1381,1357,1333,1310,1286,1263,1240,1217,1194,1171,1149,1126,1104,1082,1059,1038,1016,994,973,951,930,909,888,868,847,827,807,787,767,748,729,709,691,672,653,635,617,599,581,564,547,530,513,496,480,464,448,433,417,402,387,373,359,344,331,317,304,291,278,265,253,241,230,218,207,196,185,175,165,155,146,137,128,119,111,103,95,88,80,74,67,61,55,49,44,39,34,30,25,22,18,15,12,9,7,5,3,2,1,0,0,0,0,0,1,2,3,5,7,9,12,15,18,22,25,30,34,39,44,49,55,61,67,74,80,88,95,103,111,119,128,137,146,155,165,175,185,196,207,218,230,241,253,265,278,291,304,317,331,344,359,373,387,402,417,433,448,464,480,496,513,530,547,564,581,599,617,635,653,672,691,709,729,748,767,787,807,827,847,868,888,909,930,951,973,994,1016,1038,1059,1082,1104,1126,1149,1171,1194,1217,1240,1263,1286,1310,1333,1357,1381,1404,1428,1452,1476,1501,1525,1549,1574,1598,1623,1647,1672,1697,1721,1746,1771,1796,1821,1846,1871,1896,1921,1946,1971,1996,2021};

volatile bool printFlag;

CIRC_BUFFER_DEF(pollBuffer, LOOKUP_SIZE / 2);

void setup() {
    Serial.begin(9600);
    outputFreq = 1000.0;
    tableAddrWidth = log2(LOOKUP_SIZE);
    tuningWord = pow(2, 32) * outputFreq / SAMPLING_RATE; // Fout = increment*Fsample/(2^32)
       
    // analogReadResolution(12);
    // pinMode(A0, INPUT);

    analogWriteResolution(12);

    ConfigureTC();
    // ConfigurePDCDMA();
    // ConfigureSPI();
    // ConfigureDACC();
    // ConfigureADCC();
    
    analogWrite(DAC0, 0);
    

    // pmc_enable_periph_clk(DACC_INTERFACE_ID);
    // DACC->DACC_CR

    // attachInterrupt(digitalPinToInterrupt(30), button, RISING);

#if TEST_DAC
    delay(2000);
    const float initialFreq = outputFreq, inc = initialFreq / 10;
    for (int i = initialFreq; i <= 10*initialFreq; i += inc) {
        outputFreq = i;
        tuningWord = pow(2, 32) * outputFreq / SAMPLING_RATE;
        Serial.println(outputFreq);
        delay(1000);
    }
#endif
}

void ConfigureTC() {
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk(ID_TC4);
    // pmc_enable_periph_clk(DACC_INTERFACE_ID);

    TC_Configure(TC1, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
    // TC_SetRA(TC1, 1, VARIANT_MCK / 128 / SAMPLING_RATE / 2);
    TC_SetRC(TC1, 1, VARIANT_MCK / 128 / SAMPLING_RATE); // 8 for 44100
    TC_Start(TC1, 1);

    TC1 -> TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
    TC1 -> TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;

    NVIC_EnableIRQ(TC4_IRQn);
}

void loop() {
    uint16_t data;
    if (printFlag && circ_buffer_pop(&pollBuffer, &data) == 0) {
        printFlag = circ_buffer_is_empty(&pollBuffer);
        Serial.println(data);
    }
}

void TC4_Handler() {
    TC_GetStatus(TC1, 1);

    // We only want to index using the top log2(LOOKUP_SIZE) bits, e.g., if LOOKUP_SIZE = 512, then we index using the top 9 bits (2^9 = 512 and 32 bits - 9 bits is shift right by 23 bits)
    phaseAccumulator += tuningWord;
    phaseIncrement = phaseAccumulator >> (32 - tableAddrWidth);
    
    dacc_write_conversion_data(DACC_INTERFACE, sine_lut[phaseIncrement]);
    // analogWrite(DAC0, sine_lut[phaseIncrement]);
    // Serial.println(sine_lut[phaseIncrement]);
    circ_buffer_push(&pollBuffer, sine_lut[phaseIncrement]);
    printFlag = true;
}