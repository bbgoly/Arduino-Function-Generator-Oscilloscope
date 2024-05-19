#include <stdint.h>
#include "circular_buffer.h"

#define RE_SW 4
#define RE_DT 5
#define RE_CLK 6

#define LOOKUP_SIZE 16384  // 2^14 8192 16384
#define SAMPLING_RATE 1000000.0

uint16_t sine_lut[LOOKUP_SIZE];

volatile float outputFrequency;
volatile uint32_t tuningWord;

bool lastDebounce = 0;
int lastButtonState = 0;
int inputDebounceTime = 750;
int buttonHoldTimeThreshold = 1500;
ulong holdDownStartTime = 0;
bool wasHeldDown = false;

uint16_t frequencyStep = 100;

// volatile uint32_t phaseIncrement;
volatile uint32_t phaseAccumulator;

volatile uint16_t dds_size = 0;
uint16_t dds_lut[LOOKUP_SIZE];

uint16_t adc_buffer[2][512]; // ping pong double buffer
volatile bool swapReadWriteBuffer = false;

// Testing purposes
uint8_t tableWidth;

circular_buffer_t* adcBuffer; // circular queue buffer

#define USE_DMA 1

void generateDDSLookup() {
    tuningWord = pow(2, 32) * outputFrequency / SAMPLING_RATE;
    dds_size = SAMPLING_RATE / outputFrequency;  // size = 2^32 / tuningWord = Fs / Fout
    for (int i = 0; i < dds_size; i++) {
        uint32_t phaseIncrement = phaseAccumulator >> (32 - tableWidth);
        dds_lut[i] = sine_lut[phaseIncrement];
        phaseAccumulator += tuningWord;
    }
}

void setupTCDAC() {
    // To supply the clock of the DACC and trigger it externally, we can generate PWM pulses through TIOA2
    // (Timer Channel 2 on I/O Line A) using an internal clock. According to the block diagram in the datasheet,
    // to generate PWM pulses through TIOA2 we would need to enable and power on Timer Counter 0 (TC0) Channel 2 (which is TC2).
    pmc_enable_periph_clk(ID_TC2);

    // Set channel to operate on Waveform mode, which configures TIOA to be an output (this allows us to
    // generate PWM pulses to supply the DACC a clock)
    TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // Use MCK / 8 as clock signal (84 MHz / 8 = 10.5 MHz)
                                | TC_CMR_WAVE               // Set operating mode to Waveform (to generate PWM and set TIOA as an output)
                                | TC_CMR_WAVSEL_UP_RC       // Set TC to increment internal counter (TC_CV) from 0 to RC and automatically trigger to reset on RC compare
                                | TC_CMR_ACPA_CLEAR         // Clear TIOA2 on RA compare match (probably to clear voltage..? not sure)
                                | TC_CMR_ACPC_SET;          // Set TIOA2 on RC compare match

    // Set only RC, setting RA (controls duty cycle) is not needed since DAC doesn't require a duty cycle in
    // external trigger mode, since the DACC waits for a rising edge on the selected trigger to begin conversions
    // and duty cycle is not an important factor in single-edged triggers (actually, maybe set the duty cycle to give it some time to do the conversion? No.)
    // Set compare register C t, RC compare events will trigger the DACC interrupt
    TC0->TC_CHANNEL[2].TC_RC = VARIANT_MCK / 2 / SAMPLING_RATE;  // TC_RC = MCK / 8 / Frequency,
    TC0->TC_CHANNEL[2].TC_RA = TC0->TC_CHANNEL[2].TC_RC / 2;     // 50% duty cycle

    // TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
    // NVIC_EnableIRQ(TC2_IRQn);

    TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;  // Enable TC2 clock
    // TC0->TC_BCR = TC_BCR_SYNC;                                // Synchronize the clock of all enabled channels on timer counter 0 (TC1 and TC2 in our case),
                                                              // this allows both the ADC and DAC to simultaneously trigger at same sampling rate and clock
}

void setupDACC() {
    pmc_enable_periph_clk(ID_DACC);  // Enables and powers on the DAC Controller's (DACC) clock (all peripheral clocks are initially off to save power)

    DACC->DACC_CR = DACC_CR_SWRST;           // Reset DACC
    DACC->DACC_MR = DACC_MR_TRGEN_EN         // DACC_MR_TRGEN_DIS  // Disable external trigger mode to put DAC in free running mode
                    | DACC_MR_TRGSEL(0b011)  // Trigger by TIOA2
                    | DACC_MR_WORD_HALF      // Set half-word transfer mode (DACC_CDR[11:0])
                    // | DACC_MR_WORD_WORD
                    | DACC_MR_REFRESH(0)  // Set refresh period to 0 to disable DACC channel refresh function
                    // | DACC_MR_TAG_EN      // Enable tag mode, which allows for channel selection using DACC_CDR[15:12]
                    | DACC_MR_USER_SEL_CHANNEL1  // temp
                    | DACC_MR_STARTUP_8          // Startup takes 8 periods of DACC clock
                    | DACC_MR_MAXS_MAXIMUM;      // Enable max speed mode, which saves two DACC clock periods

    // Set analog current of DAC, which allows the DAC to adapt current output to the slew
    // rate of analog output on each channel and adapt performance based on power consumption
    // (scales better at higher frequencies)
    // Values from datasheet, would also include DACC_ACR_IBCTLCH0(0b10) but my DAC0 is not working
    DACC->DACC_ACR = DACC_ACR_IBCTLCH1(0b10) | DACC_ACR_IBCTLDACCORE(0b01);

    // Enable DAC channel(s) and IRQ
    NVIC_EnableIRQ(DACC_IRQn);        // might need this cuz see dma comments lmao, enables DACC_Handler to be used
    DACC->DACC_CHER = DACC_CHER_CH1;  // DACC_CHER_CH0 | DACC_CHER_CH1;

    // DAC PDC DMA setup
#if USE_DMA
    DACC->DACC_IER = DACC_IER_TXBUFE;  // Enable interrupt to trigger when DMA transmit buffer is empty
    DACC->DACC_TPR = *dds_lut;
    DACC->DACC_TCR = dds_size;
    DACC->DACC_TNPR = (uint32_t)dds_lut;
    DACC->DACC_TNCR = dds_size;
    DACC->DACC_PTCR = DACC_PTCR_TXTEN;  
#else
    DACC->DACC_IER = DACC_IER_TXRDY;
#endif  // USE_DMA
}

void setupTCADC() {
    pmc_enable_periph_clk(ID_TC1);

    // Same settings as the DAC timer counter, just on a different channel and in DAC timer, we sync them both
    TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // Use MCK / 2 as clock signal (84 MHz / 2 = 42 MHz)
                                | TC_CMR_WAVE               // Set operating mode to Waveform (to generate PWM and set TIOA as an output)
                                | TC_CMR_WAVSEL_UP_RC       // Set TC to increment internal counter (TC_CV) from 0 to RC and automatically trigger to reset on RC compare
                                | TC_CMR_ACPA_CLEAR         // Clear TIOA2 on RA compare match (probably to clear voltage..? not sure)
                                | TC_CMR_ACPC_SET;          // Set TIOA2 on RC compare match

    TC0->TC_CHANNEL[1].TC_RC = VARIANT_MCK / 2 / SAMPLING_RATE;
    TC0->TC_CHANNEL[1].TC_RA = TC0->TC_CHANNEL[1].TC_RC / 2;

    TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;  // Enable channel 1 of timer counter 0 (TC1)
}

void setupADCC() {
    pmc_enable_periph_clk(ID_ADC);

    ADC->ADC_CR = ADC_CR_SWRST;
    ADC->ADC_MR = ADC_MR_TRGEN_EN            // Enable hardware trigger by one of the TIOA outputs of the internal timer counter channels
                  | ADC_MR_TRGSEL_ADC_TRIG2  // Enable TIOA1 (timer counter 0 channel 1) to trigger adc conversion interrupts on rising edge
                  | ADC_MR_PRESCAL(0);

    ADC->ADC_ACR = ADC_ACR_IBCTL(0b01);  // Must use IBCTL = 0b01 for sampling frequency between 500 KHz and 1 MHz according to datasheet

    ADC->ADC_IER = ADC_IER_ENDRX;

    NVIC_EnableIRQ(ADC_IRQn);
    ADC->ADC_CHER = ADC_CHER_CH7;  // Channel 7 = A0

    ADC->ADC_RPR = (uint32_t)adc_buffer[0];
    ADC->ADC_RCR = 512;
    ADC->ADC_RNPR = (uint32_t)adc_buffer[1];
    ADC->ADC_RNCR = 512;
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;

    ADC->ADC_CR = ADC_CR_START;
}

void setup() {
    Serial.begin(9600);
    tableWidth = log2(LOOKUP_SIZE);
    outputFrequency = 50000;

    // Generate sine lookup table for debugging
    for (int i = 0; i < LOOKUP_SIZE; i++) {
        sine_lut[i] = (uint16_t)roundf(2047.0 * (sin(2.0 * PI * (float)i / LOOKUP_SIZE) + 1.0));
        // (uint16_t)roundf(2047.0 + 2047.0*sin(2*PI * (float)i / LOOKUP_SIZE) + 0.5);
    }

    adcBuffer = circular_buffer_init(512 * 4);

    Serial.println(USE_DMA);
#if USE_DMA
    generateDDSLookup();
#else
    tuningWord = pow(2, 32) * outputFrequency / SAMPLING_RATE;  // Fout = (tuningWord * Fs) / 2^32
#endif  // USE_DMA

    Serial.println(tuningWord);
    Serial.println(dds_size);

    pmc_set_writeprotect(false);  // Sets PMC_WPMR to disable write protection of registers
    setupDACC();
    // setupADCC();
    // setupTCADC();
    setupTCDAC();

    pinMode(RE_SW, INPUT_PULLUP);
    pinMode(RE_DT, INPUT_PULLUP);
    pinMode(RE_CLK, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RE_CLK), changeFrequency, FALLING);
    attachInterrupt(digitalPinToInterrupt(RE_DT), changeFrequency, FALLING);

    //     if (outputFrequency < 1000) { // Disable DMA
    //     DACC->DACC_IER = DACC_IER_TXRDY;
    // }
}

// double buffer needed
void changeFrequency() {
    if (!wasHeldDown) {
        // According to datasheet, CLK is pin A and DT is pin B
        int encoderA = digitalRead(RE_CLK);
        int encoderB = digitalRead(RE_DT);

        // handle quadrature signal, mention falling cuz pllup
        // There are two switches in a rotary encoder, with one switch connecting pin A to pin C
        // and the other connecting pin B to pin C. If both switches are closed, then turning the
        // encoder clockwise or counterclockwise one position open both switches, and if both
        // switches are open then turning the encoder one position will close both switches.
        // This open and close behaviour of the switches at every position change is what generates
        // the square wave signal from pins A and B that we can use to determine how the encoder rotates.

        // Depending on which switch changes state first, we can determine the direction of the turn. If A
        // changed states first, then the switch is being rotated clockwise and counterclockwise if B changes
        // state first. Using pull-up resistors to pull both pins HIGH, we can use a interrupt to listen to the
        // falling edge of both pins

        // Ignore if equal states because it is not a real value to consider
        // Since one switch will always change state before the other, two signals are generated each
        // time the encoder is turned. These two signals are out of phase with each other and we only
        // need to consider the first signal as that first signal represents the direction the encoder
        // was turned, while the second signal can be ignored since it is a delayed
        // Due to their posi
        // since they are always out of phase with each other so
        // the interrupt will always fire
        // If they are equal, that is because the interrupt fires twice each rotation since I have
        // both pin A and pin B connected to the same interrupt, so either pin will always fire
        // the interrupt again after the first pin already fired it due to the phase difference
        if (encoderA != encoderB) {
            // We're using a pull-up resistor because
            if (encoderA == LOW && outputFrequency + frequencyStep <= SAMPLING_RATE / 2) {
                outputFrequency += frequencyStep;
            } else if (encoderB == LOW && outputFrequency - frequencyStep > 0) {
                outputFrequency -= frequencyStep;
            }

            if (outputFrequency < 1000) {}
        }
    }
}

void loop() {
    int encoderButton = digitalRead(RE_SW);
    if ((millis() - lastDebounce) > inputDebounceTime) {
        if (encoderButton == LOW && lastButtonState == HIGH) {  // Falling
            holdDownStartTime = millis();
            wasHeldDown = false;
        } else if (!wasHeldDown && encoderButton == HIGH && lastButtonState == LOW) {  // Rising
            holdDownStartTime = 0;
            generateDDSLookup();
        }

        if (wasHeldDown) {
            int encoderA = digitalRead(RE_CLK);
            int encoderB = digitalRead(RE_DT);
            if (encoderA != encoderB) {
                int step = pow(10, floor(log10(frequencyStep)));
                if (frequencyStep + step > 100000 || (frequencyStep - step <= 0 && encoderB == LOW)) {
                    step--;
                }
                frequencyStep += encoderA == LOW ? step : -step;
            }
        }
        lastDebounce = millis();
    }

    // Rising after holding button down, moved out of other if because of debounce causing issues with holding down
    if (encoderButton == HIGH && lastButtonState == LOW && (millis() - holdDownStartTime) > buttonHoldTimeThreshold) {
        holdDownStartTime = 0;
        wasHeldDown = true;
    }
    lastButtonState = encoderButton;

    // Serial.print(outputFrequency);
    // Serial.print(" ");
    // Serial.println(frequencyStep);

    if (!circular_buffer_isEmpty(adcBuffer)) {
        Serial.println(circular_buffer_dequeue(adcBuffer));
    }
}

void DACC_Handler() {
    // 896
    // Need to check interrupt status to see if end of conversion flag is set
    // and if the transmit ready flag is set, however, transmit ready flag will
    // always be set if the DAC is ready to accept another conversion so we
    // can simplify and just check for transmit ready (TXRDY)
#if !(USE_DMA)
    if (DACC->DACC_ISR & DACC_ISR_TXRDY) {
        uint32_t phaseIncrement = phaseAccumulator >> (32 - tableWidth);
        // phaseIncrement = phaseAccumulator >> (32 - tableWidth); // convert phase to lookup table index
        DACC->DACC_CDR = sine_lut[phaseIncrement];  // (1 << 28 | data << 16) | (1 << 12 | data); // get digital amplitude corresponding to phase
        phaseAccumulator += tuningWord;             // calculate next phase shift
    }
#endif

    // Check transmit buffer empty flag, if true then TCR (transmit counter register) and TCNR (transmit next counter register)
    // have both reached zero, which indicates there is no remaining data in DMA buffer and the last data was transferred to peripheral memory (DAC)
#if USE_DMA
    if (DACC->DACC_ISR & DACC_ISR_TXBUFE) {  // overflow &&
        DACC->DACC_TNPR = (uint32_t)dds_lut;
        DACC->DACC_TNCR = dds_size;  // numSamples;
    }
#endif
}

// void ADC_Handler() {
//     if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
//         ADC->ADC_RNPR = (uint32_t)adc_buffer[swapReadWriteBuffer]; // false = 0, true = 1
//         ADC->ADC_RNCR = 512;
//         swapReadWriteBuffer = !swapReadWriteBuffer; // swap previous buffer to a read and current buffer to write
//         // swap occurs after first rnpr rather than before because we want to start at 0

//         // toggle read buffer flag to have previous buffer read
//         // write previous buffer to larger circular buffer for it to be eventually sent over spi to lcd
//         circular_buffer_enqueue(adcBuffer, adc_buffer[swapReadWriteBuffer], 512);
//     }
// }