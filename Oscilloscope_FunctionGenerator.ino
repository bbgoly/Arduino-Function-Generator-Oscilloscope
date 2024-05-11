/* 
 * EECS 2032 Project Oscilloscope & Function Generator
 * By Yousif Kndkji
 *
 * This project makes use of three public libraries each under a MIT or BSD license, such as the SdFat library by Bill Greiman 
 * (https://github.com/greiman/SdFat-beta), the Adafruit graphics libraries for the ILI9341, and the XPT2046_Touchscreen library by
 * Paul Stoffregen (https://github.com/PaulStoffregen/XPT2046_Touchscreen). The rest of the code is my own work.
 *
 * The SdFat library used is the same library used in Arduino's standard SD library, but a newer version of it to support
 * exFAT MicroSDXC cards (which is what I am using).
 *
 * As for the Adafruit graphics libraries, they are used to run the ILI9341 driver on the TFT LCD I have, and the
 * XPT2046_Touchscreen library is used to interface with the XPT2046 touchscreen IC that comes with the TFT LCD I am using.

 * Uses SdFat library by Bill Greiman (https://github.com/greiman/SdFat-beta), a newer version of the same library used in 
 * Arduino's standard SD library to support exFAT MicroSDXC cards.
 *
 * Also uses Adafruit graphics libraries to run the ILI9341 driver on the TFT LCD I have.
 * I also use the XPT2046_Touchscreen library by Paul Stoffregen (https://github.com/PaulStoffregen/XPT2046_Touchscreen)
 * to communicate with the XPT2046 touchscreen IC that comes with the ILI9341 TFT LCD I have.
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include <XPT2046_Touchscreen.h>
#include "circular_buffer.h"
#include "ILI9341_UI.h"

#define TFT_CS 28
#define TFT_DC 26
#define T_CS 40
#define SD_CS 48

#define FREQ_RE_SW 4
#define FREQ_RE_DT 5
#define FREQ_RE_CLK 6

#define AMP_RE_SW 7
#define AMP_RE_DT 8
#define AMP_RE_CLK 9

#define WAVE_RE_SW 10
#define WAVE_RE_DT 11
#define WAVE_RE_CLK 12

constexpr float SAMPLING_RATE = 1000000.0;
constexpr uint16_t LOOKUP_SIZE = 8192;  // 2^14 8192 16384
constexpr uint16_t ADC_BUFFER_SIZE = 512;

// The range of voltages that the DAC on the Arduino Due can output according to datasheet
constexpr float MAX_DAC_VOLTAGE = 2.75;
constexpr float MIN_DAC_VOLTAGE = 0.55;

// Input voltage range of ADC on the Arduino Due according to the datasheet
constexpr float ADC_VOLTAGE_RANGE = 3.3;

// Function generator variables
uint16_t waveformLookupTables[2][LOOKUP_SIZE];

float signalPeakVoltage;
float outputFrequency;
int waveformSelect;

uint32_t tuningWord;
uint32_t phaseAccumulator;
uint8_t lookupWidth;

uint16_t ddsLookupSize;
uint16_t ddsLookup[LOOKUP_SIZE];

// Function generator related rotary encoder variables
float frequencyStep = 100;
int freqLastButtonState = HIGH;
int amplitudeLastButtonState = HIGH;
int waveformLastButtonState = HIGH;

// Oscilloscope variables
uint16_t adcPingPongBuffer[2][ADC_BUFFER_SIZE];  // Ping pong double buffer
volatile bool swapReadWriteBuffer = false;

circular_buffer_t* circularADCBuffer;

bool oscilloscopeEnabled = true;
bool oscilloscopeCH1Enabled = true;  // convert to array of enabled channels to feed to adc config in future to expand number of channels
bool oscilloscopeCH2Enabled = false;

// TFT LCD and touchscreen variables
Adafruit_ILI9341 lcd(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(T_CS);

UIPage UIPages[2];
int currentDisplayedPage = 0;

int touchDebounceTime = 300;
uint32_t lastTouchTime = 0;

// General rotary encoder variables
bool wasHeldDown = false;
int encoderDebounceTime = 50;
int buttonDebounceTime = 300;
int buttonHoldTimeThreshold = 500;
uint32_t lastEncoderDebounce = 0;
uint32_t holdDownStartTime = 0;

void generateDDSLookup() {
    // waveformSelect = 0 => sine wave
    // waveformSelect = 1 => triangle wave
    // waveformSelect = 2 => square wave
    float voltageRatio = signalPeakVoltage / MAX_DAC_VOLTAGE;
    
    // if waveformSelect == 2, do something else since thats square wave
    if (waveformSelect == 2) {
        TC0->TC_CHANNEL[2].TC_RC = VARIANT_MCK / 2 / outputFrequency;
        ddsLookupSize = 2;
        ddsLookup[0] = voltageRatio * 0xFFF;
        ddsLookup[1] = 0;
    } else {
        tuningWord = pow(2, 32) * outputFrequency / SAMPLING_RATE;
        ddsLookupSize = SAMPLING_RATE / outputFrequency;  // size is number of times tuning word can go into 32 bits, so size = 2^32 / tuningWord = Fs / Fout
        for (int i = 0; i < ddsLookupSize; i++) {
            uint32_t phaseIncrement = phaseAccumulator >> (32 - lookupWidth);
            ddsLookup[i] = voltageRatio * waveformLookupTables[waveformSelect][phaseIncrement];
            phaseAccumulator += tuningWord;
        }
    }
}

void setupTCDACC() {
    // To supply the clock of the DACC and trigger it externally, we can generate PWM pulses through TIOA2
    // (Timer Channel 2 on I/O Line A) using an internal clock. According to the block diagram in the datasheet,
    // to generate PWM pulses through TIOA2 we would need to enable and power on Timer Counter 0 (TC0) Channel 2
    // (which is TC2) in the power management controller.
    pmc_enable_periph_clk(ID_TC2);

    // Set channel to operate on Waveform mode, which configures TIOA to be an output (this allows us to
    // generate PWM pulses to supply the DACC with a clock)
    TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // Use MCK / 2 as clock signal (84 MHz / 2 = 42 MHz)
                                | TC_CMR_WAVE               // Set channel operating mode to Waveform (to generate PWM and set TIOA as an output)
                                | TC_CMR_WAVSEL_UP_RC       // Set TC to increment an internal counter (TC_CV) from 0 to RC and automatically generate
                                                            // a software trigger on RC compare and reset the counter back to 0
                                | TC_CMR_ACPA_CLEAR         // Clear TIOA2 on RA compare to effectively create a duty cycle
                                | TC_CMR_ACPC_SET;          // Set TIOA2 on RC compare

    // Set compare register C to 42 MHz divided by the sampling rate, meaning a software trigger would generate
    // SAMPLING_RATE times. e.g. SAMPLING_RATE = 1 MHz, therefore RC = 42, so the counter would increment up to
    // 42 1 million times, generating a constant 1 MHz clock to trigger our DAC conversions on each rising edge.
    TC0->TC_CHANNEL[2].TC_RC = VARIANT_MCK / 2 / SAMPLING_RATE;  // TC_RC = MCK / 2 / Frequency = 84 MHz / 2 / 1 MHz = 42
    TC0->TC_CHANNEL[2].TC_RA = TC0->TC_CHANNEL[2].TC_RC / 2;     // 50% duty cycle

    TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKEN;  // Enable clock of channel 2 of timer counter 0 (TC2) in the channel control register
    TC0->TC_BCR = TC_BCR_SYNC;                 // Enables sync mode in the block control register to synchronize the clock of all enabled
                                               // channels on timer counter 0 (TC1 and TC2 in our case) by simultaneously generating a
                                               // software trigger across each channel on RC compare, which allows both the ADC and DAC to
                                               // simultaneously trigger on the same rising edges
}

void setupDACC() {
    // Enables and powers on the DAC Controller's (DACC) clock (all peripheral clocks are initially off to save power)
    pmc_enable_periph_clk(ID_DACC);

    // I had wished to make the function generator be able to operate on both DAC channels, but unfortunately my DAC0 seems to
    // have suddenly stopped working. The code to enable the dual-channel configuration is available below and would have been
    // implemented had my DAC0 been working. However, because it isn't working, I am only able to use DAC1.

    DACC->DACC_CR = DACC_CR_SWRST;               // Reset DACC
    DACC->DACC_MR = DACC_MR_TRGEN_EN             // Enable external trigger mode to have DAC conversions happen on a software trigger from TC2 over TIOA2
                    | DACC_MR_TRGSEL(0b011)      // Trigger conversions by TIOA2, value obtained from datasheet
                    | DACC_MR_WORD_HALF          // Set DAC to operate in half-word transfer mode (12 bit resolution, remaining MSB's are used for channel
                                                 // selection if tag mode was enabled, though, again, DAC0 isn't working so tag mode isn't needed).
                                                 // Otherwise, DACC_MR_WORD_WORD would be set instead of DACC_MR_WORD_HALF to simultaneously trigger
                                                 // conversions on one or both DAC channels.
                    | DACC_MR_REFRESH(0)         // Set refresh period to 0 to disable DACC channel refresh function
                                                 // | DACC_MR_TAG_EN Enables tag mode, which allows for flexible channel selection using bits [13:12] and [29:28]
                    | DACC_MR_USER_SEL_CHANNEL1  // Instead of using tag mode, I have to use this to enable only channel 1 (DAC1) due to DAC0 not working
                    | DACC_MR_STARTUP_8          // Set DACC startup to take 8 periods of DACC clock
                    | DACC_MR_MAXS_MAXIMUM;      // Enable max speed mode, which saves two DACC clock periods on each conversion (25 -> 23)

    // Set analog current of DAC, which allows the DAC to adapt current output to the slew
    // rate of analog output on each enabled channel and adapt performance based on power consumption
    // (needed for sampling rate higher than 500 KHz). Values obtained from datasheet, I would also
    // normally also set DACC_ACR_IBCTLCH0(0b10) but my DAC0 is not working so no need to configure channel 0
    DACC->DACC_ACR = DACC_ACR_IBCTLCH1(0b10) | DACC_ACR_IBCTLDACCORE(0b01);  // For dual-channel, set DACC_ACR_IBCTLCH0(0b10) as well

    // Enable DAC channel(s) and IRQ
    NVIC_EnableIRQ(DACC_IRQn);        // Enable DACC interrupt in nested vector interrupt controller
    DACC->DACC_CHER = DACC_CHER_CH1;  // For dual-channel, use DACC_CHER_CH0 | DACC_CHER_CH1 instead

    // DAC PDC DMA setup
    DACC->DACC_IER = DACC_IER_TXBUFE;       // Set DACC interrupt to trigger when DMA transmit buffer is empty
    DACC->DACC_TPR = (uint32_t)ddsLookup;   // Set pointer to first transmit buffer
    DACC->DACC_TCR = ddsLookupSize;         // Set size of transmit buffer
    DACC->DACC_TNPR = (uint32_t)ddsLookup;  // Set pointer to second transmit buffer
    DACC->DACC_TNCR = ddsLookupSize;        // Set size of next transmit buffer
    DACC->DACC_PTCR = DACC_PTCR_TXTEN;      // Enables half-duplex PDC transmit channel requests for DACC
}

void setupTCADC() {
    pmc_enable_periph_clk(ID_TC1);

    // Same settings as the DAC timer counter, but on channel 1 of timer counter 0 (which is TC1) and ADC software
    // triggers are generated through TIOA1
    TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1
                                | TC_CMR_WAVE
                                | TC_CMR_WAVSEL_UP_RC
                                | TC_CMR_ACPA_CLEAR
                                | TC_CMR_ACPC_SET;

    TC0->TC_CHANNEL[1].TC_RC = VARIANT_MCK / 2 / SAMPLING_RATE;
    TC0->TC_CHANNEL[1].TC_RA = TC0->TC_CHANNEL[1].TC_RC / 2;

    TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;  // Enable channel 1 of timer counter 0 (TC1)
}

void setupADCC() {
    pmc_enable_periph_clk(ID_ADC);

    ADC->ADC_CR = ADC_CR_SWRST;              // Reset ADCC
    ADC->ADC_MR = ADC_MR_TRGEN_EN            // Enable hardware trigger by one of the TIOA outputs of the internal timer counter channels
                  | ADC_MR_TRGSEL_ADC_TRIG2  // Select TIOA1 (timer counter 0 channel 1) to trigger ADC conversions on rising edge
                  | ADC_12_BITS              // Set ADC resolution to 12 bits
                  | ADC_MR_FWUP_ON           // Keep ADC core off between conversions for power saving when ADC is not performing conversions, but keep voltage reference on for fast wakeup
                  | ADC_MR_PRESCAL(0);       // Set ADC prescaler to 0, which sets ADC clock to 42 MHz (ADCClock = MCK / (PRESCAL + 1)*2) from datasheet, where MCK is 84 MHz)

    ADC->ADC_ACR = ADC_ACR_IBCTL(0b01);  // Must use IBCTL = 0b01 for sampling frequency between 500 KHz and 1 MHz according to datasheet

    // Configure ADC channels and interrupts, and enable ADC IRQ
    // ADC->ADC_CHDR = ~ADC_CHDR_CH7;  // Disable all channels but channel 7
    // ADC->ADC_IDR = ~ADC_IDR_ENDRX;  // Disable all interrupts but ENDRX
    ADC->ADC_CHER = ADC_CHER_CH7;// | ADC_CHER_CH6;  // Enable channel 7, which corresponds to A0 on the Arduino
    ADC->ADC_IER = ADC_IER_ENDRX;                 // Enable ADC interrupt to trigger when ENDRX flag is set
                                                  // ENDRX flag is set when ADC_RCR reaches zero (i.e., when DMA receive buffer is full)
    ADC->ADC_EMR = ADC_EMR_TAG;                   // Since we are using the PDC receive channel, which triggers each time a new conversion
                                                  // is stored in ADC_LDCR, and we are looking to make a multi-channel oscilloscope, we must
                                                  // enable tag mode to have the 4 MSB of the half-word data in ADC_LDCR carry the channel number
                                                  // associated with that conversion to be able to determine where the data came from later on

    NVIC_EnableIRQ(ADC_IRQn);       // Enable ADCC interrupt to trigger when DMA receive buffer is full
    NVIC_SetPriority(ADC_IRQn, 1);  // Set priority of ADCC interrupt to be lower than the DACC interrupt to solve jitter
                                    // appearing on DAC signal generation due to microcontroller being busy with processing
                                    // ADC interrupt and other internal interrupts.

    // ADC PDC DMA setup

    // For the PDC DMA of the ADC, I chose to use a ping pong double buffer to allow the PDC DMA to read ADC
    // conversions and write them in one buffer, and have my code asynchronously read the other buffer that the
    // PDC is not writing to and move those values into a larger circular buffer in order to free up that buffer
    // before the PDC DMA finishes writing to the other buffer. Each time the PDC DMA finishes writing to a buffer,
    // the buffer that was being written into switches into the read buffer and the buffer that was being read and
    // copied into the circular buffer now becomes the new buffer that the PDC DMA stores newly received ADC conversion
    // values into. This creates the ping-pong nature of this double buffer setup, and theoretically allows for no loss of
    // ADC conversions, while not blocking the ADC interrupt at the same time unlike in a typical setup where data would
    // be processed in the interrupt immediately after it is received from the ADC. Instead, the Arduino can store all the
    // received data into a larger circular buffer, and read those values in the main Arduino loop and process the data one
    // batch at a time at its own leisure, without worrying about affecting the performance or accuracy of the ADC whatsoever.
    // This is also possible because using PDC DMA also heavily frees up the processor by directly accessing peripheral
    // ADC/DAC memory to receive/transmit conversions, which the processor would otherwise have to handle, allowing the
    // processor more cycles to process other interrupts and the Arduino loop instead.

    ADC->ADC_RPR = (uint32_t)adcPingPongBuffer[0];   // Set first pointer to first receive (write) buffer in ping pong buffer
    ADC->ADC_RCR = ADC_BUFFER_SIZE;                  // Set size of receive buffer
    ADC->ADC_RNPR = (uint32_t)adcPingPongBuffer[1];  // First pointer is now full and becomes a read buffer, so set second pointer to other receive buffer
    ADC->ADC_RNCR = ADC_BUFFER_SIZE;                 // Set size of next receive buffer
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;                  // Enables half-duplex PDC receive channel requests for ADCC
}

// Text button callbacks, when text button is touched on LCD, one of these callback functions are invoked
// Callbacks for oscilloscope UI
void switchUIPage(int sourceButtonIndex) {
    // UIPages[currentDisplayedPage].unrenderAll(lcd, ILI9341_BLACK);
    // currentDisplayedPage = (currentDisplayedPage + 1) & 1;
    // UIPages[currentDisplayedPage].renderAll(lcd);
}

void toggleOscilloscope(int sourceButtonIndex) {
    TextButton* toggleADCButton = UIPages[currentDisplayedPage].getTextButton(sourceButtonIndex);
    toggleADCButton->unrender(lcd, ILI9341_BLACK);
    if (oscilloscopeEnabled) {
        // Disable oscilloscope
        Serial.println("hello");
        Serial.println(ADC->ADC_RCR);
        Serial.println(ADC->ADC_RNCR);

        ADC->ADC_CHDR = 0xFFFF;
        ADC->ADC_IDR = ~(0ul);

        ADC->ADC_PTCR = ADC_PTCR_RXTDIS;

        toggleADCButton->setFrameColor(ILI9341_RED);
        toggleADCButton->setBorderColor(ILI9341_UI_DARKRED);
        toggleADCButton->setText(lcd, "Disabled");
        oscilloscopeEnabled = false;
    } else {
        // Enable oscillscope
        Serial.println(ADC->ADC_RCR);
        Serial.println(ADC->ADC_RNCR);
        // Re-enable all channels and interrupts that were previously enabled
        ADC->ADC_CHDR = ~(oscilloscopeCH1Enabled << 7 | oscilloscopeCH2Enabled << 6);
        ADC->ADC_IDR = ~ADC_IDR_ENDRX;

        ADC->ADC_CHER = oscilloscopeCH1Enabled << 7 | oscilloscopeCH2Enabled << 6;
        ADC->ADC_IER = ADC_IER_ENDRX;

        ADC->ADC_PTCR = ADC_PTCR_RXTEN;

        toggleADCButton->setFrameColor(ILI9341_GREEN);
        toggleADCButton->setBorderColor(ILI9341_UI_DARKGREEN);
        toggleADCButton->setText(lcd, "Enabled");
        oscilloscopeEnabled = true;
    }
    toggleADCButton->render(lcd);
}

void toggleADCChannel1(int sourceButtonIndex) {
    TextButton* toggleADCCH1 = UIPages[currentDisplayedPage].getTextButton(sourceButtonIndex);
    toggleADCCH1->unrender(lcd, ILI9341_BLACK);
    if (ADC->ADC_CHSR & ADC_CHSR_CH7) {
        toggleADCCH1->setFrameColor(ILI9341_RED);
        toggleADCCH1->setBorderColor(ILI9341_UI_DARKRED);
        toggleADCCH1->setText(lcd, "CH1: Disabled");
        ADC->ADC_CHDR = ~(oscilloscopeCH2Enabled << 6);
        oscilloscopeCH1Enabled = false;
    } else {
        toggleADCCH1->setFrameColor(ILI9341_GREEN);
        toggleADCCH1->setBorderColor(ILI9341_UI_DARKGREEN);
        toggleADCCH1->setText(lcd, "CH1: Enabled");
        ADC->ADC_CHER = ADC_CHER_CH7 | (oscilloscopeCH2Enabled << 6);
        oscilloscopeCH1Enabled = true;
    }
    toggleADCCH1->render(lcd);
}

void toggleADCChannel2(int sourceButtonIndex) {
    TextButton* toggleADCCH2 = UIPages[currentDisplayedPage].getTextButton(sourceButtonIndex);
    toggleADCCH2->unrender(lcd, ILI9341_BLACK);
    if (ADC->ADC_CHSR & ADC_CHSR_CH6) {
        toggleADCCH2->setFrameColor(ILI9341_RED);
        toggleADCCH2->setBorderColor(ILI9341_UI_DARKRED);
        toggleADCCH2->setText(lcd, "CH2: Disabled");
        ADC->ADC_CHDR = ~(oscilloscopeCH1Enabled << 7);
        oscilloscopeCH2Enabled = false;
    } else {
        toggleADCCH2->setFrameColor(ILI9341_GREEN);
        toggleADCCH2->setBorderColor(ILI9341_UI_DARKGREEN);
        toggleADCCH2->setText(lcd, "CH2: Enabled");
        ADC->ADC_CHER = ADC_CHER_CH6 | (oscilloscopeCH1Enabled << 7);
        oscilloscopeCH2Enabled = true;
    }
    toggleADCCH2->render(lcd);
}

// Callbacks for function generator UI

void initLCD() {
    // Configure the TFT LCD and setup hardware SPI connection
    lcd.begin();
    lcd.setRotation(3);
    lcd.fillScreen(ILI9341_BLACK);

    lcd.setTextSize(1);

    // Configure the XPT2046 and setup hardware SPI connection
    ts.begin();
    ts.setRotation(3);

    // Setup oscilloscope UI page
    Frame* frames = new Frame[1]{
        Frame(0, 25 + 6 + 5, lcd.width(), lcd.height() - (25 + 6 + 5) - 20 - 11, ILI9341_BLACK, 1, ILI9341_DARKGREY)
    };

    // char formattedStatus[50];
    // sprintf(formattedStatus, "Status: %.1f MHz @ %d samples", SAMPLING_RATE / 1.0e6, ADC_BUFFER_SIZE);
    // Serial.println(formattedStatus);
    TextField* textFields = new TextField[3]{
        TextField(0, 0, 210, 25, "formattedStatus", ILI9341_BLUE, 15, CircularBorderType::Right, ILI9341_WHITE, TextAlignment::LeftAlign, 3, ILI9341_UI_DARKBLUE),
        TextField(0, lcd.height() - 20, 47, 20, "0.00 V", ILI9341_BLUE, ILI9341_WHITE, TextAlignment::LeftAlign, 3, ILI9341_UI_DARKBLUE),
        TextField(47, lcd.height() - 20, 80, 20, "0.0 Hz", ILI9341_BLUE, 10, CircularBorderType::Right, ILI9341_WHITE, TextAlignment::LeftAlign, 3, ILI9341_UI_DARKBLUE)
    };

    TextButton* textButtons = new TextButton[4]{
        TextButton(280, 0, 40, 25, "Switch", ILI9341_GREEN, switchUIPage, 10, CircularBorderType::Bottom, ILI9341_WHITE, TextAlignment::CenterAlign, 3, ILI9341_UI_DARKGREEN),
        TextButton(220 - 6, 0, 60, 25, "Enabled", ILI9341_GREEN, toggleOscilloscope, 10, CircularBorderType::Bottom, ILI9341_WHITE, TextAlignment::CenterAlign, 3, ILI9341_UI_DARKGREEN),
        TextButton(lcd.width() - 180 - 3, lcd.height() - 20, 90, 20, "CH1: Enabled", ILI9341_GREEN, toggleADCChannel1, 10, CircularBorderType::Left, ILI9341_WHITE, TextAlignment::CenterAlign, 3, ILI9341_UI_DARKGREEN),
        TextButton(lcd.width() - 90, lcd.height() - 20, 90, 20, "CH2: Disabled", ILI9341_RED, toggleADCChannel2, ILI9341_WHITE, TextAlignment::CenterAlign, 3, ILI9341_UI_DARKRED),
    };

    UIPages[0].setElements(frames, textFields, textButtons, 1, 3, 4);
    UIPages[0].renderAll(lcd);
}

void setup() {
    Serial.begin(9600);

    pinMode(FREQ_RE_SW, INPUT_PULLUP);
    pinMode(FREQ_RE_DT, INPUT_PULLUP);
    pinMode(FREQ_RE_CLK, INPUT_PULLUP);

    pinMode(AMP_RE_SW, INPUT_PULLUP);
    pinMode(AMP_RE_DT, INPUT_PULLUP);
    pinMode(AMP_RE_CLK, INPUT_PULLUP);

    // pinMode(WAVE_RE_SW, INPUT_PULLUP);
    // pinMode(WAVE_RE_DT, INPUT_PULLUP);
    // pinMode(WAVE_RE_CLK, INPUT_PULLUP);

    // Instantiate circular buffer and set size to a power of 2, through testing
    // different frequencies, the buffer seems to reach less than 2^13 entries
    circularADCBuffer = circular_buffer_init(pow(2, 13));

    lookupWidth = log2(LOOKUP_SIZE);

    signalPeakVoltage = MAX_DAC_VOLTAGE;
    outputFrequency = 1000;
    waveformSelect = 2;

    // Generate sine wave lookup table
    for (int i = 0; i < LOOKUP_SIZE; i++) {
        waveformLookupTables[0][i] = (uint16_t)roundf(2047.0 * (sin(2.0 * PI * (float)i / LOOKUP_SIZE) + 1.0));
    }

    // Generate triangle wave lookup table
    // ...
    generateDDSLookup();

    Serial.print("\nTuning word: ");
    Serial.println(tuningWord);
    Serial.print("DDS lookup table size: ");
    Serial.println(ddsLookupSize);

    initLCD();

    // Disable register write protection in the power management controller
    pmc_set_writeprotect(false);

    // Setup ADC controller registers and the timer counter that will trigger ADC conversions
    setupADCC();
    setupTCADC();
    Serial.println(ADC->ADC_CHER, BIN);

    // Setup DAC controller registers and the timer counter that will trigger DAC conversions
    setupDACC();
    setupTCDACC();

    lastTouchTime = millis();
    lastEncoderDebounce = millis();
}

uint32_t timeElapsed = 0;

void loop() {
    timeElapsed = micros();
    if (circular_buffer_isFull(circularADCBuffer)) {  // Debugging
        TextField* statusField = UIPages[0].getTextField(0);
        statusField->setText(lcd, "ADC CIRCULAR FIFO CAPACITY HIT", ILI9341_RED);
    }

    // If circular buffer is populated with conversion values from ADC, DMA it to SD card over SPI
    // to be used for graphing on TFT LCD and so entire waveform can be stored for user's analyzing purposes
    if (!circular_buffer_isEmpty(circularADCBuffer)) {
        uint16_t adcDataBatch[ADC_BUFFER_SIZE];
        circular_buffer_dequeue(circularADCBuffer, adcDataBatch, ADC_BUFFER_SIZE);

        // DMA batch of data from ADC to SD card over SPI

        // uint16_t maxADCValue = 0;
        for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
            // Serial.print("CH ");
            // Serial.print(adcDataBatch[i] >> 12);
            // Serial.print(" Data: ");
            // Uncomment only the next 3 lines to see ADC signal - prints a batch of adc data to view in serial plotter, should display a clean waveform of 512 samples
            // Serial.print("0, ");
            // Serial.print(adcDataBatch[i] & 0xFFF);
            // Serial.println(", 4095");
            char adcData[5];
            snprintf(adcData, 5, "%d", adcDataBatch[i] & 0xFFF);
            printf("%p %p\n", (void*)adcData, (void*)UIPages[0].getTextField(2)->getText());
            UIPages[0].getTextField(2)->setText(lcd, adcData);
            //     // if (adcDataBatch[i] > maxADCValue) {
            //         // maxADCValue = adcDataBatch[i];
            //     // }
        }

        // double freq = 0.0; // freq = 1.0e6 / (double)(timeElapsed - micros());
        // float peakVoltage = maxADCValue * ADC_VOLTAGE_RANGE / pow(2, 12);

        // char peakVoltageText[7], freqText[8];
        // sprintf(peakVoltageText, "%.2f V", peakVoltage);
        // sprintf(freqText, ".1f% %s", freq >= 1000.0 ? freq / 1000.0 : freq, freq >= 1000.0 ? "KHz" : "Hz");

        // UIPages[0].textFields[1].setText(lcd, peakVoltageText);
        // UIPages[0].textFields[2].setText(lcd, freqText);
    }

    // If user touches touchscreen and debounce isn't active, then determine
    // where on the screen the user touched and convert from touchscreen coordinates
    // to ILI9341 screen coordinates and perform a bounds check on each rendered
    // text button to see if point where user touched screen falls within the
    // bounds of any one of the rendered text buttons, and if so, invoke the callback
    // function corresponding to the touched text button(s) and activate debounce
    if (ts.touched() && (millis() - lastTouchTime) > touchDebounceTime) {
        TS_Point touchPoint = convertTSCoords(lcd, ts.getPoint());
        for (int i = 0; i < 4; i++) {
            TextButton* textButton = UIPages[currentDisplayedPage].getTextButton(i);
            if (textButton->isPressed(touchPoint)) {
                textButton->executeCallback(i);
                break;
            }
        }
        lastTouchTime = millis();
    }

    // For each rotary encoder, user must click switch to confirm changes made
    // to corresponding parameters. This is so processor won't have to waste
    // resources on re-computing waveform when switching very quickly through
    // different parameter values as each rotary encoder configures parameters
    // that will result in the waveform being recomputed afterwards.

    // FREQ_RotaryEncoder functionality, handles waveform frequency control on the
    // function generator side and allows the user to control the frequency step
    // value if they hold down the switch for 0.5 seconds or more
    int freqEncoderButtonState = digitalRead(FREQ_RE_SW);
    if (freqEncoderButtonState == HIGH) {  // While switch is not being pressed, reset hold down timer
        holdDownStartTime = millis();
    }

    // We don't want to re-compute waveform table during any switch press while in hold down mode, as output frequency can't change in hold down mode
    if (!wasHeldDown && freqLastButtonState == LOW && freqEncoderButtonState == HIGH && (millis() - lastEncoderDebounce) > buttonDebounceTime) {
        generateDDSLookup();
        Serial.println("reconstructed dds waveform from frequency change");
        lastEncoderDebounce = millis();
    }
    // Serial.print((millis() - holdDownStartTime) > buttonHoldTimeThreshold);
    // Serial.print(" - ");
    // Serial.println(wasHeldDown);
    if (wasHeldDown && freqLastButtonState == HIGH && freqEncoderButtonState == LOW) {
        wasHeldDown = false;             // Press switch again to disable hold down mode
        lastEncoderDebounce = millis();  // re-enables debounce to prevent recomputing waveform after pressing switch to disable hold down mode
    } else if ((millis() - holdDownStartTime) > buttonHoldTimeThreshold) {
        wasHeldDown = true;  // If switch is held down long enough, enable hold down mode
    }
    freqLastButtonState = freqEncoderButtonState;

    int freqEncoderA = digitalRead(FREQ_RE_CLK), freqEncoderB = digitalRead(FREQ_RE_DT);
    if (freqEncoderA != freqEncoderB && (millis() - lastEncoderDebounce) > encoderDebounceTime) {
        if (wasHeldDown && freqEncoderA == LOW) {                       // If in hold down mode, use pin A to increase frequency step
            frequencyStep = pow(10, min(log10(frequencyStep) + 1, 5));  // Set frequency step to the next multiple of 10, up to 100,000
        } else if (wasHeldDown && freqEncoderB == LOW) {                // If in hold down mode, use pin B to decrease frequency step
            frequencyStep = pow(10, log10(frequencyStep) - 1);          // Set frequency step to the previous multiple of 10, down until 1
        } else if (freqEncoderA == LOW) {
            outputFrequency = min(outputFrequency + frequencyStep, SAMPLING_RATE / 2);  // Increment output frequency by frequency step, up to SAMPLING_RATE / 2 (up to 500 KHz)
        } else if (freqEncoderB == LOW && outputFrequency - frequencyStep > 0) {
            outputFrequency -= frequencyStep;  // Decrement output frequency by frequency step, down until 1 Hz
        }

        Serial.print("A: ");
        Serial.print(freqEncoderA);
        Serial.print(" - B: ");
        Serial.print(freqEncoderB);
        Serial.print("\tfreqStep: ");
        Serial.print(frequencyStep);
        Serial.print(" - outputFreq: ");
        Serial.println(outputFrequency);
        lastEncoderDebounce = millis();
    }


    // AMP_RotaryEncoder functionality, controls peak voltage of waveform in steps of 0.05 V (50 mV)
    int amplitudeEncoderButtonState = digitalRead(AMP_RE_SW);
    if (amplitudeLastButtonState == LOW && amplitudeEncoderButtonState == HIGH && (millis() - lastEncoderDebounce) > buttonDebounceTime) {
        generateDDSLookup();
        Serial.println("reconstructed dds waveform from peak amplitude change");
        lastEncoderDebounce = millis();
    }
    amplitudeLastButtonState = amplitudeEncoderButtonState;

    int amplitudeEncoderA = digitalRead(AMP_RE_CLK), amplitudeEncoderB = digitalRead(AMP_RE_DT);
    if (amplitudeEncoderA != amplitudeEncoderB && (millis() - lastEncoderDebounce) > encoderDebounceTime) {
        if (amplitudeEncoderA == LOW && signalPeakVoltage + 0.05 <= (MAX_DAC_VOLTAGE + 0.01)) {  // + 0.01 to account for inequality on 2.70 + 0.05 <= 2.75 due to floating precision
            signalPeakVoltage += 0.05;
        } else if (amplitudeEncoderB == LOW && signalPeakVoltage - 0.05 >= MIN_DAC_VOLTAGE) {
            signalPeakVoltage -= 0.05;
        }
        Serial.print("A: ");
        Serial.print(amplitudeEncoderA);
        Serial.print(" - B: ");
        Serial.print(amplitudeEncoderB);
        Serial.print("\tsignalPeakVoltage: ");
        Serial.println(signalPeakVoltage);
        lastEncoderDebounce = millis();
    }

    // WAVE_RotaryEncoder functionality, handles waveform shape selection
    // int waveformEncoderButtonState = digitalRead(WAVE_RE_SW);
    // if (waveformLastButtonState == LOW && waveformEncoderButtonState == HIGH && (millis() - lastEncoderDebounce) > buttonDebounceTime) {
    //     generateDDSLookup();
    //     Serial.println("reconstructed dds waveform from changing waveform shape");
    //     lastEncoderDebounce = millis();
    // }
    // waveformLastButtonState = waveformEncoderButtonState;

    // int waveformEncoderA = digitalRead(WAVE_RE_CLK), waveformEncoderB = digitalRead(WAVE_RE_DT);
    // if (waveformEncoderA != waveformEncoderB && (millis() - lastEncoderDebounce) > encoderDebounceTime) {
    //     if (waveformEncoderA == LOW && waveformSelect + 1 <= 2) {
    //         waveformSelect++;
    //     } else if (waveformEncoderB == LOW && waveformSelect - 1 >= 0) {
    //         waveformSelect--;
    //     }
    //     lastEncoderDebounce = millis();
    // }
}

void DACC_Handler() {
    if (DACC->DACC_ISR & DACC_ISR_TXBUFE) {
        DACC->DACC_TNPR = (uint32_t)ddsLookup;
        DACC->DACC_TNCR = ddsLookupSize;
    }
}

void ADC_Handler() {
    if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
        circular_buffer_enqueue(circularADCBuffer, adcPingPongBuffer[swapReadWriteBuffer], ADC_BUFFER_SIZE);
        ADC->ADC_RNPR = (uint32_t)adcPingPongBuffer[swapReadWriteBuffer];
        ADC->ADC_RNCR = ADC_BUFFER_SIZE;
        swapReadWriteBuffer = !swapReadWriteBuffer;
    }
}