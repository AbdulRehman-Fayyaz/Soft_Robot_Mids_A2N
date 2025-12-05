/*
  ===================================================================
  SOFT ROBOTICS SEQUENCER (6-STEP LOOP)
  - Based on original hardware configuration
  - Implements specific P1/P2/P3/P4 sequence
  ===================================================================
*/

#include <avr/io.h>

// ==============================================================================
// ******* USER CONFIGURATION SECTION (EDIT HERE) *******
// ==============================================================================

// --- 1. PRESSURE PERCENTAGES (0.0 to 100.0) ---
// How much power/pressure when the valve is "UP" (Open/Active)
float P1_HIGH_VAL   = 100.0; 
float P2_HIGH_VAL   = 100.0;
float P34_HIGH_VAL  = 100.0; // Extension actuators

// How much power/pressure when the valve is "DOWN" (Released/Closed)
float P1_LOW_VAL    = 0.0;
float P2_LOW_VAL    = 0.0;
float P34_LOW_VAL   = 0.0;

// --- 2. TIME DELAYS (in Milliseconds) ---
// How long to wait after each step before moving to the next.
// 1000 = 1 second.

unsigned long DELAY_STEP_1 = 2000; // After P1 goes UP
unsigned long DELAY_STEP_2 = 1000; // After P2 goes DOWN
unsigned long DELAY_STEP_3 = 2000; // After P3 & P4 go UP
unsigned long DELAY_STEP_4 = 2000; // After P2 goes UP
unsigned long DELAY_STEP_5 = 1000; // After P1 goes DOWN
unsigned long DELAY_STEP_6 = 2000; // After P3 & P4 go DOWN

// ==============================================================================
// ******* END OF USER CONFIGURATION *******
// ==============================================================================

// =================== Hardware Definitions (UNCHANGED) ===================
#define SERIAL_BAUD     9600
#define PRINT_PERIOD_MS 50
#define CLAMP_NEGATIVE  1
#define N_DUMMY         3
#define N_AVG_HW        32
#define N_AVG_SW        16

// Function: Voltage difference to pressure (psi)
static inline float vdiff_to_psi(float vdiff_volts) {
  float mV = vdiff_volts * 1000.0f;
  return mV / 0.87f; 
}

// PWM Settings
int prescaler = 256;
const float FIXED_PWM_FREQ = 50.0;

// Pin Definitions
const int SW1=50, SW2=51, SW3=52, SW4=53;
const int PWM1=5,  PWM2=6,  PWM3=7,  PWM4=8;
const int POT1 = A12, POT2 = A13, POT3 = A14, POT4 = A15;

// Variables to hold current target duty cycle
// THESE VARIABLES "REMEMBER" THE STATE. 
// If a step does not change them, they stay the same (HOLD).
float currentDC1 = 0;
float currentDC2 = 0;
float currentDC3 = 0;
float currentDC4 = 0;

// Sequencing Variables
int currentStep = 6; // Start at end of loop so it resets to Step 1 immediately
unsigned long lastStepTime = 0;
unsigned long stepInterval = 0;

static inline float clampPct(float x){ return x<0?0:(x>100?100:x); }

// Configure PWM frequency and duty cycles for four channels
void pPWM(float pwmfreq, float pwmDC1, float pwmDC2, float pwmDC3, float pwmDC4) {
  ICR3 = F_CPU / (prescaler * pwmfreq * 2);
  ICR4 = F_CPU / (prescaler * pwmfreq * 2);

  pwmDC1 = clampPct(pwmDC1);
  pwmDC2 = clampPct(pwmDC2);
  pwmDC3 = clampPct(pwmDC3);
  pwmDC4 = clampPct(pwmDC4);

  OCR3A = (uint16_t)(ICR3 * (pwmDC1 * 0.01f)); // D5  (OC3A)
  OCR4A = (uint16_t)(ICR4 * (pwmDC2 * 0.01f)); // D6  (OC4A)
  OCR4B = (uint16_t)(ICR4 * (pwmDC3 * 0.01f)); // D7  (OC4B)
  OCR4C = (uint16_t)(ICR4 * (pwmDC4 * 0.01f)); // D8  (OC4C)
}

// =================== ADC: HW Differential (x200) ===================
#define MUX200_A1A0 0b01011
#define MUX200_A3A2 0b01111

static inline void adc_init_AVcc() {
  ADMUX  = (1<<REFS0);
  ADCSRB = 0;
  ADCSRA = (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
  DIDR0 |= (1<<ADC0D)|(1<<ADC1D)|(1<<ADC2D)|(1<<ADC3D)|
           (1<<ADC4D)|(1<<ADC5D)|(1<<ADC6D)|(1<<ADC7D);
}

static inline int16_t adc_read_diff_hw(uint8_t mux4_0) {
  ADMUX  = (1<<REFS0) | (mux4_0 & 0x1F);
  ADCSRB = 0;

  for (int i=0; i<N_DUMMY; i++) {
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    (void)ADCL; (void)ADCH;
  }

  long acc = 0;
  for (int i=0; i<N_AVG_HW; i++) {
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    uint8_t low  = ADCL;
    uint8_t high = ADCH;
    int16_t code = (high<<8) | low;
    if (code & 0x0200) code -= 1024;
    acc += code;
  }
  return (int16_t)(acc / N_AVG_HW);
}

static inline float code_to_vdiff_hw(int16_t code) {
  const float VREF = 5.0f;
  const int   GAIN = 200;
  return ((float)code / 512.0f) * (VREF / (float)GAIN);
}

// =================== ADC: SW Differential on A8..A11 ===================
static inline int analogReadAvg(uint8_t pin, int n=N_AVG_SW) {
  long acc=0;
  for (int i=0;i<n;i++) acc += analogRead(pin);
  return (int)(acc / n);
}

static inline float read_vdiff_sw(uint8_t pin_plus, uint8_t pin_minus) {
  static bool ref_set = false;
  if (!ref_set) { analogReference(DEFAULT); ref_set = true; }

  int rp = analogReadAvg(pin_plus);
  int rm = analogReadAvg(pin_minus);
  int diff_code = rp - rm;

  const float VREF = 5.0f;
  return (diff_code * VREF) / 1023.0f;
}

// =================== Offsets ===================
int16_t off_hw_s1=0, off_hw_s2=0;
float   off_sw_s3=0.0f, off_sw_s4=0.0f;

// =================== Setup ===================
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PWM1, OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT); pinMode(PWM4, OUTPUT);

  TCCR3A = _BV(COM3A1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS42);

  ICR3 = F_CPU / (prescaler * FIXED_PWM_FREQ * 2);
  ICR4 = F_CPU / (prescaler * FIXED_PWM_FREQ * 2);

  adc_init_AVcc();
  delay(300);

  off_hw_s1 = adc_read_diff_hw(MUX200_A1A0);
  off_hw_s2 = adc_read_diff_hw(MUX200_A3A2);
  off_sw_s3 = read_vdiff_sw(A9,  A8);
  off_sw_s4 = read_vdiff_sw(A11, A10);

  Serial.println(F("P1_PSI\tP2_PSI\tP3_PSI\tP4_PSI\tSTEP\tDC1\tDC2\tDC34"));
  
  // Initialize start time
  lastStepTime = millis();
}

// =================== Loop ===================
void loop() {
  unsigned long now = millis();

  // =================== SEQUENCER LOGIC ===================
  // Check if it is time to move to the next step
  if (now - lastStepTime >= stepInterval) {
    
    // Advance step
    currentStep++;
    if (currentStep > 6) currentStep = 1; // Loop back to 1
    
    lastStepTime = now; // Reset timer

    // Apply Logic based on Step Number
    switch (currentStep) {
      case 1: 
        // Step 1: Pressure 1 UP, Pressure 2 DOWN
        currentDC1 = P1_HIGH_VAL; // CHANGE: P1 goes HIGH
        currentDC2 = P2_LOW_VAL;  // CHANGE: P2 goes LOW
        // HOLDING: P3 & P4 remain at LOW (from previous Step 6)
        stepInterval = DELAY_STEP_1;
        break;

      case 2:
        // Step 2: Pressure 2 goes DOWN (redundant but ensures low)
        currentDC2 = P2_LOW_VAL; // CHANGE: P2 stays LOW
        // HOLDING: P1 remains HIGH (from Step 1)
        // HOLDING: P3 & P4 remain LOW
        stepInterval = DELAY_STEP_2;
        break;

      case 3:
        // Step 3: Pressure 3 and 4 go UP
        currentDC3 = P34_HIGH_VAL; // CHANGE: P3 goes HIGH
        currentDC4 = P34_HIGH_VAL; // CHANGE: P4 goes HIGH
        // HOLDING: P1 remains HIGH (from Step 1)
        // HOLDING: P2 remains LOW
        stepInterval = DELAY_STEP_3;
        break;

      case 4:
        // Step 4: Pressure 2 goes UP
        currentDC2 = P2_HIGH_VAL; // CHANGE: P2 goes HIGH
        // HOLDING: P1 remains HIGH
        // HOLDING: P3 & P4 remain HIGH (from Step 3)
        stepInterval = DELAY_STEP_4;
        break;

      case 5:
        // Step 5: Pressure 1 goes DOWN
        currentDC1 = P1_LOW_VAL; // CHANGE: P1 goes LOW
        // HOLDING: P2 remains HIGH (from Step 4)
        // HOLDING: P3 & P4 remain HIGH
        stepInterval = DELAY_STEP_5;
        break;

      case 6:
        // Step 6: Pressure 3 and 4 go DOWN
        currentDC3 = P34_LOW_VAL; // CHANGE: P3 goes LOW
        currentDC4 = P34_LOW_VAL; // CHANGE: P4 goes LOW
        // HOLDING: P1 remains LOW
        // HOLDING: P2 remains HIGH
        stepInterval = DELAY_STEP_6;
        break;
    }
  }

  // =================== Hardware Actuation ===================
  // Send the calculated values to the PWM function
  pPWM(FIXED_PWM_FREQ, currentDC1, currentDC2, currentDC3, currentDC4);

  // =================== Sensor Reading (Unchanged) ===================
  int16_t c1 = adc_read_diff_hw(MUX200_A1A0) - off_hw_s1;
  int16_t c2 = adc_read_diff_hw(MUX200_A3A2) - off_hw_s2;
  float v1 = code_to_vdiff_hw(c1);
  float v2 = code_to_vdiff_hw(c2);

  float v3 = read_vdiff_sw(A9,  A8)  - off_sw_s3;
  float v4 = read_vdiff_sw(A11, A10) - off_sw_s4;

  float P1 = vdiff_to_psi(v1);
  float P2 = vdiff_to_psi(v2);
  float P3 = vdiff_to_psi(v3);
  float P4 = vdiff_to_psi(v4);

#if CLAMP_NEGATIVE
  if (P1 < 0) P1 = 0;
  if (P2 < 0) P2 = 0;
  if (P3 < 0) P3 = 0;
  if (P4 < 0) P4 = 0;
#endif

  // =================== Serial Output ===================
  // Printing Pressure values followed by Debug info (Step # and Duty Cycles)
  Serial.print(P1,2); Serial.print('\t');
  Serial.print(P2,2); Serial.print('\t');
  Serial.print(P3,2); Serial.print('\t');
  Serial.print(P4,2); Serial.print('\t');
  
  // Debug columns: Current Step, DC1, DC2, DC3
  Serial.print(currentStep); Serial.print('\t');
  Serial.print((int)currentDC1); Serial.print('\t');
  Serial.print((int)currentDC2); Serial.print('\t');
  Serial.println((int)currentDC3);

  delay(PRINT_PERIOD_MS);
}