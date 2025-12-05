/*
  ===================================================================
  - AUTO_MODE = 1 → automatic scripted potentiometer values
  - AUTO_MODE = 0 → manual real potentiometer readings
*/

#include <avr/io.h>

// =================== User Options ===================
#define SERIAL_BAUD     9600
#define PRINT_PERIOD_MS 50
#define CLAMP_NEGATIVE  1
#define N_DUMMY         3
#define N_AVG_HW        32
#define N_AVG_SW        16

#define AUTO_MODE 1   // 0 = Manual, 1 = Automatic

// =================== Function: Voltage difference to pressure (psi) ===================
static inline float vdiff_to_psi(float vdiff_volts) {
  float mV = vdiff_volts * 1000.0f;
  return mV / 0.87f; // Example calibration for TBP sensor (0.87 mV/psi)
}

// =================== PWM (fixed 50 Hz) ===================
int prescaler = 256;
float potDC1 = 0, potDC2 = 0, potDC3 = 0, potDC4 = 0;
const float FIXED_PWM_FREQ = 50.0;

const int SW1=50, SW2=51, SW3=52, SW4=53;
const int PWM1=5,  PWM2=6,  PWM3=7,  PWM4=8;

const int POT1 = A12, POT2 = A13, POT3 = A14, POT4 = A15;

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

// =================== ADC: HW Differential (×200) ===================
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

  Serial.println(F("P1\tP2\tP3\tP4\tPOT1\tPOT2\tPOT3\tPOT4"));
}

// =================== Loop ===================
void loop() {
  unsigned long now = millis();
  int pot1_val, pot2_val, pot3_val, pot4_val;

#if AUTO_MODE
  unsigned long t = now / 1000; // seconds

  // Channel 1
  if (t < 5) pot1_val = 0;
  else if (t < 10) pot1_val = 400;
  else if (t < 20) pot1_val = 800;
  else pot1_val = 0;

  // Channel 2
  if (t < 8) pot2_val = 0;
  else if (t < 16) pot2_val = 600;
  else pot2_val = 0;

  // Channel 3
  if (t < 3) pot3_val = 0;
  else if (t < 12) pot3_val = 700;
  else pot3_val = 0;

  // Channel 4
  if (t < 5) pot4_val = 0;
  else if (t < 20) pot4_val = 500;
  else pot4_val = 0;

#else
  pot1_val = analogRead(POT1);
  pot2_val = analogRead(POT2);
  pot3_val = analogRead(POT3);
  pot4_val = analogRead(POT4);
#endif

  potDC1 = pot1_val * 100.0 / 1023.0;
  potDC2 = pot2_val * 100.0 / 1023.0;
  potDC3 = pot3_val * 100.0 / 1023.0;
  potDC4 = pot4_val * 100.0 / 1023.0;

  pPWM(FIXED_PWM_FREQ, potDC1, potDC2, potDC3, potDC4);

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

  // Output to Serial Plotter
  Serial.print(P1,2); Serial.print('\t');
  Serial.print(P2,2); Serial.print('\t');
  Serial.print(P3,2); Serial.print('\t');
  Serial.print(P4,2); Serial.print('\t');
  Serial.print(pot1_val); Serial.print('\t');
  Serial.print(pot2_val); Serial.print('\t');
  Serial.print(pot3_val); Serial.print('\t');
  Serial.println(pot4_val);

  delay(PRINT_PERIOD_MS);
}
