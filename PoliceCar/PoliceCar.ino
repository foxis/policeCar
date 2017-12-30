#include "LowPower.h"

#define TIME_TURNOFF 30100
#define TIME_SIREN_ON 1000
#define TIME_STOP_ON 19000
#define TIME_HEAD_OFF 20000
#define TIME_STOP_OFF 21000
#define TIME_SIREN_OFF 23000
#define TIME_BLINKERS_OFF 30000


#define PIN_HEAD_L A0
#define PIN_HEAD_R A1
#define PIN_STOP_L 5
#define PIN_STOP_R 6
#define PIN_BLINK_L 11
#define PIN_BLINK_R 10
#define PIN_SPEAKER 9

long start_millis = 0;
int state = 0;
bool blinkers = false;
bool siren = false;

void setup()
{
  pinMode(PIN_HEAD_L, OUTPUT);
  pinMode(PIN_HEAD_R, OUTPUT);
  pinMode(PIN_STOP_L, OUTPUT);
  pinMode(PIN_STOP_R, OUTPUT);
  pinMode(PIN_BLINK_L, OUTPUT);
  pinMode(PIN_BLINK_R, OUTPUT);
  pinMode(PIN_SPEAKER, OUTPUT);

  digitalWrite(PIN_SPEAKER, HIGH);

  start_millis = millis();
}

long readVcc() 
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
} 

void do_blinkers(long now)
{
  static long last_millis = 0;
  static int state = 0;
  static int sequence[] = {
    PIN_BLINK_L,
    PIN_BLINK_R,
    PIN_BLINK_R,
    PIN_BLINK_R,
    PIN_BLINK_R,
    PIN_BLINK_R,
    PIN_BLINK_R,
    PIN_BLINK_L,
    PIN_BLINK_L,
    PIN_BLINK_L,
    PIN_BLINK_L,
    PIN_BLINK_L,
  };

  if (!blinkers)
    return;

  if (now - last_millis < 50)
    return;

  digitalWrite(sequence[state], state % 2);
  state = (state + 1) % (sizeof(sequence) / sizeof(sequence[0]));

  last_millis = now;
}

void do_lowbat()
{
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(PIN_STOP_L, HIGH);
    digitalWrite(PIN_STOP_R, HIGH);
    digitalWrite(PIN_HEAD_L, HIGH);
    digitalWrite(PIN_HEAD_R, HIGH);
    delay(500);
    digitalWrite(PIN_STOP_L, LOW);
    digitalWrite(PIN_STOP_R, LOW);
    digitalWrite(PIN_HEAD_L, LOW);
    digitalWrite(PIN_HEAD_R, LOW);
    delay(500);
  }
}

void do_beepbeep()
{
  for (int i = 0; i < 30; i++)
  {
    tone(PIN_SPEAKER, 440, 10);
    delay(5);
    noTone(PIN_SPEAKER);
    digitalWrite(PIN_SPEAKER, HIGH);
    delay(5);
  }
}

void do_siren(long now)
{
  static int freq = 150;
  static int freq_delta = 1;
  static long last_millis = 0;
  static int state = 0;
  
  if (!siren)
    return;

  if (now - last_millis < 1)
    return;

  if (freq_delta == 1 && freq >= 1800)
  {
    freq_delta = -1;
  }
  else if (freq_delta == -1 && freq <= 150)
  {
    freq_delta = 1;
    state++;
    if (state > 2)
    {
      freq_delta = 0;
      state = 0;
    }
  }
  else if (freq_delta == 0)
  {
    if (now - last_millis < 5)
      return;

    if ((state == 25 || state == 50) && now - last_millis < 50)
      return;

    if (!(state % 2))
      tone(PIN_SPEAKER, 440, 10);
    else
      noTone(PIN_SPEAKER);

    state++;
    if (state > 75)
    {
      freq_delta = 1;
      freq = 150;
      state = 0;
    }
  }
  else
  {
    tone(PIN_SPEAKER, freq, 10);     // Beep pin, freq, time
    freq += freq_delta;
  }
  
  last_millis = now;
}

void loop() 
{
  long now = millis();
  
  // turn on stop and head lights
  if (state == 0)
  {

    if (readVcc() < 2900)
    {
      do_lowbat();
      state = -1;
      return;
    }
    
    digitalWrite(PIN_STOP_L,HIGH);
    digitalWrite(PIN_STOP_R,HIGH);
    digitalWrite(PIN_HEAD_L,HIGH);
    digitalWrite(PIN_HEAD_R,HIGH);
    state ++;
  }

  // turn on blinkies and siren
  if (now - start_millis > TIME_SIREN_ON && state == 1)
  {
    digitalWrite(PIN_STOP_L,LOW);
    digitalWrite(PIN_STOP_R,LOW);
    blinkers = true;
    siren = true;
    state ++;
  }

  // turn on stop lights
  if (now - start_millis > TIME_STOP_ON && state == 2)
  {
    digitalWrite(PIN_STOP_L,HIGH);
    digitalWrite(PIN_STOP_R,HIGH);
    state ++;
  }
  
  // turn off headlamps
  if (now - start_millis > TIME_HEAD_OFF && state == 3)
  {
    digitalWrite(PIN_HEAD_L,LOW);
    digitalWrite(PIN_HEAD_R,LOW);
    state ++;
  }
  
  // turn off stop lights
  if (now - start_millis > TIME_STOP_OFF && state == 4)
  {
    digitalWrite(PIN_STOP_L,LOW);
    digitalWrite(PIN_STOP_R,LOW);
    state ++;
  }
  
  // turn off sirens and blinkies
  if (now - start_millis > TIME_SIREN_OFF && state == 5)
  {
    noTone(PIN_SPEAKER);
    digitalWrite(PIN_SPEAKER, HIGH);

    siren =false;
    state ++;
  }

  if (now - start_millis > TIME_BLINKERS_OFF && state == 6)
  {
    digitalWrite(PIN_BLINK_L, LOW);
    digitalWrite(PIN_BLINK_R, LOW);

    blinkers = false;
    state ++;
  }

  do_siren(now);
  do_blinkers(now);
  
  // go into a deep sleep
  if (now - start_millis > TIME_TURNOFF || state == -1)
  {
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}
