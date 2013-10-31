#include <Api.h>
#include <Configuration.h>
#include <HardwareSerial.h>
#include <SdFat.h>

// Voltage level corresponding to background noise (subtracted from samples)
#define REF_VOLTAGE 508
// Voltage level corresponding to actual activity
#define THRESHOLD 200


#define BUFFER_SIZE 512
// 12500 ticks @250kHZ yields a 50ms sampling period
#define TICKS_PER_PERIOD 12500
#define INTERNET_TRIGGER
#define INTERNET_TRIGGER_INTERVAL 60000

const char HOST[] PROGMEM = "dsn.absolutlabs.com";
const char PATH[] PROGMEM = "/";
const char ENDPOINT[] PROGMEM = "channels/current/points";
const char KEY_LEVEL[] PROGMEM = "value";

volatile uint16_t reading;
volatile bool flag;

ISR(ADC_vect) {
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  reading = ((high << 8) | low);  
  TIFR1 |= ((1 << OCF1A) | (1 << OCF1B));
  flag = true;
}

char buffer[2048];

Wifly wifly(Serial3, A0, A4, A5, A6);
Api api(wifly, buffer, 2048, HOST, PATH);
Configuration config(Serial, wifly, 77);

void setup () {
  Serial.begin(115200);
  wifly.initialize();
  config.synchronize(10000);
  // {"cmd":"config", "ssid":"absolutLabs", "phrase:"balibump", "mode":"dhcp"}
  Serial.println(F("Configuration done."));
  Serial.println(F("========================================================================="));
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  initializeTimer();
  initializeADC();
  
  startADC();
  startTimer();
}

void loop () {
  if (flag) {
    updateOutput(reading);
    flag = false;
  }
}

void updateOutput(uint16_t newSample) {
  static uint16_t buffer[BUFFER_SIZE];
  static uint16_t head;
  static uint32_t sum;
  
  // remove oldest sample
  sum -= buffer[head];
  // add newest sample
  int16_t offset = newSample - REF_VOLTAGE;
  buffer[head] = abs(offset);
  sum += buffer[head];
  // update the cursor pointer
  ++head;
  head %= BUFFER_SIZE;
  
  uint16_t level = 32*sum/BUFFER_SIZE;

#ifdef BAR_GRAPH
  // Send an horizontal bar graph to the UART
  for (int i = 0; i < level; i++) {
    Serial.print("-");
  }
  Serial.print("\n");
#endif

#ifdef INTERNET_TRIGGER
  stopTimer();
  stopADC();
  
  if (level > THRESHOLD) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
  
  static uint32_t deadline;
  Serial.println(level);
  if (level > THRESHOLD) {
        Serial.println(F("Activity detected"));
        if (millis() <= deadline) {
            Serial.println(F("Lock is enabled -- activity ignored."));
        } else {
            Serial.println(F("Lock is disabled -- new data point"));
      if (!api.connected()) {
          api.connect();
      }
      Serial.println(("-------------------------------------------------------------------------"));
      Serial.println(F("HTTP request sent: "));
      const char value[] = "1";
      api.post(ENDPOINT, KEY_LEVEL, value);
      Serial.println(("-------------------------------------------------------------------------"));
      Serial.println(F("Server response: "));
      Serial.println(api);
      Serial.println(("-------------------------------------------------------------------------"));
      
      deadline = millis() + INTERNET_TRIGGER_INTERVAL;
      }
  }
  startADC();
  startTimer();
#endif
}


void initializeADC() {
  // Set ADC pin as input
  DDRF &= ~(1 << PINF1);
  // Disable the digital input buffer on the ADC pin
  DIDR1 |= (1 << ADC1D);
  // Clear the Multiplexer Selection Register
  ADMUX = 0x00;
  // Set AVCC as the voltage reference
  ADMUX |= (1 << REFS0);
  // Set data presentation to right-aligned
  ADMUX &= ~(1 << ADLAR);
  // Set ADC1 as the input channel
  ADMUX |= (1 << MUX0);
  ADCSRB &= ~(1 << MUX5);
  // Set Timer/Counter1 as the Auto Trigger Source
  ADCSRB |= ((1 << ADTS2) | (1 << ADTS0));
  // Clear the Control & Status register
  ADCSRA = 0x00;
  // Clear the ADC Interrupt Flag
  ADCSRA |= (1 << ADIF);
  // Enable ADC Interrupt
  ADCSRA |= (1 << ADIE);
  // Set the ADC prescaler to 128 (sets ADC clock @125kHz)
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  // Turn ADC on
  ADCSRA |= (1 << ADEN);
}

void startADC() {
  // Start auto-triggered ADC conversion
  ADCSRA |= (1 << ADATE);
}

void stopADC() {
  // Disable ADC auto-trigger
  ADCSRA &= ~(1 << ADATE);
}

void initializeTimer() {
  // Clear the Timer/Counter1 control registers
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  // Set the Waveform Generation Mode to CTC
  TCCR1B |= (1 << WGM12);
  // Set the Channel A threshold to reset the counter
  OCR1A = TICKS_PER_PERIOD;
  // Set the Channel B threshold to trigger the ADC
  OCR1B = TICKS_PER_PERIOD;
  // Disable all Timer/Counter1 interrupts while using ADC auto-trigger
  TIMSK1 = 0x00;
}

void startTimer() {
  // Set the clock prescaler to 64 (250kHz / 4µs)
  TCCR1B |= ((1 << CS11) | (1 << CS10)); 
}

void stopTimer() {
  // Disable the Timer/Counter clock
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
}
