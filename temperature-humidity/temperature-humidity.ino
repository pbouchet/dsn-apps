#include <DHT.h>
#include <SdFat.h>
#include <Api.h>
#include <Configuration.h>
#define DHTPIN A7
#define DHTTYPE DHT22

uint32_t const REFRESH_INTERVAL = 600000;
const char HOST[] PROGMEM = "dsn.absolutlabs.com";
const char PATH[] PROGMEM = "/";
const char ENDPOINT_HUMIDITY[] PROGMEM = "channels/humidity/points";
const char ENDPOINT_TEMPERATURE[] PROGMEM = "channels/temperature/points";
const char KEY_HUMIDITY[] PROGMEM = "value";
const char KEY_TEMPERATURE[] PROGMEM = "value";

DHT dht(DHTPIN, DHTTYPE);
uint32_t deadline = 0;
char buffer[4096];
Wifly wifly(Serial3, A0, A4, A5, A6);
Api api(wifly, buffer, 4096, HOST, PATH);
Configuration config(Serial, wifly, 77);

void setup() {
  Serial.begin(115200);
  wifly.initialize();
  config.synchronize(10000);
  // {"cmd":"config", "ssid":"absolutLabs", "phrase:"balibump", "mode":"dhcp"}
  Serial.println(F("Configuration done."));
  Serial.println(F("========================================================================="));
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  dht.begin();
}

void loop() {
  if (millis() < deadline)
    return;

  deadline = millis() + REFRESH_INTERVAL;

  digitalWrite(13, HIGH);

  float humidity = dht.readHumidity();
  if (isnan(humidity) || humidity <= 0.0f)
    return;

  Serial.print(F("New humidity measurement: "));
  Serial.print(humidity);
  Serial.println("%");
  
  char value[9] = {0};
  dtostrf(humidity, 4, 2, value);
  
  if (!api.connected()) {
    api.connect();
  }
  Serial.println(("-------------------------------------------------------------------------"));
  Serial.println(F("HTTP request sent: "));
  api.post(ENDPOINT_HUMIDITY, KEY_HUMIDITY, value);

  Serial.println(("-------------------------------------------------------------------------"));
  Serial.println(F("Server response: "));
  Serial.println(api);

  Serial.println(("-------------------------------------------------------------------------"));
  float temperature = dht.readTemperature();
  if (isnan(temperature) || temperature <= 0.0f)
    return;

  Serial.print(F("New temperature measurement: "));
  Serial.print(temperature);
  Serial.println(" degrees");

  memset(value, 0x00, 9);
  dtostrf(temperature, 4, 2, value);

  if (!api.connected()) {
    api.connect();
  }
  Serial.println(("-------------------------------------------------------------------------"));
  Serial.println(F("HTTP request sent: "));
  api.post(ENDPOINT_TEMPERATURE, KEY_TEMPERATURE, value);
  
  Serial.println(("-------------------------------------------------------------------------"));
  Serial.println(F("Server response: "));
  Serial.println(api);

  digitalWrite(13, LOW);
  Serial.println(F("========================================================================="));
}
