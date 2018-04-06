/*
  xsns_20_novasds.ino - Nova SDS011/SDS021 particle concentration sensor support for Sonoff-Tasmota

  Copyright (C) 2018  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_HCSR04
/*********************************************************************************************\
 * Ultrasonic snsor HC-SR04
 * For background information see https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
 * Code provided here came from https://gist.github.com/flakas/3294829
\*********************************************************************************************/

long hc_sr204_distance = 0;
uint8_t hc_sr204_echo_pin = 0;
uint8_t hc_sr204_trig_pin = 0;

/*********************************************************/
void UltrasonicInit() {
  hc_sr204_trig_pin = pin[GPIO_HCSR04_TRIG];
  hc_sr204_echo_pin = pin[GPIO_HCSR04_ECHO];

  // set echo pin to INPUT and trig pin to OUTPUT
  pinMode(hc_sr204_echo_pin, INPUT);
  pinMode(hc_sr204_trig_pin, OUTPUT);
}

void UltrasonicRead() {
  digitalWrite(hc_sr204_trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(hc_sr204_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(hc_sr204_trig_pin, LOW);

  long duration = pulseIn(hc_sr204_echo_pin, HIGH);

  // add logic here to calculate distance
  // either in cm or in
  // distance = microsecondsToCentimeters(duration);
  hc_sr204_distance = microsecondsToInches(duration);
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


TasmotaSerial *USSerial;

long distance;

bool NovaSdsReadData1()
{
  if (! NovaSdsSerial->available()) return false;

  while ((NovaSdsSerial->peek() != 0xAA) && NovaSdsSerial->available()) {
    NovaSdsSerial->read();
  }

  byte d[8] = { 0 };
  NovaSdsSerial->read();  // skip 0xAA
  NovaSdsSerial->readBytes(d, 8);
  NovaSdsSerial->flush();

  AddLogSerial(LOG_LEVEL_DEBUG_MORE, d, 8);

  if (d[7] == ((d[1] + d[2] + d[3] + d[4] + d[5] + d[6]) & 0xFF)) {
    novasds_data.pm25 = (d[1] + 256 * d[2]);
    novasds_data.pm100 = (d[3] + 256 * d[4]);
  } else {
    AddLog_P(LOG_LEVEL_DEBUG, PSTR("SDS: " D_CHECKSUM_FAILURE));
    return false;
  }

  novasds_valid = 10;

  return true;
}

/*********************************************************************************************/

void NovaSdsSecond1()                 // Every second
{
  if (NovaSdsReadData()) {
    novasds_valid = 10;
  } else {
    if (novasds_valid) {
      novasds_valid--;
    }
  }
}

/*********************************************************************************************/

void USInit()
{
  novasds_type = 0;

  if (pin[GPIO_HCSR04_ECHO] < 99 && pin[GPIO_HCSR04_TRIG] < 99) {
    USSerial = new TasmotaSerial(pin[GPIO_HCSR04_ECHO], pin[GPIO_HCSR04_TRIG]);
    if (USSerial->begin()) {
      novasds_type = 1;
    }
  }
}

// #ifdef USE_WEBSERVER
// const char HTTP_SDS0X1_SNS[] PROGMEM = "%s"
//   "{s}SDS0X1 " D_ENVIRONMENTAL_CONCENTRATION " 2.5 " D_UNIT_MICROMETER "{m}%s " D_UNIT_MICROGRAM_PER_CUBIC_METER "{e}"
//   "{s}SDS0X1 " D_ENVIRONMENTAL_CONCENTRATION " 10 " D_UNIT_MICROMETER "{m}%s " D_UNIT_MICROGRAM_PER_CUBIC_METER "{e}";      // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
// #endif  // USE_WEBSERVER

void NovaSdsShow1(boolean json)
{
  if (novasds_valid) {
    char pm10[10];
    char pm2_5[10];
    float pm10f = (float)(novasds_data.pm100) / 10.0f;
    float pm2_5f = (float)(novasds_data.pm25) / 10.0f;
    dtostrfd(pm10f, 1, pm10);
    dtostrfd(pm2_5f, 1, pm2_5);
    if (json) {
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s,\"SDS0X1\":{\"PM2.5\":%s,\"PM10\":%s}"), mqtt_data, pm2_5, pm10);
#ifdef USE_DOMOTICZ
      DomoticzSensor(DZ_VOLTAGE, pm2_5);  // PM2.5
      DomoticzSensor(DZ_CURRENT, pm10);   // PM10
#endif  // USE_DOMOTICZ
#ifdef USE_WEBSERVER
    } else {
      snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_SDS0X1_SNS, mqtt_data, pm2_5, pm10);
#endif  // USE_WEBSERVER
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#define XSNS_21

boolean Xsns21(byte function)
{
  boolean result = false;

  if (novasds_type) {
    switch (function) {
      case FUNC_INIT:
        // USInit();
        break;
      case FUNC_PREP_BEFORE_TELEPERIOD:
        // USRead();
        break;
      case FUNC_EVERY_SECOND:
        // NovaSdsSecond();
        break;
      case FUNC_JSON_APPEND:
        NovaSdsShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_APPEND:
        NovaSdsShow(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_NOVA_SDS
