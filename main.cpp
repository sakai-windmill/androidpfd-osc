/*
SD
GPIO 5--->CS
GPIO23--->MOSI
GPIO18--->CLK
GPIO19--->MISO

IMU
GPIO21--->SDA
GPIO22--->CLK(SCL)

HC-SR04
GPIO33--->TRIG
GPIO32--->ECHO

Airspeed
GPIO25--->TLV4968-1T

GPS
GPIO26--->TX
GPIO27--->RX

D52
GPIO13--->TX
GPIO14--->RX
VCC--->BR3
GND--->BR1, BR2
*/

#define ARDUINOOSC_DEBUGLOG_ENABLE
#include <ArduinoOSCWiFi.h>
const char *ssid = "esp-wifi";
const char *pwd = "12345678";
const IPAddress ip(192, 168, 0, 1);
const IPAddress subnet(255, 255, 255, 0);

#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPSPlus.h>
#include "ANT.h"
#include "ANTPLUS.h"
#include "SparkFun_BNO080_Arduino_Library.h"
String wifidata = "";
String neutraldata = "0.00,0.00,0.00";
int i;
float f;

const uint8_t NETWORK_KEY[] = {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}; // get this from thisisant.com
#define CHANNEL_0 0
#define CHANNEL_1 1

#define antSerial Serial2
int cad;
int power;
ArduinoSerialAntWithCallbacks ant;
AntPlusRouter router;
ProfileBicyclePowerDisplay bikePower;
TinyGPSPlus gps;
BNO080 myIMU;

void powerOnlyDataPageHandler(BicyclePowerStandardPowerOnly &msg, uintptr_t data)
{

  // Serial.print("Cadence:");
  // Serial.print(msg.getInstantaneousCadence());
  cad = msg.getInstantaneousCadence();
  // Serial.print(", ");
  // Serial.print("Power:");
  // Serial.println(msg.getInstantaneousPower());
  power = msg.getInstantaneousPower();
  delay(10);
}

#define ss Serial1

void writeFile(fs::FS &fs, String path, String message)
{
  Serial.printf("Writing file:");
  Serial.println(path);

  File file = fs.open(path, FILE_WRITE);

  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }

  file.close();
}

void appendFile(fs::FS &fs, String path, String message)
{
  // Serial.printf("Appending to file:");
  // Serial.println(path);
  File file = fs.open(path, FILE_APPEND);

  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    // Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }

  file.close();
}

int inPin = 25;
int TRIG = 33;
int ECHO = 32;
double air;
unsigned long rate;
short hn = 0;
short hp = 0;
unsigned long c[5] = {0};
unsigned long count = 0;
double roundtrip[3];
double speed_of_sound = 331.5 + 0.6 * 25.0;
double distance;

void chiho(void *arg)
{
  while (1)
  {
    c[4] = c[3];
    c[3] = c[2];
    c[2] = c[1];
    c[1] = c[0];
    c[0] = 0;
    rate = millis();
    while ((millis() - rate) < 100)
    {
      hn = digitalRead(inPin);
      // Serial.println(hn);
      if (hp == 0 && hn == 1)
      {
        hp = 1;
      }
      else if (hp == 1 && hn == 0)
      {
        c[0]++;
        hp = 0;
      }
    }
    count = c[0] + c[1] + c[2] + c[3] + c[4];
    if (count < 3)
    {
      air = 0;
    }
    else
    {
      air = (109.138 * (double)count + 202.986) / 1000;
    }
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);
      roundtrip[i] = pulseIn(ECHO, HIGH); // 往復にかかった時間が返却される[マイクロ秒]
      // delay(50);
    }
    int j, k;
    double tmp;

    for (j = 0; j < 3; j++)
    {
      for (k = j + 1; k < 3; k++)
      {
        if (roundtrip[j] > roundtrip[k])
        {
          tmp = roundtrip[j];
          roundtrip[j] = roundtrip[k];
          roundtrip[k] = tmp;
        }
      }
    }
    // distance = roundtrip[1] / 2 * speed_of_sound * 100 / 1000000;
    distance = roundtrip[1] * speed_of_sound / 20000;
    // Serial.print(distance);
    // Serial.print(",");
    // Serial.println(air, 2);
    delay(1);
  }
}
void kaho(void *arg)
{
  while (1)
  {
    String fname = "";
    String fnamecheck = "";
    while (1)
    {
      if (ss.available() > 0)
      {
        if (gps.encode(ss.read()))
        {
          fname = "/";
          fname += String(gps.date.year());
          if (gps.date.month() < 10)
            fname += "0";
          fname += String(gps.date.month());
          if (gps.date.day() < 10)
            fname += "0";
          fname += String(gps.date.day());
          fnamecheck = fname;
          if (gps.time.hour() < 10)
            fname += "0";
          fname += String(gps.time.hour());
          if (gps.time.minute() < 10)
            fname += "0";
          fname += String(gps.time.minute());
          if (gps.time.second() < 10)
            fname += "0";
          fname += String(gps.time.second());
          fname += ".csv";
          if (fnamecheck != "/20000000")
          {
            break;
          }
        }
      }
    }
    // Serial.println(fname);
    writeFile(SD, fname, "");
    appendFile(SD, fname, "time,latitude,longitude,cadence,power,altitude,airspeed,speedmeter[500ms],w,x,y,z,roll,pitch,yaw,nroll,npitch,nyaw\n");
    while (1)
    {
      String str = "";
      String hms = "";

      for (int i = 0; i < 10; i++)
      {

        while (1)
        {
          hms = "";
          if (ss.available() > 0)
          {
            if (gps.encode(ss.read()))
            {
              if (gps.time.hour() < 10)
                hms += "0";
              hms += String(gps.time.hour());
              hms += ":";
              if (gps.time.minute() < 10)
                hms += "0";
              hms += String(gps.time.minute());
              hms += ":";
              if (gps.time.second() < 10)
                hms += "0";
              hms += String(gps.time.second());
              hms += ",";
              hms += String(gps.location.lat(), 8);
              hms += ",";
              hms += String(gps.location.lng(), 8);
              // Serial.println(hms);
              break;
            }
          }
        }
        double x;
        double y;
        double z;
        double w;
        float roll;
        float pitch;
        float yaw;
        float nroll;
        float npitch;
        float nyaw;
        float sroll = roll - nroll;
        float spitch = pitch - npitch;
        float syaw = yaw - nyaw;

        if (myIMU.dataAvailable() == true)
        {
          x = myIMU.getQuatI();
          y = myIMU.getQuatJ();
          z = myIMU.getQuatK();
          w = myIMU.getQuatReal();
          double t0 = 2.0 * (w * x + y * z);
          double t1 = 1.0 - 2.0 * (x * x + y * y);
          roll = atan2(t0, t1);
          double t2 = 2.0 * (w * y - z * x);
          if (t2 > 1.0)
          {
            t2 = 1.0;
          }
          if (t2 < -1.0)
          {
            t2 = -1.0;
          }
          pitch = asin(t2);
          double t3 = 2.0 * (w * z + x * y);
          double t4 = 1.0 - 2.0 * (y * y + z * z);
          yaw = atan2(t3, t4);
          roll *= 57.2957795131;
          pitch *= 57.2957795131;
          yaw *= -57.2957795131;

          yaw -= 90;

          if (yaw < 0)
          {
            yaw = 360 + yaw;
          }
        }
        router.loop();

        String vec = hms + "," + String(cad) + "," + String(power) + "," + String(distance, 2) + "," + String(air, 2) + "," + String(count) + "," + String(w, 8) + "," + String(x, 8) + "," + String(y, 8) + "," + String(z, 8) + "," + String(roll, 3) + "," + String(pitch, 3) + "," + String(yaw, 3) + "," + String(sroll, 3) + "," + String(spitch, 3) + "," + String(syaw, 3) + "\n";
        //Serial.println(vec);
        str += vec;
        wifidata = String(sroll, 2) + "," + String(spitch, 2) + "," + String(syaw, 2);
        // 文字列分割処理
        char Buf[30];
        neutraldata.toCharArray(Buf,50);
        char delim[] = ",";
        char *token;
        token = strtok(Buf, delim);
        nroll = atof(token);
        token = strtok(NULL, delim);
        npitch = atof(token);
        token = strtok(NULL, delim);
        nyaw = atof(token);

      }
      appendFile(SD, fname, str);
    }
    delay(1);
  }
}

void setup()
{
  pinMode(inPin, INPUT_PULLUP);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  Serial.begin(115200);
  byte data;
  ss.begin(115200, SERIAL_8N1, 26, 27);
  antSerial.begin(115200, SERIAL_8N1, 13, 14);
  ant.setSerial(antSerial);
  delay(100);
  router.setDriver(&ant); // never touch ant again
  router.setAntPlusNetworkKey(NETWORK_KEY);
  router.setProfile(CHANNEL_0, &bikePower);
  bikePower.onBicyclePowerStandardPowerOnly(powerOnlyDataPageHandler);
  bikePower.begin();
  uint8_t status = bikePower.waitForPair();
  Wire.begin();
  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Wire.setClock(400000); // Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(10); // Send data update every 50ms
  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP(ssid, pwd);
  if (!SD.begin())
  {
    Serial.println("Card Mount Failed");
    return;
  }

  delay(100);
  xTaskCreatePinnedToCore(chiho, "chiho", 8192, NULL, 1, NULL, 1); //マルチタスク
  xTaskCreatePinnedToCore(kaho, "kaho", 8192, NULL, 2, NULL, 0);
  disableCore0WDT();

  OscWiFi.publish("192.168.0.2", 55555, "/publish/value", 0, 0.0, wifidata); //メイン電装用
  OscWiFi.publish("192.168.0.3", 55555, "/publish/value", 0, 0.0, wifidata); //搭載用
  OscWiFi.publish("192.168.0.4", 55555, "/publish/value", 0, 0.0, wifidata);

  OscWiFi.subscribe(55555, "/bind/values", i, f, neutraldata);

}
void loop()
{
  OscWiFi.update();
}