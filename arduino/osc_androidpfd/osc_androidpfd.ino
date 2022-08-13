
#include <Wire.h>

#include <ArduinoOSCWiFi.h>
const char* ssid = "esp-wifi";
const char* pwd = "12345678";
const IPAddress ip(192, 168, 0, 1);
const IPAddress subnet(255, 255, 255, 0);



#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;



void setup()
{
  Serial.begin(115200);

  Wire.begin();
  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP(ssid, pwd);


  Wire.setClock(400000); // Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); // Send data update every 50ms

}

void loop()
{
  // Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float x = myIMU.getQuatI();
    float y = myIMU.getQuatJ();
    float z = myIMU.getQuatK();
    float w = myIMU.getQuatReal();
    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(t0, t1);
    double t2 = 2.0 * (w * y - z * x);
    if (t2 > 1.0)
    {
      t2 = 1.0;
    }
    if (t2 < -1.0)
    {
      t2 = -1.0;
    }
    double pitch = asin(t2);
    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(t3, t4);
    roll *= 57.2957795131;
    pitch *= 57.2957795131;
    yaw *= -57.2957795131;

    yaw -= 90;

    if (yaw < 0)
    {yaw = 360 + yaw;}

    String str = String(roll) + "," + String(pitch) + "," + String(yaw);

    OscWiFi.send("192.168.0.255", 55555, "/data", 0, 0.0, str);
    Serial.println(str);

  }
}
