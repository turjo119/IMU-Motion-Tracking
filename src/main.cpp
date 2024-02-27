// Mahony AHRS for the ICM_20948  S.J. Remington 6/2021
// Requires the Sparkfun ICM_20948 library
// Standard sensor orientation X North (yaw=0), Y West, Z up
// magnetometer Y and Z axes are reflected to reconcile with accelerometer.

// New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc cross Mag) as the orientation reference vectors
// heavily modified from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion program ICM_20948_get_cal_data
//
/*
  Sparkfun ICM_20948
  Hardware setup: This library supports communicating with the
  ICM_20948 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  ICM_20948 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VIN ------------- 5V or 3.3V
   GND ------------- GND

*/
#include <Arduino.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Wire.h>
#include <string>
#include "WiFi.h"
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "batterymonitor.h"

//////////////////////////
// ICM_20948 Library Init //
//////////////////////////
// default settings for accel and magnetometer

#define WIRE_PORT Wire // desired Wire port.
#define AD0_VAL 1      // value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when
// the ADR jumper is closed the value becomes 0

ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The compass will NOT work well or at all if these are not correct

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; //250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {36.5, 65.8, -46.2};

//Accel scale: divide by 16604.0 to normalize
float A_B[3]
{   0.020915,  -0.018836,  -0.030144};

float A_Ainv[3][3]
{ {  1.004752,  0.012259, 0.012313},
  {  0.012259,  1.002487, 0.004673},
  { 0.012313, 0.004673,  0.961464}
};

//Mag scale divide by 369.4 to normalize
float M_B[3]
{ 12.541804,  0.459038, 44.037736};

float M_Ainv[3][3]
{ {  1.061172, 0.016141,  -0.0067047},
  { 0.016141,  0.930478,  -0.002444},
  {  -0.006704,  -0.002444,  1.001498}
};

// local magnetic declination in degrees
float declination = 3.6;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 30 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output

// Function prototypes
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

/* SERVER SETUP*/

// for sensor one =========================================
//Server ip in string array
String serverIP[1] = {
  // "http://192.168.0.109:3000/data", // Home Router
  "http://192.168.90.239:3000/data", // Mobile Hotspot Fahim
  // "http://192.168.220.239:8000/post-esp-data.php", 
//Mobile Hotspot Fahim Docker
};
// Sensor descxription
String sensorName = "TripleAdapt_ICM_20948_One";String sensorLocation = "FESTO";
char sensorNameChar[] = "TripleAdapt_ICM_20948_One";
// =========================================================

// // for sensor two =========================================
// //Server ip in string array
// String serverIP[1] = {
//   // "http://192.168.0.109:3000/data", // Home Router
//   "http://192.168.90.239:3000/dataSecondSensor", // Mobile Hotspot Fahim
//   // "http://192.168.220.239:8000/post-esp-data.php", 
// //Mobile Hotspot Fahim Docker
// };
// // Sensor description
// String sensorName = "TripleAdapt_ICM_20948_Two";String sensorLocation = "FESTO";
// char sensorNameChar[] = "TripleAdapt_ICM_20948_Two";
// // =========================================================

int serverIpIndex = 0;
int serverIpArrayLength = 1;
//Http Response Code
int httpResponseCode = 200;
int httpResponseOkCount = 0;

// for timestamp
struct tm timeinfo;
struct timeval timeinfoInSeconds;
char timestamp[70];
bool timestampReadSuccessful = true;

// global variables for storing sensor data
String sensor, location, q0, q1, q2, q3, ax, ay, az, gravX, gravY, gravZ, magX, magY, magZ, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, charge, action, sensorTimestamp;
int dataCount = 0;

// Wifi
bool wifiConnected = false;
int loopCount = 0;


void setup()
{
  Serial.begin(115200);
  while (!Serial); //wait for connection
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }
  //Builtin LED for Wi-Fi status//
  pinMode(LED_BUILTIN, OUTPUT);

  //Blink for Power/Reset
  for (int i = 0; i < 5; i++){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  }

  /* Connect to Wi-Fi*/
    WiFiManager wm; // Create Wifi Manager
    // wm.resetSettings();// Reset settings every time ESP32 reboots
    bool res;
    wm.setConnectRetries(3);
    res = wm.autoConnect(sensorNameChar, "password"); //password protected AP
    if (!res) {
      Serial.println("Failed to connect");
      ESP.restart();
    }

    Serial.println("Connected to the WiFi Network");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5000);
    digitalWrite(LED_BUILTIN, LOW);
    // -------------------------- end of WiFi connection//

    // Set time via NTP server
    configTime(3600, 0, "pool.ntp.org"); // parameters: time zone in GMT (value in seconds, for example +1 GMT is 3600 seconds), daylight offset (daylight saving time, for 1 hour daylight time saving add 3600 seconds), NTP server (the server from which the time is taken)

}

 

void loop()
{

  static int loop_counter = 0; //sample & update loop counter
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data


  // Update the sensor values whenever new data is available
  if ( imu.dataReady() ) {

    imu.getAGMT();

    loop_counter++;
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

    Mxyz[1] = -Mxyz[1]; //reflect Y and Z
    Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint > PRINT_SPEED) {

      // Define Tait-Bryan angles. Strictly valid only for approximately level movement
      
      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order.
      //
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
            
      // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
      // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
      
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      // Print  gyro,acceleration and mag data
      Serial.print("Accel, Gyro, Mag: ");
      Serial.print(Axyz[0], 2);
      Serial.print(", ");
      Serial.print(Axyz[1], 2);
      Serial.print(", ");
      Serial.print(Axyz[2], 2);
      Serial.print(", ");
      Serial.print(Gxyz[0], 2);
      Serial.print(", ");
      Serial.print(Gxyz[1], 2);
      Serial.print(", ");
      Serial.print(Gxyz[2], 2);
      Serial.print(", ");
      Serial.print(Mxyz[0], 2);
      Serial.print(", ");
      Serial.print(Mxyz[1], 2);
      Serial.print(", ");
      Serial.print(Mxyz[2], 2);
      Serial.println();
    
      // Print Quaternion data
      Serial.print("q0, q1, q2, q3: ");
      Serial.print(q[0], 2);
      Serial.print(", ");
      Serial.print(q[1], 2);
      Serial.print(", ");
      Serial.print(q[2], 2);
      Serial.print(", ");
      Serial.print(q[3], 2);
      Serial.println();

      // Print Euler angles in degrees
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      //          Serial.print(", ");  //prints 49 in 300 ms (~160 Hz) with 8 MHz ATmega328
      //          Serial.print(loop_counter);  //sample & update loops per print interval
      loop_counter = 0;
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }

    //read battery level
    float battery_level = batterymonitor();

    //get current timestamp
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      timestampReadSuccessful = false;
    }
    else{
      gettimeofday(&timeinfoInSeconds, NULL); //get current time in seconds
      strftime(timestamp, sizeof(timestamp)-7, "%Y-%m-%dT%H:%M:%S", &timeinfo); //convert time to string
      sprintf(timestamp + strlen(timestamp), ".%03ldZ", timeinfoInSeconds.tv_usec / 1000); //add milliseconds to string
    }

    // Store data in global variables ------------------------------//
    sensor.concat(sensorName);
    sensor.concat("|");
    location.concat(sensorLocation);
    location.concat("|");
    q0.concat(q[0]);
    q0.concat("|");
    q1.concat(q[1]);
    q1.concat("|");
    q2.concat(q[2]);
    q2.concat("|");
    q3.concat(q[3]);
    q3.concat("|");
    // dummy linear acceleration values
    ax.concat(1.0);
    ax.concat("|");
    ay.concat(1.0);
    ay.concat("|");
    az.concat(1.0);
    az.concat("|");
    // gravX.concat(grav_x);
    // gravX.concat("|");
    // gravY.concat(grav_y);
    // gravY.concat("|");
    // gravZ.concat(grav_z);
    // gravZ.concat("|");
    magX.concat(Mxyz[0]);
    magX.concat("|");
    magY.concat(Mxyz[1]);
    magY.concat("|");
    magZ.concat(Mxyz[2]);
    magZ.concat("|");
    accelX.concat(Axyz[0]);
    accelX.concat("|");
    accelY.concat(Axyz[1]);
    accelY.concat("|");
    accelZ.concat(Axyz[2]);
    accelZ.concat("|");
    gyroX.concat(Gxyz[0]);
    gyroX.concat("|");
    gyroY.concat(Gxyz[1]);
    gyroY.concat("|");
    gyroZ.concat(Gxyz[2]);
    gyroZ.concat("|");
    charge.concat(battery_level);
    charge.concat("|");
    //action.concat(activity);
    //action.concat("|");
    if(timestampReadSuccessful){
      sensorTimestamp.concat(timestamp);
      sensorTimestamp.concat("|");

    }
    else{
      sensorTimestamp.concat("NA");
      sensorTimestamp.concat("|");
    }
    dataCount = dataCount + 1;
    // Store data in global variables finished ------------------------------//

    // Sending data through wifi ---------------------------------//
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      wifiConnected = true;
      WiFiClient client;
      HTTPClient http;

      // Traverse through IP array until correct ip for server is found
      if (httpResponseCode == 200){
        http.begin(client, serverIP[serverIpIndex]);
        httpResponseOkCount = httpResponseOkCount + 1;
      }
      else{
        if(serverIpIndex < (serverIpArrayLength-1)){
          serverIpIndex = serverIpIndex + 1;
          http.begin(client, serverIP[serverIpIndex]);
        }
        else{
          serverIpIndex = 0;
          http.begin(client, serverIP[serverIpIndex]);
        }
        
      }

      if(dataCount == 10){
        dataCount = 0;

        // Specify content-type header
        http.addHeader("Content-Type", "application/json");

        // Prepare JSON document
        StaticJsonDocument<2048> doc;
        // doc["sensor"] = sensor;
        // doc["location"] = location;
        doc["q0"] = q0;
        doc["q1"] = q1;
        doc["q2"] = q2;
        doc["q3"] = q3;
        doc["ax"] = ax;
        doc["ay"] = ay;
        doc["az"] = az;
        // doc["gravX"] = gravX;
        // doc["gravY"] = gravY;
        // doc["gravZ"] = gravZ;
        doc["magX"] = magX;
        doc["magY"] = magY;
        doc["magZ"] = magZ;
        doc["accelX"] = accelX;
        doc["accelY"] = accelY;
        doc["accelZ"] = accelZ;
        doc["gyroX"] = gyroX;
        doc["gyroY"] = gyroY;
        doc["gyroZ"] = gyroZ;
        // doc["charge"] = charge;
        // doc["activity"] = action;
        doc["sensorTimestamp"] = sensorTimestamp;

        // Serialize JSON document
        String httpRequestData;
        serializeJson(doc, httpRequestData);

        // Send HTTP POST request
        httpResponseCode = http.POST(httpRequestData);
        if (httpResponseCode>0) {
          Serial.print("HTTP Response code: ");
          Serial.println(httpResponseCode);
          Serial.println(http.getString());
        }
        else {
          Serial.print("Error code: ");
          Serial.println(httpResponseCode);
          Serial.print("Could not connect to server");
          Serial.println(serverIP[serverIpIndex]);
          Serial.println(WiFi.SSID());
          // Blink LED to indicate error
          digitalWrite(LED_BUILTIN, HIGH);
          delay(1000);
          digitalWrite(LED_BUILTIN, LOW);
        }

      // clear global variables
        sensor.clear();
        location.clear();
        q0.clear();
        q1.clear();
        q2.clear();
        q3.clear();
        ax.clear();
        ay.clear();
        az.clear();
        gravX.clear();
        gravY.clear();
        gravZ.clear();
        magX.clear();
        magY.clear();
        magZ.clear();
        accelX.clear();
        accelY.clear();
        accelZ.clear();
        gyroX.clear();
        gyroY.clear();
        gyroZ.clear();
        charge.clear();
        action.clear();
        sensorTimestamp.clear();
        doc.clear();
     
      }
      http.end(); // Free resources
    }

  else
    {
      wifiConnected = false;
      Serial.println("WiFi Disconnected");
    }
    // ---------------------------------------- end of sending data through wifi//


  }

  else
  {
    Serial.println("ICM20948 not found");
    // Blink LED to indicate error
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
  }

}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];

  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}