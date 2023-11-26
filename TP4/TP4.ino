// Basic demo for accelerometer readings from Adafruit MPU6050

// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <FastTrig.h>

Adafruit_MPU6050 mpu;

float aX, aY, aZ;
float wX, wY, wZ;
float oX, oY, oZ;

float Kvel = M_PI/180;  // Convierto º/s a rad/s
float Kpos = M_PI/180;

float eaX, eaY, eaZ;
float ewX, ewY, ewZ;
int i=0;
int samples=200;

float roll,pitch,yaw;

#define OFFSET_A_X  -0.03
#define OFFSET_A_Y  -0.18
#define OFFSET_A_Z  0.68

#define OFFSET_W_X  -0.05
#define OFFSET_W_Y  -0.02
#define OFFSET_W_Z  -0.00

float dt = 0.0262;
float sT = 0.75;
float alpha = (sT/dt)/(1+sT/dt);
float beta = 1-alpha;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  void calc_error();

  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  aX = a.acceleration.x - OFFSET_A_X;
  aY = a.acceleration.y - OFFSET_A_Y;
  aZ = a.acceleration.z - OFFSET_A_Z;

  wX = g.gyro.x - OFFSET_W_X;
  wY = g.gyro.y - OFFSET_W_Y;
  wZ = g.gyro.z - OFFSET_W_Z;

  oX = alpha*(oX+dt*wX) + beta*aX;
  oY = alpha*(oY+dt*wY) + beta*aY;
  oZ = alpha*(oZ+dt*wZ) + beta*aZ;

  pitch= atan2(oX,oZ);
  roll = atan2(oY,oZ);

  /* Print out the values */
  Serial.print(oX);
  Serial.print(",");
  Serial.print(oY);
  Serial.print(",");
  Serial.print(oZ);
  Serial.print(", ");
  Serial.print(wX);
  Serial.print(",");
  Serial.print(wY);
  Serial.print(",");
  Serial.print(wZ);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.println("");


  delay(10);
}

void calc_error()
{
  eaX = 0;
  eaY = 0;
  eaZ = 0;

  ewX = 0;
  ewY = 0;
  ewZ = 0;
  while(i<samples)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    eaX += a.acceleration.x;
    eaY += a.acceleration.y;
    eaZ += a.acceleration.z;

    ewX += g.gyro.x;
    ewY += g.gyro.y;
    ewZ += g.gyro.z;

    i++;
    delay(10);
  }
  eaX /= samples;
  eaY /= samples;
  eaZ /= samples;

  ewX /= samples;
  ewY /= samples;
  ewZ /= samples;
  
}


