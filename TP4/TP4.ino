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
int samples=500;

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
  Serial.println();

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

  calc_error();

  Serial.println("Parámetros del filtro complementario");
  Serial.println("alpha\tbeta");
  Serial.print(alpha);
  Serial.print("\t");
  Serial.print(beta);
  Serial.println("\t");

  Serial.println("");
  delay(2000);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

/*
  aX = a.acceleration.x - OFFSET_A_X;
  aY = a.acceleration.y - OFFSET_A_Y;
  aZ = a.acceleration.z - OFFSET_A_Z;

  wX = g.gyro.x - OFFSET_W_X;
  wY = g.gyro.y - OFFSET_W_Y;
  wZ = g.gyro.z - OFFSET_W_Z;
*/

  // Offsets automáticos
  aX = a.acceleration.x - eaX;
  aY = a.acceleration.y - eaY;
  aZ = a.acceleration.z - eaZ;

  wX = g.gyro.x - ewX;
  wY = g.gyro.y - ewY;
  wZ = g.gyro.z - ewZ;

  // Salida del filtro complementario
  oX = alpha*(oX+dt*wX) + beta*aX;
  oY = alpha*(oY+dt*wY) + beta*aY;
  oZ = alpha*(oZ+dt*wZ) + beta*aZ;

  pitch= atan2(oY,oZ);
  roll = atan2(oX,oZ);

  /* Print out the values */
  Serial.print(aX);
  Serial.print("\t");
  Serial.print(oX);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(aY);
  Serial.print("\t");
  Serial.print(oY);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(-15);
  Serial.print("\t");
  Serial.print(15);
  Serial.println("");

  delay(10);
}

void calc_error()
{
  Serial.println("Calculando offsets");
  eaX = 0.0;
  eaY = 0.0;
  eaZ = 0.0;

  ewX = 0.0;
  ewY = 0.0;
  ewZ = 0.0;
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

  // Serial.print(eaX);
  // Serial.print("\t");
  // Serial.print(eaY);
  // Serial.print("\t");
  // Serial.print(eaZ);
  // Serial.print("\t");

  // Serial.print(ewX);
  // Serial.print("\t");
  // Serial.print(ewY);
  // Serial.print("\t");
  // Serial.print(ewZ);
  // Serial.print("\t");

  // Serial.println("");

    i++;
    delay(10);
  }
  eaX = eaX/samples;
  eaY = eaY/samples;
  eaZ = eaZ/samples - 9.81;

  ewX = ewX/samples;
  ewY = ewY/samples;
  ewZ = ewZ/samples;
  
  Serial.println("Offsets calculados:");
  Serial.println("a.x\ta.y\ta.z\tg.x\tg.y\tg.z");

  Serial.print(eaX);
  Serial.print("\t");
  Serial.print(eaY);
  Serial.print("\t");
  Serial.print(eaZ);
  Serial.print("\t");

  Serial.print(ewX);
  Serial.print("\t");
  Serial.print(ewY);
  Serial.print("\t");
  Serial.print(ewZ);
  Serial.print("\t");

  Serial.println("");

  delay(2000);
}


