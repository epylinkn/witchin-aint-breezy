#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Servo botServo;
Servo topServo;
int pot1 = A0;
int pot2 = A1;
int statusPin = 3;
int buttonPin = 13;
bool prevButtonState;
int sensorMode = 0;
int serialByte = 0;
int pos = 0;
int topServoAngle = 90;
int botServoAngle = 80;

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float baseYaw;
float prevYaw = 0;


void setup() {
  botServo.attach(9);
  topServo.attach(6);
  pinMode(buttonPin, INPUT);
  pinMode(statusPin, OUTPUT);


  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(38400);
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

   // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      delay(2000);
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void loop() {
  // 1. Button to toggle between accelerometer and potentiometers
  int buttonState = digitalRead(buttonPin);
  if (buttonState != prevButtonState && buttonState == HIGH) {
    sensorMode = (sensorMode + 1) % 3; // 0 - dmp "calibration"; 1 - dmpOn; 2 - potentiometers
  }
  prevButtonState = buttonState;

  if (sensorMode == 0) {
    Serial.println(F("Calibrating!"));

    digitalWrite(statusPin, LOW);
    delay(300);
    digitalWrite(statusPin, HIGH);
    delay(300);

    // get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      baseYaw = ypr[0] * 180/M_PI;  // [-180, 180]
    }

  } else if (sensorMode == 1) {
    digitalWrite(statusPin, HIGH);

    // get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float yaw = ypr[0] * 180/M_PI - baseYaw;  // [-180, 180]
      if (yaw < -180) { yaw += 360; }   // weird lower limit
      if (yaw > 180) { yaw -= 360; }    // weird upper limit


      float tolerance = 5.0;
      if (yaw < prevYaw - tolerance || yaw > prevYaw + tolerance) {
        float botTargetAngle = map(yaw, -180, 180, 0, 180);


        botServo.write(botTargetAngle);
        delay(15);
        prevYaw = yaw;
      }

      Serial.print("yaw:\t");
      Serial.print(yaw);

//      Serial.print("ypr\t");
//      Serial.print(ypr[0] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[1] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[2] * 180/M_PI);
    }

  } else {
    digitalWrite(statusPin, LOW);

    int val1 = analogRead(pot1);
    int val2 = analogRead(pot2);

    Serial.print("val1 \t");
    Serial.print(val1);
    Serial.print("\tval2 \t");
    Serial.print(val2);

    int mappedVal1 = map(val1, 0, 1023, 0, 180);
    int mappedVal2 = map(val2, 0, 1023, 0, 180);

    Serial.print("\tmappedVal1 \t");
    Serial.print(mappedVal1);
    Serial.print("\tmappedVal2 \t");
    Serial.print(mappedVal2);

    botServo.write(mappedVal1);
    topServo.write(mappedVal2);
    delay(15); // waits for the servo to get there
  }

  Serial.println();
}

