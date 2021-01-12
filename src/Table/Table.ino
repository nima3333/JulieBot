#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <DuePWM.h>

// ---------------------------------------------------------------------------
// --------------------- MPU650 variables ------------------------------------
// ---------------------------------------------------------------------------

#define YAW 0
#define PITCH 1
#define ROLL 2
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // Indicates whether MPU interrupt pin has gone high

// ---------------------------------------------------------------------------
// -------------------------PID variables-------------------------------------
// ---------------------------------------------------------------------------
float Kp = 1000;
float Ki = 0;
float Kd = 0;
float iTerm;
float lastTime = 0;
float maxPID = 6000;
float minPID = 10;
float oldCurrent = 0;

// ---------------------------------------------------------------------------
// -------------------------Pins variables------------------------------------
// ---------------------------------------------------------------------------
const int dirA = 6;
const int dirB = 8;
const int stepA = 7;
const int stepB = 9;

// ---------------------------------------------------------------------------
// -------------------------PID Controller------------------------------------
// ---------------------------------------------------------------------------
float PID(float current_angle, float target_angle = 0)
{
    float thisTime = millis();
    float dT = thisTime - lastTime;
    lastTime = thisTime;

    // Calculate error between target and current values
    float error = target_angle - current_angle;
    float pid = 0;

    // Calculate the integral and derivative terms
    iTerm += error * dT;
    float dTerm = (current_angle - oldCurrent) / dT;

    // Set old variable to equal new ones
    oldCurrent = current_angle;

    // Multiply each term by its constant, and add it all up
    pid = (error * Kp) + (iTerm * Ki) - (dTerm * Kd);

    // ------ Saturation ---------------
    // Limit PID value to maximum values
    if (maxPID > 0 && pid > maxPID)
        pid = maxPID;
    else if (maxPID > 0 && pid < -maxPID)
        pid = -maxPID;

    // Limit PID value to maximum values
    if (pid > 0 && pid < minPID)
        pid = minPID;
    else if (pid < 0 && pid > -minPID)
        pid = -minPID;

    return pid;
}

// ---------------------------------------------------------------------------
// -------------------------Motor Controller----------------------------------
// ---------------------------------------------------------------------------
void MotorControl(float pid)
{
    int sens = 1;
    if (pid < 0)
    {
        sens = !sens;
        pid = -pid;
    }
    digitalWrite(dirA, !sens);
    digitalWrite(dirB, sens);
    //TODO
}

void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{

    // --------------------MPU6050 Initialisation---------------------------------

    Wire.begin();
    Wire.setClock(400000L);
    Serial.begin(57600);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // Verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // MPU calibration: set YOUR offsets here.
    mpu.setXAccelOffset(200);
    mpu.setYAccelOffset(-6034);
    mpu.setZAccelOffset(4766);
    mpu.setXGyroOffset(68);
    mpu.setYGyroOffset(-92);
    mpu.setZGyroOffset(188);

    // Returns 0 if it worked
    if (devStatus == 0)
    {
        // Turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0 : #pin2)..."));
        attachInterrupt(12, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop()
{
    // If programming failed, don't try to do anything
    if (!dmpReady)
    {
        return;
    }

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // Do nothing...
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // Wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }

        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Convert Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //TODO: correct
        MotorControl(Kp * ypr[PITCH]);
    }
}
