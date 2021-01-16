#include <Wire.h>

bool on = false;

const int MPU_address = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

void setup()
{
  // Interupt routine setup
  TCCR1A = 0; // Reset to 0
  TCCR1B = 0;

  TIMSK1 |= (1 << OCIE1A); // Pin compare, enable interrupt
  TCCR1B |= (1 << WGM12);  // Clear Timer on Compare mode
  TCCR1B |= (1 << CS12);   // Prescaler : 8
  OCR1A = 20;              // Output Compare Flag

  pinMode(13, OUTPUT);

  // Initialize MPU
  Wire.begin();
  Wire.beginTransmission(MPU_address);
  Wire.write(0x6B);
  Wire.write(0x00); // Sleep to 0 -> wakeup
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_address);
  Wire.write(0x1C);
  Wire.write(0x10); // +/- 8g full scale range
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_address);
  Wire.write(0x1B);
  Wire.write(0x10); // +/- 1000°/s full scale range
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_address);
  Wire.write(0x1A); 
  //TODO: setup the value
  Wire.write(0x06); // Setup a low-pass filter to accel and gyro
  Wire.endTransmission();
}

void loop()
{
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address, 6, true);
  float AccLsbSensitivity = 4096.; // Value found in the datasheet, LSB/g
  AccX = (Wire.read() << 8 | Wire.read()) / AccLsbSensitivity; 
  AccY = (Wire.read() << 8 | Wire.read()) / AccLsbSensitivity;
  AccZ = (Wire.read() << 8 | Wire.read()) / AccLsbSensitivity;

  Wire.beginTransmission(MPU_address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address, 6, true);
  float GyroLsbSensitivity = 32.8; // Value found in the datasheet, LSB/(°/s)
  GyroX = (Wire.read() << 8 | Wire.read()) / GyroLsbSensitivity; 
  GyroY = (Wire.read() << 8 | Wire.read()) / GyroLsbSensitivity;
  GyroZ = (Wire.read() << 8 | Wire.read()) / GyroLsbSensitivity;

  delay(1000);
}

ISR(TIMER1_COMPA_vect)
{ // Interrupt routine
  on = on ^ true;
  digitalWrite(13, on); // Test
}
