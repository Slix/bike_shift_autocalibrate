// Demo auto calibration process
// By Peter Kowalczyk
// Setup code by Matthew Potok

#include <Wire.h>
#include <Servo.h>

// ITG-3200 registers
const char WHO_AM_I = 0x00;
const char SMPLRT_DIV= 0x15;
const char DLPF_FS = 0x16;
const char GYRO_XOUT_H = 0x1D;
const char GYRO_XOUT_L = 0x1E;
const char GYRO_YOUT_H = 0x1F;
const char GYRO_YOUT_L = 0x20;
const char GYRO_ZOUT_H = 0x21;
const char GYRO_ZOUT_L = 0x22;

// ITG-3200 settings
const char DLPF_CFG_0 = (1<<0);
const char DLPF_CFG_1 = (1<<1);
const char DLPF_CFG_2 = (1<<2);
const char DLPF_FS_SEL_0 = (1<<3);
const char DLPF_FS_SEL_1 = (1<<4);

// ITG-3200 VDD pin I2C address
const char ITG_ADDRESS = 0x69;

// Servo
Servo der_servo;
// ?? Is this a pin number?
const int SERVO_ATTACH_NUM = 9;
const int SERVO_FEEDBACK_PIN = A0;

void setup() {
  // For serial console
  Serial.begin(9600);
  // For I2C (gyro)
  Wire.begin();

  configure_gyro();

  // Initialize servo
  der_servo.attach(SERVO_ATTACH_NUM);
  // Put servo feedback in analog
  pinMode(SERVO_FEEDBACK_PIN, INPUT);
}

void configure_gyro() {
  // Gyroscope scale +/-2000 degrees per second (default)
  itgWrite(ITG_ADDRESS, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));

  // Sample rate 1KHz (this should match up with "0")
  itgWrite(ITG_ADDRESS, SMPLRT_DIV, 0);
}

void loop() {
  // Rate of rotation of derailleur servo
  const float ROTATE_DEG_PER_SEC = 3.0f;
  // Threshold in deg/sec. Beyond this threshold, a shift is detected
  const float SHIFT_DETECT_THRESHOLD = 195.6525;
  // During a shift, the gyro threshold will be hit multiple times a second
  // Ignore shift data for this long after a shift
  const float SHIFT_COOLDOWN_SEC = 1.0f;

  const int ABSOLUTE_MIN_SERVO_ANGLE = 0;
  const int ABSOLUTE_MAX_SERVO_ANGLE = 140; // Really 180, but we don't want to stress our servo in case of an error

  // Once we haven't seen new maximums for this many seconds, give up finding more gears
  // Servo likely hit physical max angle
  const float MAX_TIMEOUT_THRESHOLD_SECONDS = 2.5f;

  // Max number of gears our program will store
  const int GEAR_ANGLE_ARR_MAX = 15;
  
  Serial.println("Starting calibration...");

  int gearToAngle[GEAR_ANGLE_ARR_MAX];
  int currGear = 0; // 0-indexed in code, presented to user as 1-indexed
  
  // Start by finding smallest mechanical angle of servo
  const float FIND_MIN_WAIT_SEC = 1.5f; // must wait for servo to rotate all the way
  setServoAngle(ABSOLUTE_MIN_SERVO_ANGLE);
  delay(FIND_MIN_WAIT_SEC * 1000);
  // Feedback isn't super-accurate, but it's a good place to start
  int startAngle = getServoFeedback();
  Serial.println("Starting at angle " + String(startAngle));
  // This is a good angle to switch to first gear
  gearToAngle[currGear++] = startAngle;

  unsigned long startTimeMs = millis();
  double lastShift = -999.0; // Equivalent to "never shifted"

  // Keep track of maximums for finding 
  int maxFeedbackAngle = startAngle;
  int maxServoInputAngle = startAngle;
  float lastMaxHitSec = 0.0f;
 
  while (1) {
    float elapsedSec = float(millis() - startTimeMs) / 1000;

    // Rotate gyro continually
    int servoAngle = startAngle + (elapsedSec * ROTATE_DEG_PER_SEC);
    setServoAngle(servoAngle);
    
    float gyroVelocity = getGyroVelocityZ();
    // Make sure cooldown has passed before detecting new shifts
    if ((lastShift + SHIFT_COOLDOWN_SEC) < elapsedSec && abs(gyroVelocity) >= SHIFT_DETECT_THRESHOLD) {
      Serial.println("Shift detected for gear " + String(currGear+1) + " at servo angle "
        + String(servoAngle) + " from gyro velocity " + String(gyroVelocity));
      lastShift = elapsedSec;

      if (currGear >= GEAR_ANGLE_ARR_MAX) {
        Serial.println("Gear ignored; exceeded array storage");
        currGear++;
      } else {
        gearToAngle[currGear++] = servoAngle;
      }
    }

    // If servo isn't moving anymore, we reached its mechanical limit
    //
    // MAX TIMEOUT ALGORITHM:
    // Servo feedback is very volatile. To compensate, we track the maximum ever reached feedback angle
    // Maximum will increase frequently as servo angle increases
    // Once maximum stays constant for x seconds, we know we've reached the max physical angle
    // Note: oddly once this servo reaches its maximum, its feedback drops from 140 to 110. Alg works anyway
    int reportedAngle = getServoFeedback();
    if (reportedAngle > maxFeedbackAngle) {
      maxFeedbackAngle = reportedAngle;
      // It's useful to know the exact servo input we used to reach this maximum
      maxServoInputAngle = servoAngle;
      lastMaxHitSec = elapsedSec;
    }
    // Make sure at least one shift is detected before we give up
    // Feedback servo angle tends to be sticky at low range of angles before first shift
    if (currGear > 1 && elapsedSec - lastMaxHitSec >= MAX_TIMEOUT_THRESHOLD_SECONDS) {
      Serial.println("Detected max servo angle at " + String(maxServoInputAngle));
      break;
    }
    if (servoAngle >= ABSOLUTE_MAX_SERVO_ANGLE) {
      Serial.println("Reached max servo angle " + String(servoAngle) + " without triggering max timeout threshold");
      break;
    }
  }
  Serial.println("Calibration complete! " + String(currGear) + " gears detected.");
  Serial.println("------------------");
  for(int i = 0; i < min(currGear, GEAR_ANGLE_ARR_MAX); i++) {
    // One-index for the user
    Serial.println(String(i+1) + "\t" + gearToAngle[i]);
  }
  Serial.println("------------------");

  delay(1000000000);
}

// Return current gyro Z velocity in degrees/second
float getGyroVelocityZ() {
  int rawZ = readZ();
  if (rawZ == -4352) {
    Serial.println("POSSIBLE GYRO READ ERROR, FATAL IF REPEATING CONTINUALLY");
    return 0.0f;
  }
  // Scale to degrees/second by a ITF-3200 constant
  return rawZ / 14.375f;
}

// Set angle of derailleur servo
void setServoAngle(int angle) {
  der_servo.write(angle);
}

// Get angle servo thinks it is at
int getServoFeedback() {
  // TODO: calibrate this to servo
  int raw = analogRead(SERVO_FEEDBACK_PIN);
  return map(raw, 106, 461, 0, 180);
}

/** 
 * From SparkFun hookup guide 
 */

//This function will write a value to a register on the itg-3200.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be written to.
//  char data: The value to be written to the specified register.
void itgWrite(char address, char registerAddress, char data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(address);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}

//This function will read the data from a specified register on the ITG-3200 and return the value.
//Parameters:
//  char address: The I2C address of the sensor. For the ITG-3200 breakout the address is 0x69.
//  char registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char itgRead(char address, char registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();
  
  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int xRate = readX();
int readX(void)
{
  int data=0;
  data = itgRead(ITG_ADDRESS, GYRO_XOUT_H)<<8;
  data |= itgRead(ITG_ADDRESS, GYRO_XOUT_L);  
  
  return data;
}

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int yRate = readY();
int readY(void)
{
  int data=0;
  data = itgRead(ITG_ADDRESS, GYRO_YOUT_H)<<8;
  data |= itgRead(ITG_ADDRESS, GYRO_YOUT_L);  
  
  return data;
}

//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int zRate = readZ();
int readZ(void)
{
  int data=0;
  data = itgRead(ITG_ADDRESS, GYRO_ZOUT_H)<<8;
  data |= itgRead(ITG_ADDRESS, GYRO_ZOUT_L);  
  
  return data;
}
