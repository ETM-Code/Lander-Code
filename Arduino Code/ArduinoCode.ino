// FOR CALIBRATION: PLACE IT FLAT - 1G, PLACE IT FLAT UPSIDE DOWN - -1G

//===========================================================================
// Import necessary libraries
//===========================================================================

#include <SoftwareSerial.h>  //Permits the use of digital pins as Serial Pins
#include <Wire.h>  //Permits interaction with devices using the I2C protcol. Used for the BMP280 Environmental. Provides functions used such as:
                   //begin(), write(), available(), read()
#include <Adafruit_BMP280.h>  //Library specifically for the BMP280 Environmental Sensor, allowing easy interaction
#include <Adafruit_Sensor.h>  //Supports the Adafruit_BMP280 library and allows standard interaction with Adafruit Sensors
#include <SPI.h>  //Not used for any particular device, but maintained for compatibility
#include <math.h>  //Used for advanced math operations - Unused, but kept for flexibility

//===========================================================================
// Constants - These never change
//===========================================================================

// N.B. Every I2C device has a unique address/chip ID
#define I2CADDR (0x76)
#define CHIPID (0x60)  // 0x60 for BME and 0x58 for BMP.

// Moving Average Filter / Buffer / Track values
#define WINDOW_SIZE 5

const int ForceMode =
    0;  // Set to 1 to force the arduino to use a particular mode
const int ForceModeMode =
    3;  // Change number to set the mode that is being forced

#define GROUND_PRESSURE (1013.25)
float g = 9.81;

//===========================================================================
// Initialising variables and peripherals
//===========================================================================

// HC-12 Wireless Transceiver
SoftwareSerial HC12(7, 6);  // HC-12 TX Pin, HC-12 RX Pin

// Bosch Environmental Sensor (BME/BMP280)
Adafruit_BMP280 env_sensor;  // use I2C interface
Adafruit_Sensor *env_temp = env_sensor.getTemperatureSensor();
Adafruit_Sensor *env_pressure = env_sensor.getPressureSensor();

// time variables
unsigned long time_start = 0, time_prev = 0, time_diff = 0,
              pre_launch_last_transmit_time = 0, landed_last_transmit_time = 0;

// mode variable
int prev_mode = 0, mode = 0;

float floatArray[7] = {0};

// Custom Variables - Added by Student
float AccelBufferX[WINDOW_SIZE] = {0};
float AccelBufferY[WINDOW_SIZE] = {0};
float AccelBufferZ[WINDOW_SIZE] = {0};
int currentBookmarkA = 0;
float tempBuffer[WINDOW_SIZE] = {0};
float pressBuffer[WINDOW_SIZE] = {0};
int currentBookmarkE = 0;

float calibPosX = 981;
float calibNegX = -981;
float calib0X = 0;
float calibPosY = 981;
float calibNegY = -981;
float calib0Y = 0;
float calibPosZ = 981;
float calibNegZ = -981;
float calib0Z = 0;

float oldCalibPosX = 410;
float oldCalibNegX = 274;
float oldCalib0X = 335;
float oldCalibPosY = 404;
float oldCalibNegY = 268;
float oldCalib0Y = 341;
float oldCalibPosZ = 408;
float oldCalibNegZ = 272;
float oldCalib0Z = 339;

float slopeNegX = 0;
float slopePosX = 0;
float slopeNegY = 0;
float slopePosY = 0;
float slopeNegZ = 0;
float slopePosZ = 0;

String inputString = "";

int calibNum = 1;
int beginner = 0;
unsigned long subtractTime = 0;
int checkCount = 0;

float AccelNowX = 0;
float AccelNowY = 0;
float AccelNowZ = 0;
float AccelNowAbsolute = 0;

float SumX = 0;
float SumY = 0;
float SumZ = 0;
float sumPress = 0;
float sumTemp = 0;

float TempNow = 0;
float PressNow = 0;
float pastPress = 0;

uint16_t timeChange = 0xFFF;
bool modeChange = false;
uint16_t rawAy = 0x3FF;      // 10 bits
int8_t rawAx = 0xFF;         // 8 bits
int8_t rawAz = 0xFF;         // 8 bits
int16_t PressDiff = 0x3FFF;  // 14 bits
int16_t tempChange = 0x3FF;  // 10 bits
uint16_t terminator =
    0xFFFF;  // 8 bits, could be used as a packet end or separator
float initialTemp = -2000;

float dataToSend[9] = {0.00};
int dataToSendSize = 7;

int landable = 0;

void setup() {
  // Code has started, start timestamp.
  time_start = millis();

  // Setup serial connections to the HC-12/ PC
  // Start serial streams between Arduino and HC-12/ PC
  HC12.begin(9600);
  Serial.begin(9600);

  // Give console time to get running
  while (!Serial) {
    Serial.println(F("Waiting for serial connection..."));
  }

  // Setup for the Environmental Sensor
  if (!env_sensor.begin(I2CADDR, CHIPID)) {
    Serial.println(
        F("Could not find a valid BME/BMP280 (Environmental) sensor, check "
          "CHIPID/libraries/wiring!"));
    while (1) delay(10);
  }

  // Default Adafruit_BMP280 settings from datasheet/
  env_sensor.setSampling(
      Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // initialise times
  pre_launch_last_transmit_time = time_start;
  landed_last_transmit_time = time_start;
}

float mapAccelX(float rawX) {
  float realAcceleration;
  if (rawX >= calib0X) {
    // If the rawValue is between 0 and g, use the slope for that segment
    realAcceleration = slopePosX * (rawX - calib0X);
  } else {
    // If the rawValue is between -g and 0, use the slope for that segment
    realAcceleration = slopeNegX * (rawX - calib0X);
  }

  return realAcceleration;
}

float mapAccelY(float rawY) {
  float realAcceleration;

  if (rawY >= calib0Y) {
    // If the rawValue is between 0 and g, use the slope for that segment
    realAcceleration = slopePosY * (rawY - calib0Y);

  } else {
    // If the rawValue is between -g and 0, use the slope for that segment
    realAcceleration = slopeNegY * (rawY - calib0Y);
  }
  return realAcceleration;
}

float mapAccelZ(float rawZ) {
  float realAcceleration;

  if (rawZ >= calib0Z) {
    // If the rawValue is between 0 and g, use the slope for that segment
    realAcceleration = slopePosZ * (rawZ - calib0Z);
  } else {
    // If the rawValue is between -g and 0, use the slope for that segment
    realAcceleration = slopeNegZ * (rawZ - calib0Z);
  }
  return realAcceleration;
}
void smoothAccelReading(
    float *a_x /*Pointer to the input variable, which allows it to be changed
                  without needing to return a value or use global variables*/
    ,
    float *a_y, float *a_z) {
  float SumX = 0;
  float SumY = 0;
  float SumZ = 0;

  AccelBufferX[currentBookmarkA] =
      *a_x;  // Updates one of the data values to the new value
  AccelBufferY[currentBookmarkA] = *a_y;
  AccelBufferZ[currentBookmarkA] = *a_z;
  currentBookmarkA++;  // Cycles which value is getting updated

  if (currentBookmarkA == WINDOW_SIZE) {
    currentBookmarkA = 0;
  }
  for (int i = 0; i < WINDOW_SIZE; i++) {  // Gets the sum of the buffers
    SumX += AccelBufferX[i];
    SumY += AccelBufferY[i];
    SumZ += AccelBufferZ[i];
  }
  *a_x = SumX / WINDOW_SIZE;  // Performs the averaging calculation
  *a_y = SumY / WINDOW_SIZE;
  *a_z = SumZ / WINDOW_SIZE;
}

void smoothEnvReading(float *temp, float *press) {
  float sumTemp = 0;
  float sumPress = 0;
  tempBuffer[currentBookmarkE] = *temp;
  pressBuffer[currentBookmarkE] = *press;
  currentBookmarkE++;
  if (currentBookmarkE == WINDOW_SIZE) {
    currentBookmarkE = 0;
  }

  for (int i = 0; i < WINDOW_SIZE; i++) {
    sumTemp += tempBuffer[i];
    sumPress += pressBuffer[i];
  }

  *temp = sumTemp / WINDOW_SIZE;
  *press = sumPress / WINDOW_SIZE;
}

void readFromAccelerometer() {
  short Xraw = analogRead(A1);
  short Yraw = analogRead(A2);
  short Zraw = analogRead(A3);

  // Used for transmission
  rawAx = (int8_t)(Xraw - calib0X);
  rawAy = (uint16_t)(Yraw);
  rawAz = (int8_t)(Zraw - calib0Z);

  // Used for mode calculation
  AccelNowX = mapAccelX((float)Xraw);
  AccelNowY = mapAccelY((float)Yraw);
  AccelNowZ = mapAccelZ((float)Zraw);
  smoothAccelReading(&AccelNowX, &AccelNowY, &AccelNowZ);
}

void readEnvironmental() {
  // Read Environmental Sensor
  sensors_event_t temp_event, pressure_event;

  env_temp->getEvent(&temp_event);
  env_pressure->getEvent(&pressure_event);

  if (initialTemp == -2000) {
    initialTemp = temp_event.temperature;
  }

  // Store value into variables: temperature (Celcius), pressure (hPa)
  TempNow = temp_event.temperature;
  PressNow = pressure_event.pressure;
  smoothEnvReading(&TempNow, &PressNow);
  tempChange = (int16_t)((TempNow - initialTemp) * 1000);
  PressDiff = (int16_t)((pressure_event.pressure - GROUND_PRESSURE) * 1000);

  if ((PressNow - pastPress) < 0.05) {
    landable =
        1;  // This variable decides whether the mode can be set to "LANDED"
  } else {
    landable = 0;
  }
  pastPress = PressNow;
}

int detectMode(float acceleration) {
  if (!ForceMode /*Only activate the function if we're not currently forcing a specific mode for testing*/) {
    // MODE CODES:
    // 0 - UNASSIGNED
    // 1 - PRE-LAUNCH
    // 2 - ASCENDING
    // 3 - DESCENDING
    // 4 - LANDED
    if (mode == 0 && acceleration < (g * 1.1)) {
      mode = 1;
    } else if (mode == 1 && acceleration > (g * 1.2)) {
      mode = 2;
      prev_mode = 1;
      modeChange = true;
    } else if (mode == 2 && acceleration < (g * 0.8)) {
      mode = 3;
      prev_mode = 2;
      modeChange = true;
    } else if (mode == 3 && acceleration > (g * 0.95) && (landable == 1)) {
      mode = 4;
      prev_mode = 3;
      modeChange = true;
    }
    // else {HC12.println("No Mode Change");}
  }

  else {
    mode = ForceModeMode;
  }
}

void transmit(String dataSending) {
  // Send to HC-12 for wireless transmission
  HC12.println(dataSending);
}

void provideSlopes() {
  slopeNegX = g / (calib0X - calibNegX);
  slopePosX = g / (calibPosX - calib0X);

  slopeNegY = g / (calib0Z - calibNegY);
  slopePosY = g / (calibPosZ - calib0Y);

  slopeNegZ = g / (calib0Z - calibNegZ);
  slopePosZ = g / (calibPosZ - calib0Z);
}

void Calibrate(int calibMode) {
  float xValue = analogRead(A1);
  float yValue = analogRead(A2);
  float zValue = analogRead(A3);

  if (calibMode == 1) {
    calibPosZ = zValue;
    calib0X = xValue;
    calib0Y = yValue;
    transmit("C");
    Serial.println("Received");
  } else if (calibMode == 2) {
    calibNegZ = zValue;
    calib0X += xValue;
    calib0Y += yValue;
    transmit("C");
    Serial.println("Received");
  }
  if (calibMode == 3) {
    calibPosX = xValue;
    calib0Z = zValue;
    calib0Y += yValue;
    transmit("C");
    Serial.println("Received");
  } else if (calibMode == 4) {
    calibNegX = xValue;
    calib0Z += zValue;
    calib0Y += yValue;
    transmit("C");
    Serial.println("Received");
  } else if (calibMode == 5) {
    calibPosY = yValue;
    calib0X += xValue;
    calib0Z += zValue;
    transmit("C");
    Serial.println("Received");
  } else if (calibMode == 6) {
    calibNegY = yValue;
    calib0X += xValue;
    calib0Z += zValue;
    transmit("C");
    Serial.println("Received");
  } else if (calibMode == 7) {
    calib0X = calib0X / 4;
    calib0Y = calib0Y / 4;
    calib0Z = calib0Z / 4;
    provideSlopes();
    delay(100);

    Serial.println("Received");
    String transmitter = "CALIBVALUES";
    transmitter = transmitter + "," + slopePosX + "," + calib0X + "," +
                  slopeNegX + "," + slopePosY + "," + calib0Y + "," +
                  slopeNegY + "," + slopePosZ + "," + calib0Z + "," +
                  slopePosZ + "," + TempNow;
    transmit(transmitter);
    transmit("C");
    beginner = 1;
  } else if ((calibMode > 7) || (calibMode < 1)) {
    Serial.println("ERROR IN CALIBRATION MODE");
  }
}

unsigned long lastCommandTime =
    0;  // Initialize a global variable to track the last command execution time

String removeSubstring(String original, String toRemove) {
  // Find the start position of the substring to remove
  int startPos = original.indexOf(toRemove);

  // Check if the substring was found
  if (startPos != -1) {
    // Find the end position of the substring
    int endPos = startPos + toRemove.length();

    // Construct the new string without the unwanted substring
    String before = original.substring(0, startPos);
    String after = original.substring(endPos);
    return before + after;
  }

  // If the substring was not found, return the original string
  return original;
}

void useOldCalibrations() {
  calib0X = oldCalib0X;
  calibPosX = oldCalibPosX;
  calibNegX = oldCalibNegX;
  calib0Y = oldCalib0Y;
  calibPosY = oldCalibPosY;
  calibNegY = oldCalibNegY;
  calib0Z = oldCalib0Z;
  calibPosZ = oldCalibPosZ;
  calibNegZ = oldCalibNegZ;
  provideSlopes();
  String transmitter = "CALIBVALUES";
  transmitter = transmitter + "," + slopePosX + "," + calib0X + "," +
                slopeNegX + "," + slopePosY + "," + calib0Y + "," + slopeNegY +
                "," + slopePosZ + "," + calib0Z + "," + slopePosZ + "," +
                initialTemp;
  transmit(transmitter);
  Serial.println("Sending X");
  transmit("X");
  calibNum = 7;
  delay(100);
  beginner = 1;
}

void resetCalibration(unsigned long cTime) {
  Serial.println("RESETTING");
  beginner = 0;
  calibNum = 1;
  transmit("R");
  inputString = "";  // Clear the input string after processing the command
  lastCommandTime = cTime;  // Update the last command time to the current time
}

void receiveInput() {
  unsigned long currentTime = millis();  // Get the current time in milliseconds

  // Check if more than a second (1000 milliseconds) has passed since the last
  // command
  if (currentTime - lastCommandTime > 1000) {
    if (inputString.indexOf("C") !=
        -1) {  // Check if "C" is contained within the inputString
      if (inputString.indexOf(String(calibNum)) != -1) {
        Calibrate(calibNum);
        calibNum++;
        inputString =
            "";  // Clear the input string after processing the command
        lastCommandTime =
            currentTime;  // Update the last command time to the current time
        checkCount = 0;
      } else if (checkCount > 7) {
        resetCalibration(currentTime);
        checkCount = 0;
      } else {
        checkCount++;
      }
    } else if (inputString.indexOf("R") !=
               -1) {  // Check if "RESET" is contained within the inputString
      resetCalibration(currentTime);
    } else if (inputString.indexOf("OLD") != -1) {
      // Serial.println("Old read");
      useOldCalibrations();
    } else if (inputString.length() > 15) {
      inputString = "";
    }
  }
}

void checkInput() {
  if (HC12.available()) {
    char inChar = (char)HC12.read();
    inputString += inChar;
    receiveInput();
    // Serial.println(inputString);
  }
}

void transmitOptimisedBinary() {
  uint8_t dataStream[9] = {0};
  // So, I was originally transmitting the floats as binary, but I realised that
  // for a lot of things I don't need all that data. So I had a look at what I
  // felt I needed and only provided for that
  // Pack the data into the byte stream
  // timeChange: 12 bits - allows for 4 seconds between transmission at 3
  // decimal accuracy
  dataStream[0] = timeChange >> 4;          // Top 8 bits
  dataStream[1] = (timeChange & 0xF) << 4;  // Bottom 4 bits of timeChange

  // Mode: 1 bit - just says if the mode changed or not, rawAy: 10 bits - that's
  // the size of the values provided by the accelerometer
  dataStream[1] |= modeChange << 3;     // 1 bit from modeChange
  dataStream[1] |= (rawAy >> 7) & 0x7;  // Top 3 bits of rawAy
  dataStream[2] = (rawAy & 0x7F) << 1;  // Remaining 7 bits from rawAy

  // rawAx: 8 bits - these values cannot go above or below their gravity
  // calibrations in theory, so this gives the difference from their 0g value
  dataStream[2] |= (rawAx >> 7);        // Top 1 bit of rawAx
  dataStream[3] = (rawAx & 0x7F) << 1;  // Bottom 7 bits of rawAx

  // rawAz: 8 bits
  dataStream[3] |= (rawAz >> 7);        // Top 1 bit of rawAz
  dataStream[4] = (rawAz & 0x7F) << 1;  // Bottom 7 bits of rawAz

  // tempChange: 10 bits - required for 4 degrees change to 3 decimals
  dataStream[4] |= (tempChange >> 8) & 0x1;  // Top 2 bits of tempChange
  dataStream[5] = (tempChange >> 0) & 0xFF;  // Middle 8 bits of tempChange

  // PressDiff: 14 bits - required for 60 hPa change to 3 decimals
  dataStream[5] |= (PressDiff >> 12) & 0x3;  // Top 2 bits of PressDiff
  dataStream[6] = (PressDiff >> 4) & 0xFF;   // Middle 8 bits of PressDiff
  dataStream[7] = (PressDiff & 0xF) << 4;    // Bottom 4 bits of PressDiff

  // Terminator: 12 bits
  dataStream[7] |=
      (terminator >> 8) &
      0xF;  // Shift the terminator right by 8 bits to get the top 4 bits, then
            // mask with 0xF to ensure only the top 4 bits are used.

  // Store the remaining 8 bits of the terminator in dataStream[8]
  dataStream[8] = terminator & 0xFF

                  // Transmit the data stream
                  for (int i = 0; i < sizeof(dataStream); i++) {
    // This just writes the data to the HC12
    HC12.write((byte *)(void *)dataStream[i], sizeof(uint8_t));
  }
}

void transmitBinaryFloats(float *floatArray, size_t numFloats) {
  for (size_t i = 0; i < numFloats; i++) {
    HC12.write((byte *)(void *)&floatArray[i], sizeof(float));
  }

  // byte terminator[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  // HC12.write(terminator, 4);
}

void readPackageAndTransmit(unsigned long time_now) {
  Serial.println(time_now);
  readEnvironmental();
  floatArray[0] = time_now;
  floatArray[1] = (float)mode;
  floatArray[2] = AccelNowX;
  floatArray[3] = AccelNowY;
  floatArray[4] = AccelNowZ;
  floatArray[5] = PressNow;
  floatArray[6] = TempNow;
  transmitBinaryFloats(floatArray, 7);
}

void loop() {
  if (beginner == 0) {
    checkInput();
    subtractTime = millis();
  } else {
    // delay(1000);
    Serial.println(millis());
    unsigned long time_now = millis() - subtractTime;
    readFromAccelerometer();
    detectMode(AccelNowY);
    // Serial.println(AccelNowX);

    switch (mode) {
      case 1:
        if ((time_now - pre_launch_last_transmit_time) > 5000) {
          pre_launch_last_transmit_time = time_now;
          readPackageAndTransmit(time_now);
          checkInput();
        }
        break;

      case 2:
        readPackageAndTransmit(time_now);
        break;

      case 3:
        readPackageAndTransmit(time_now);
        break;

      case 4:
        if ((time_now - landed_last_transmit_time) > 10000) {
          readPackageAndTransmit(time_now);
          checkInput();
        }
        break;

      default:
        // Serial.println("ERROR: No Mode Initialised");
        // HC12.println("ERROR: No Mode Initialised");
        // HC12.print("Acceleration (vertical) Value:   ");
        // HC12.println(AccelNowZ);
        break;
    }
  }
}
