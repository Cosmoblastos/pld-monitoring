#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SEALEVELPRESSURE_HPA (1013.25)

const char * packetsFilePath = "/packets.txt";

#define PIN_SDL_1 26
#define PIN_SCL_1 25
#define PIN_SDL_2 2
#define PIN_SCL_2 4

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);
//I2C instances

Adafruit_BME280 bme;
Adafruit_BME280 bme2;
Adafruit_MPU6050 mpu;
String packet;

void appendFile(fs::FS &fs, const char * path, const char * message) {
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (!file.print(message)) {
        Serial.println("Append failed");
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

int setupSDCard () {
  Serial.println("Mounting SD Card");

  if (!SD.begin(5)) {
    Serial.println("Card Mount Failed");
    while (1) delay(10);
    return 0;
  }

  //Serial.printf("Configuring file: %s\n", packetsFilePath);
  //writeFile(SD, packetsFilePath, "");

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
      return 0;
  }

  return 1;
}

int setupBMESensor() {
    Serial.println(F("BME280 SETUP"));

    unsigned status;
    
    //BME SETUP
    status = bme.begin(0x76, &I2C_1);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); 
        Serial.println(bme.sensorID(),16);
        while (1) delay(10);
        return 0;
    }

    Serial.println("BME configured");
    return 1;
}

int setupBMESensor2() {
    Serial.println(F("BME280 2 SETUP"));

    unsigned status;
    
    //BME SETUP
    status = bme2.begin(0x76, &I2C_2);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); 
        Serial.println(bme2.sensorID(),16);
        while (1) delay(10);
        return 0;
    }

    Serial.println("BME 2 configured");
    return 1;
}

int setupMPUSensor() {
  Serial.println("Setting up MPU-6050");
  if (!mpu.begin(0x68, &I2C_1)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
    return 0;
  }

  Serial.println("MPU configured");
  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  return 1;
}

void setup() {
    Serial.begin(115200);
    while(!Serial);

    SPI.begin();
    I2C_1.begin(PIN_SDL_1, PIN_SCL_1);
    I2C_2.begin(PIN_SDL_2, PIN_SCL_2);
    pinMode(27, OUTPUT);

    if (setupSDCard() && setupBMESensor() && setupBMESensor2() && setupMPUSensor()) {
      digitalWrite(27, HIGH);
      Serial.println("-- SETUP FINISHED --");
    } else {
      Serial.println("-- SETUP FAILED --");
      return;
    }
}


void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  packet.concat(bme.readTemperature());
  packet.concat(",");
  packet.concat(bme.readPressure() / 100.0F);
  packet.concat(",");
  packet.concat(bme.readAltitude(SEALEVELPRESSURE_HPA));
  packet.concat(",");
  packet.concat(bme.readHumidity());
  packet.concat(",");
  packet.concat(bme2.readTemperature());
  packet.concat(",");
  packet.concat(bme2.readPressure() / 100.0F);
  packet.concat(",");
  packet.concat(bme2.readAltitude(SEALEVELPRESSURE_HPA));
  packet.concat(",");
  packet.concat(bme2.readHumidity());
  packet.concat(",");
  packet.concat(a.acceleration.x);
  packet.concat(",");
  packet.concat(a.acceleration.y);
  packet.concat(",");
  packet.concat(a.acceleration.z);
  packet.concat(",");
  packet.concat(g.gyro.x);
  packet.concat(",");
  packet.concat(g.gyro.y);
  packet.concat(",");
  packet.concat(g.gyro.z);
  packet.concat('\n');

  Serial.print(packet);
  appendFile(SD, packetsFilePath, packet.c_str());

  packet = "";
  delay(300);
}