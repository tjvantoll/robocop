#include <Wire.h>
#include <Notecard.h>
#include <Adafruit_BME680.h>
#include <SparkFun_VL53L1X.h>
#include <Adafruit_LSM9DS1.h>
#include <SparkFun_APDS9960.h>
#include "Adafruit_seesaw.h"
#include <HardwareSerial.h>

// Define Notehub Product UID
#define PRODUCT_UID "com.blues.flex_forge.production_line"

// Notecard setup
Notecard notecard;
HardwareSerial *notecardSerial = &Serial1;  // Adjust to your MCU

// Sensors
Adafruit_BME680 bme;
SFEVL53L1X distanceSensor;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
SparkFun_APDS9960 apds = SparkFun_APDS9960();
Adafruit_seesaw ss;  // Seesaw I2C rotary encoder

uint32_t last_position = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    notecardSerial->begin(9600);
    notecard.begin(*notecardSerial);
    notecard.setDebugOutputStream(Serial);

    configureNotecard();
    initSensors();
}

void loop() {
    // --- Collect Sensor Data ---
    float temperature = readBME();
    float humidity = bme.humidity;
    float pressure = bme.pressure / 100.0;

    // Start measurement
    distanceSensor.startRanging();

    // Wait for data ready
    while (!distanceSensor.checkForDataReady()) {
        delay(1);  // Small wait
    }

    // Read distance
    float distance_mm = distanceSensor.getDistance();

    // Clear interrupt and stop
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();


    sensors_event_t accel, mag, gyro, tempEvent;
    lsm.getEvent(&accel, &mag, &gyro, &tempEvent);

    bool gesture_detected = apds.isGestureAvailable();

    float position = (float)ss.getEncoderPosition();
    float encoder_delta = position - (float)last_position;
    last_position = (uint32_t)position;  // Still store integer for tracking

    // --- Send Note ---
    sendSensorDataToNotehub(temperature, humidity, pressure, distance_mm,
                            accel.acceleration.x, gyro.gyro.x,
                            gesture_detected, position, encoder_delta);

    delay(20000);  // 5-second reporting interval
}

void configureNotecard() {
    J *req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", PRODUCT_UID);
    JAddStringToObject(req, "mode", "continuous");
    JAddStringToObject(req, "sn", "robocop");

    notecard.sendRequestWithRetry(req, 5);
}

void sendSensorDataToNotehub(float temperature, float humidity, float pressure,
                             float distance_mm, float accel_x, float gyro_x,
                             bool gesture_detected, float position, float encoder_delta) {

    J *noteReq = notecard.newRequest("note.add");
    JAddStringToObject(noteReq, "file", "sensors2.qo");
    JAddBoolToObject(noteReq,"sync",true);

    J *body = JCreateObject();
    JAddNumberToObject(body, "temp_c", temperature);
    JAddNumberToObject(body, "humidity_pct", humidity);
    JAddNumberToObject(body, "pressure_hpa", pressure);
    JAddNumberToObject(body, "distance_mm", distance_mm);
    JAddNumberToObject(body, "accel_x_g", accel_x);
    JAddNumberToObject(body, "gyro_x_dps", gyro_x);
    JAddBoolToObject(body, "gesture_detected", gesture_detected);
    JAddNumberToObject(body, "encoder_position", position);
    JAddNumberToObject(body, "encoder_delta", encoder_delta);

    JAddItemToObject(noteReq, "body", body);

    notecard.sendRequest(noteReq);

    Serial.println("Note sent to Notehub.");
}

void initSensors() {
    if (!bme.begin(0x76)) {
        Serial.println("BME688 not found! Halting.");
        // while (1);
    }

    if (distanceSensor.begin()) {
        Serial.println("ToF sensor not found! Halting."); // warn..
        // while (1);
    }

    if (!lsm.begin()) {
        Serial.println("LSM9DS1 not found! Halting.");
        // while (1);
    }

    if (!apds.init()) {
        Serial.println("APDS9960 not found! Halting.");
        // while (1);
    }
    apds.enableGestureSensor();

    if (!ss.begin(0x36)) {  // Default I2C address
        Serial.println("Seesaw encoder not found! Halting.");
        // while (1);
    }
}

float readBME() {
    bme.performReading();
    return bme.temperature;
}
