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

#define TEMP_THRESHOLD_C 21
#define MAG_CHANGE_THRESHOLD 0.15  // 15%

bool paranormalFlag = false;
float lastStableMagField = NAN;

uint32_t last_position = 0;

// Notecard setup
Notecard notecard;
HardwareSerial *notecardSerial = &Serial1;  // Adjust to your MCU

// Sensors
Adafruit_BME680 bme;
SFEVL53L1X distanceSensor;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
SparkFun_APDS9960 apds = SparkFun_APDS9960();
Adafruit_seesaw ss;  // Seesaw I2C rotary encoder

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
    while (!distanceSensor.checkForDataReady()) {
        delay(1);
    }
    float distance_mm = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();

    sensors_event_t accel, mag, gyro, tempEvent;
    lsm.getEvent(&accel, &mag, &gyro, &tempEvent);

    bool gesture_detected = apds.isGestureAvailable();

    float position = (float)ss.getEncoderPosition();
    float encoder_delta = position - (float)last_position;
    last_position = (uint32_t)position;

    float totalMagField = fabs(mag.magnetic.x) + fabs(mag.magnetic.y) + fabs(mag.magnetic.z);
    // Serial.println(totalMagField);

    if (isnan(lastStableMagField)) {
        lastStableMagField = totalMagField;  // Initialize baseline
    }

    float magChange = fabs(totalMagField - lastStableMagField) / fmax(lastStableMagField, 0.01);
    bool magExceeded = magChange > MAG_CHANGE_THRESHOLD;

    bool tempBelowThreshold = temperature < TEMP_THRESHOLD_C;

    float gyroThresh = 1;
    bool motionEvent = (gyro.gyro.x > gyroThresh || gyro.gyro.y > gyroThresh || gyro.gyro.z > gyroThresh );
    float totalAccel = sqrt(
        accel.acceleration.x * accel.acceleration.x +
        accel.acceleration.y * accel.acceleration.y +
        accel.acceleration.z * accel.acceleration.z
    ); // automated iterating ghost predictive tool

    bool gravEvent = (totalAccel < 9.4 || totalAccel > 10.2);
    Serial.println("Mag: " + String(magExceeded) + 
               " TempEvent: " + String(tempBelowThreshold) + 
               " MotionEvent: " + String(motionEvent) + 
               " GravEvent: " + String(gravEvent));

    // Paranormal flag logic
    if (tempBelowThreshold || magExceeded || motionEvent || gravEvent) {
        paranormalFlag = true;
    } else {
        paranormalFlag = false;
        lastStableMagField = totalMagField;  // Update baseline after calm
    }
    

    // --- Send Note ---
    sendSensorDataToNotehub(
        temperature, humidity, pressure, distance_mm,
        accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
        gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
        mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
        gesture_detected, position, encoder_delta
    );

    delay(5000);  // 20-second reporting interval
}

void configureNotecard() {
    J *req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", PRODUCT_UID);
    JAddStringToObject(req, "mode", "continuous");
    JAddStringToObject(req, "sn", "robocop");

    notecard.sendRequestWithRetry(req, 5);
}

void sendSensorDataToNotehub(
    float temperature, float humidity, float pressure, float distance_mm,
    float accel_x, float accel_y, float accel_z,
    float gyro_x, float gyro_y, float gyro_z,
    float mag_x, float mag_y, float mag_z,
    bool gesture_detected, float position, float encoder_delta
) {
    J *noteReq = notecard.newRequest("note.add");
    JAddStringToObject(noteReq, "file", "sensors2.qo");
    JAddBoolToObject(noteReq,"sync",true);

    J *body = JCreateObject();
    JAddNumberToObject(body, "temp_c", temperature);
    JAddNumberToObject(body, "humidity_pct", humidity);
    JAddNumberToObject(body, "pressure_hpa", pressure);
    JAddNumberToObject(body, "distance_mm", distance_mm);

    // Accelerometer
    JAddNumberToObject(body, "accel_x_g", accel_x);
    JAddNumberToObject(body, "accel_y_g", accel_y);
    JAddNumberToObject(body, "accel_z_g", accel_z);

    // Gyroscope
    JAddNumberToObject(body, "gyro_x_dps", gyro_x);
    JAddNumberToObject(body, "gyro_y_dps", gyro_y);
    JAddNumberToObject(body, "gyro_z_dps", gyro_z);

    // Magnetometer
    JAddNumberToObject(body, "mag_x_uT", mag_x);
    JAddNumberToObject(body, "mag_y_uT", mag_y);
    JAddNumberToObject(body, "mag_z_uT", mag_z);

    // Other sensors
    JAddBoolToObject(body, "gesture_detected", gesture_detected);
    JAddNumberToObject(body, "encoder_position", position);
    JAddNumberToObject(body, "encoder_delta", encoder_delta);

    JAddBoolToObject(body, "paranormal", paranormalFlag);

    JAddItemToObject(noteReq, "body", body);

    notecard.sendRequest(noteReq);

    Serial.println("Note sent to Notehub.");
}

void initSensors() {
    if (!bme.begin(0x76)) {
        Serial.println("BME688 not found! Halting.");
    }

    if (distanceSensor.begin()) {  // You confirmed 0 = success
        Serial.println("ToF sensor not found! Halting.");
    }

    if (!lsm.begin()) {
        Serial.println("LSM9DS1 not found! Halting.");
    }

    if (!apds.init()) {
        Serial.println("APDS9960 not found! Halting.");
    }
    apds.enableGestureSensor();

    if (!ss.begin(0x36)) {
        Serial.println("Seesaw encoder not found! Halting.");
    }
}

float readBME() {
    bme.performReading();
    return bme.temperature;
}
