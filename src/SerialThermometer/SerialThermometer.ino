/*
 * Converts an Arduino Uno board into a single-purpose controller for a set of DS18B20 temperature sensors capable of automated isolation of faulty sensors
 */

#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Arduino.h>
#include "ErrorQueue.h"

// Temperature fetch period in ms (1 s)
#define UPDATE_PERIOD 1000L

// Period of re-initialisation of all sensors in ms (1 min)
#define SETUP_PERIOD 1000 * 60L

// Update / re-initialisation working variables
unsigned long nextUpdate = 0;
unsigned long nextSetup = 0;

// 1-wire bus I/O pin
#define ONE_WIRE_BUS A5

// Pin where voltage is measured during diagnostic
#define BUS_VOLTAGE A4

// Maximal count of sensors (8 due to limited number of free GPIO pin pairs)
#define MAX_SENSORS_COUNT 8

// Defines vcc pin for each sensor
uint8_t vccPins[] = {13, 11, 9, 7, 5, 3, A3, A1};
// Presence flags - true for sensors that were present during (re)initialisation
bool presence[MAX_SENSORS_COUNT];
// Lookup for registered device addresses
DeviceAddress device[MAX_SENSORS_COUNT];
// Lookup for registered device names (address formatted as string)
String deviceName[MAX_SENSORS_COUNT];

// Count of sensors that were present during (re)initialisation
uint8_t sensorCount;

// Flag indicating that normal operation is not possible - forces diagnostic in next iteration
bool fallbackMode;

// Sensor network initialization
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/*
 * Returns Vcc pin number for sensor ordinal number
 */
uint8_t vccPin(int sensorOrdinal) {
    return vccPins[sensorOrdinal];
}

/*
 * Returns GND pin number for sensor ordinal number
 */
uint8_t gndPin(int sensorOrdinal) {
    return vccPins[sensorOrdinal] - (uint8_t) 1;
}

/*
 * Prints sensor reading to Serial
 */
void printReading(const String &name, float temperature) {
    Serial.print('#');
    Serial.print(name);
    Serial.print("\t|\t");
    Serial.println(temperature);
}

/*
 * Tests sensor given by its ordinal number:
 * 1. performs tests detecting possible shortcuts of DQ pin to Vcc/GND
 * 2. attempts to initialize the sensor and read temperature
 * 3. sets presence flag if sensor present
 * Results are printed to Serial.
 */
void testSensor(const uint8_t sensorOrdinal) {
    uint8_t vcc = vccPin(sensorOrdinal);
    uint8_t gnd = gndPin(sensorOrdinal);

    int voltages[9];

    pinMode(vcc, INPUT);
    pinMode(gnd, INPUT);
    delay(100);
    voltages[0] = analogRead(BUS_VOLTAGE);

    pinMode(vcc, OUTPUT);
    digitalWrite(vcc, LOW);
    delay(100);
    voltages[1] = analogRead(BUS_VOLTAGE);

    delay(100);
    voltages[2] = analogRead(BUS_VOLTAGE);

    pinMode(vcc, INPUT);
    pinMode(gnd, INPUT);
    delay(100);
    voltages[3] = analogRead(BUS_VOLTAGE);

    pinMode(gnd, OUTPUT);
    digitalWrite(gnd, LOW);
    delay(100);
    voltages[4] = analogRead(BUS_VOLTAGE);

    delay(100);
    voltages[5] = analogRead(BUS_VOLTAGE);

    pinMode(vcc, OUTPUT);
    digitalWrite(vcc, HIGH);
    delay(100);
    voltages[6] = analogRead(BUS_VOLTAGE);

    pinMode(ONE_WIRE_BUS, OUTPUT);
    digitalWrite(ONE_WIRE_BUS, LOW);
    delay(1);
    voltages[7] = analogRead(BUS_VOLTAGE);

    long recoveryMicros;
    long start = micros();
    int breakpoint = -1;
    pinMode(ONE_WIRE_BUS, INPUT);
    for (int i = 0; i < 10000; i++) {
        if (breakpoint == -1 && digitalRead(BUS_VOLTAGE) == HIGH) {
            breakpoint = i;
        }
    }
    recoveryMicros = (micros() - start) * breakpoint / 10000;

    voltages[8] = analogRead(BUS_VOLTAGE);

    for (int i = 0; i < 9; i++) {
        Serial.print(voltages[i]);
        Serial.print('\t');
    }
    Serial.print("|\t");
    Serial.print(breakpoint);
    Serial.print('\t');
    Serial.print(recoveryMicros);

    Serial.print("|\t");
    sensors.begin();
    sensorCount = sensors.getDeviceCount();
    Serial.print(sensorCount);
    Serial.print('\t');
    if (sensorCount == 0) {
        presence[sensorOrdinal] = false;
        Serial.println("-\t\t\t\t|\t-");
    } else {
        presence[sensorOrdinal] = true;
        DeviceAddress tmp;
        sensors.getAddress(tmp, 0);
        for (int b = 0; b < 8; b++) {
            device[sensorOrdinal][b] = tmp[b];
        }
        String name = "";
        for (int b = 0; b < 8; b++) {
            if (device[sensorOrdinal][b] < 16) {
                name += String('0');
            }
            name += String(device[sensorOrdinal][b], HEX);
            if (b < 7) {
                name += String(':');
            }
        }
        deviceName[sensorOrdinal] = name;
        sensors.requestTemperatures(); // Send the command to get temperatures
        float temperature = sensors.getTempC(device[sensorOrdinal]);
        printReading(name, temperature);
    }
    pinMode(vcc, INPUT);
    pinMode(gnd, INPUT);
}

/*
 * Sets up power for sensors that were present during (re)initialization. Returns count of such sensors.
 */
int setupPower() {
    int presentDevicesCount = 0;
    for (int i = 0; i < MAX_SENSORS_COUNT; i++) {
        uint8_t vcc = vccPin(i);
        uint8_t gnd = gndPin(i);
        if (presence[i]) {
            pinMode(vcc, OUTPUT);
            pinMode(gnd, OUTPUT);
            digitalWrite(vcc, HIGH);
            digitalWrite(gnd, LOW);
            presentDevicesCount++;
        } else {
            pinMode(vcc, INPUT);
            pinMode(gnd, INPUT);
        }
    }
    return presentDevicesCount;
}

/*
 * Sets up sensor network by testing sensors one-by-one and then setting up power for those that were present.
 */
void setupSensors() {
    Serial.println("===BEGIN DIAGNOSTIC==========================================================================");
    for (int i = 0; i < MAX_SENSORS_COUNT; i++) {
        presence[i] = false;
    }
    setupPower();
    delay(100);
    for (int i = 0; i < MAX_SENSORS_COUNT; i++) {
        testSensor(i);
    }
    int expectedDeviceCount = setupPower();
    sensors.begin();
    sensorCount = sensors.getDeviceCount();
    if (expectedDeviceCount != sensorCount) {
        Serial.print("Device count does not match: expected ");
        Serial.print(expectedDeviceCount);
        Serial.print(" but was ");
        Serial.println(sensorCount);
        Serial.println("FALLBACK MODE");
        fallbackMode = true;
        return;
    }
    Serial.print("Device count matches: ");
    Serial.println(sensorCount);
    fallbackMode = sensorCount == 0;
    nextSetup = millis() + SETUP_PERIOD;
    Serial.println("===END DIAGNOSTIC==========================================================================");
}

void setup() {
    Serial.begin(9600);
    setupSensors();
}

void loop() {

    // Update only at regular periods
    if (millis() <= nextUpdate) return;
    nextUpdate += UPDATE_PERIOD;

    // Do not accumulate missed updates
    if (nextUpdate < millis()) {
        nextUpdate = millis();
    }

    // Switch to fallback mode if errors accumulated in error queue.
    if (errorsAccumulated()) {
        fallbackMode = true;
        Serial.println("Errors accumulated, switching to fallback mode");
    }

    // Force re-initialisation if in fall-back mode or if next re-initialisation is due
    if (fallbackMode || millis() > nextSetup) {
        setupSensors();
        clearErrors();
        return;
    }

    // Send the command to the sensor network to get the temperatures
    sensors.requestTemperatures();

    // Error flag to be set if any of the sensors returns invalid temperature value.
    bool error = false;

    // Readings printout
    for (int i = 0; i < MAX_SENSORS_COUNT; i++) {
        if (!presence[i]) continue;
        String reading = "";
        float temperature = sensors.getTempC(device[i]);

        // Set error flag if temperature value is invalid
        if (temperature == 85 || temperature == -127) {
            error = true;
        }
        printReading(deviceName[i], temperature);
    }

    // Log error to the error queue if error flag is set.
    if (error) {
        logError();
        Serial.println("Error happened & logged.");
    }
}

