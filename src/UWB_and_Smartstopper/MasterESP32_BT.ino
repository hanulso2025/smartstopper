/*
 * Master ESP32 Bluetooth Receiver for UWB Positioning System
 * 
 * This code receives data from 4 UWB devices via Bluetooth and performs:
 * 1. NLOS filtering based on signal quality metrics
 * 2. Selects top 3 most reliable distance measurements
 * 3. Performs trilateration for position calculation
 * 
 * Data format received: "DEVICE_ID,DISTANCE,FP_POWER,RX_POWER,RX_QUALITY\n"
 * Example: "1,3.45,-45.2,-42.1,0.85\n"
 */

#include "BluetoothSerial.h"

// Bluetooth configuration
BluetoothSerial SerialBT;

// Data structure for UWB measurements
struct UWBData {
  int deviceId;
  double distance;
  float fpPower;
  float rxPower;
  float rxQuality;
  float reliability;  // Calculated reliability score
  unsigned long timestamp;
  bool valid;
};

// Anchor positions (x, y coordinates in meters)
struct AnchorPosition {
  float x, y;
};

// System configuration
#define MAX_DEVICES 4
#define MAX_MEASUREMENTS 10
#define NLOS_THRESHOLD_FP_POWER -50.0  // dBm threshold for NLOS detection
#define NLOS_THRESHOLD_RX_POWER -45.0  // dBm threshold for NLOS detection
#define NLOS_THRESHOLD_QUALITY 0.7     // Quality threshold for NLOS detection
#define MAX_DISTANCE 50.0              // Maximum valid distance in meters
#define MIN_DISTANCE 0.1               // Minimum valid distance in meters

// Anchor positions (modify these coordinates according to your setup)
AnchorPosition anchors[MAX_DEVICES] = {
  {0.0, 0.0},    // Device 1
  {10.0, 0.0},   // Device 2
  {5.0, 8.66},   // Device 3
  {5.0, 4.33}    // Device 4
};

// Data storage
UWBData measurements[MAX_DEVICES];
UWBData filteredMeasurements[MAX_DEVICES];
int measurementCount = 0;

// Position calculation variables
float calculatedX = 0.0;
float calculatedY = 0.0;
bool positionValid = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("### Master ESP32 UWB Positioning System ###");
  
  // Initialize Bluetooth
  SerialBT.begin("MasterESP32");
  Serial.println("Bluetooth initialized as MasterESP32");
  Serial.println("Waiting for UWB devices to connect...");
  
  // Initialize measurement arrays
  for (int i = 0; i < MAX_DEVICES; i++) {
    measurements[i].valid = false;
    measurements[i].deviceId = i + 1;
  }
  
  Serial.println("System ready. Waiting for data from UWB devices...");
}

void loop() {
  // Check for incoming Bluetooth data
  if (SerialBT.available()) {
    String receivedData = SerialBT.readStringUntil('\n');
    receivedData.trim();
    
    if (receivedData.length() > 0) {
      processUWBData(receivedData);
    }
  }
  
  // Perform positioning calculation every 100ms
  static unsigned long lastCalculation = 0;
  if (millis() - lastCalculation > 100) {
    performPositioning();
    lastCalculation = millis();
  }
  
  // Print status every 5 seconds
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 5000) {
    printSystemStatus();
    lastStatusPrint = millis();
  }
}

void processUWBData(String data) {
  // Parse data: "DEVICE_ID,DISTANCE,FP_POWER,RX_POWER,RX_QUALITY"
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);
  int fourthComma = data.indexOf(',', thirdComma + 1);
  
  if (firstComma == -1 || secondComma == -1 || thirdComma == -1 || fourthComma == -1) {
    Serial.println("Invalid data format: " + data);
    return;
  }
  
  int deviceId = data.substring(0, firstComma).toInt();
  double distance = data.substring(firstComma + 1, secondComma).toDouble();
  float fpPower = data.substring(secondComma + 1, thirdComma).toFloat();
  float rxPower = data.substring(thirdComma + 1, fourthComma).toFloat();
  float rxQuality = data.substring(fourthComma + 1).toFloat();
  
  // Validate device ID
  if (deviceId < 1 || deviceId > MAX_DEVICES) {
    Serial.println("Invalid device ID: " + String(deviceId));
    return;
  }
  
  // Store measurement
  int index = deviceId - 1;
  measurements[index].deviceId = deviceId;
  measurements[index].distance = distance;
  measurements[index].fpPower = fpPower;
  measurements[index].rxPower = rxPower;
  measurements[index].rxQuality = rxQuality;
  measurements[index].timestamp = millis();
  measurements[index].valid = true;
  
  // Calculate reliability score
  measurements[index].reliability = calculateReliability(fpPower, rxPower, rxQuality, distance);
  
  Serial.println("Received from Device " + String(deviceId) + 
                ": D=" + String(distance, 2) + 
                "m, FP=" + String(fpPower, 1) + 
                "dBm, RX=" + String(rxPower, 1) + 
                "dBm, Q=" + String(rxQuality, 2) + 
                ", R=" + String(measurements[index].reliability, 2));
}

float calculateReliability(float fpPower, float rxPower, float rxQuality, double distance) {
  float reliability = 1.0;
  
  // Distance validity check
  if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
    return 0.0;
  }
  
  // NLOS detection based on signal quality
  if (fpPower < NLOS_THRESHOLD_FP_POWER) {
    reliability *= 0.5;  // Reduce reliability for weak first path
  }
  
  if (rxPower < NLOS_THRESHOLD_RX_POWER) {
    reliability *= 0.7;  // Reduce reliability for weak overall signal
  }
  
  if (rxQuality < NLOS_THRESHOLD_QUALITY) {
    reliability *= 0.6;  // Reduce reliability for poor signal quality
  }
  
  // Signal strength bonus
  if (fpPower > -40.0 && rxPower > -35.0) {
    reliability *= 1.2;  // Bonus for strong signals
  }
  
  // Quality bonus
  if (rxQuality > 0.9) {
    reliability *= 1.1;  // Bonus for high quality
  }
  
  return min(reliability, 1.0);  // Cap at 1.0
}

void performPositioning() {
  // Filter and sort measurements by reliability
  int validCount = 0;
  
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (measurements[i].valid && 
        millis() - measurements[i].timestamp < 2000 &&  // Data not older than 2 seconds
        measurements[i].reliability > 0.3) {  // Minimum reliability threshold
      filteredMeasurements[validCount] = measurements[i];
      validCount++;
    }
  }
  
  if (validCount < 3) {
    positionValid = false;
    return;
  }
  
  // Sort by reliability (bubble sort for simplicity)
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = 0; j < validCount - i - 1; j++) {
      if (filteredMeasurements[j].reliability < filteredMeasurements[j + 1].reliability) {
        UWBData temp = filteredMeasurements[j];
        filteredMeasurements[j] = filteredMeasurements[j + 1];
        filteredMeasurements[j + 1] = temp;
      }
    }
  }
  
  // Use top 3 measurements for trilateration
  int useCount = min(3, validCount);
  
  // Perform trilateration
  if (trilateration(useCount)) {
    positionValid = true;
    Serial.println("Position calculated: X=" + String(calculatedX, 2) + 
                  "m, Y=" + String(calculatedY, 2) + "m");
  } else {
    positionValid = false;
  }
}

bool trilateration(int count) {
  // Simple trilateration using least squares method
  // This is a simplified version - for production use, implement proper trilateration
  
  if (count < 3) return false;
  
  // Use the first 3 measurements
  float x1 = anchors[filteredMeasurements[0].deviceId - 1].x;
  float y1 = anchors[filteredMeasurements[0].deviceId - 1].y;
  float r1 = filteredMeasurements[0].distance;
  
  float x2 = anchors[filteredMeasurements[1].deviceId - 1].x;
  float y2 = anchors[filteredMeasurements[1].deviceId - 1].y;
  float r2 = filteredMeasurements[1].distance;
  
  float x3 = anchors[filteredMeasurements[2].deviceId - 1].x;
  float y3 = anchors[filteredMeasurements[2].deviceId - 1].y;
  float r3 = filteredMeasurements[2].distance;
  
  // Calculate position using trilateration formula
  float A = 2 * (x2 - x1);
  float B = 2 * (y2 - y1);
  float C = r1 * r1 - r2 * r2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
  float D = 2 * (x3 - x2);
  float E = 2 * (y3 - y2);
  float F = r2 * r2 - r3 * r3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3;
  
  float det = A * E - B * D;
  
  if (abs(det) < 0.001) {
    return false;  // No solution
  }
  
  calculatedX = (C * E - F * B) / det;
  calculatedY = (A * F - D * C) / det;
  
  return true;
}

void printSystemStatus() {
  Serial.println("\n=== System Status ===");
  Serial.println("Bluetooth connected devices: " + String(SerialBT.hasClient() ? "Yes" : "No"));
  
  Serial.println("UWB Device Status:");
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (measurements[i].valid) {
      unsigned long age = millis() - measurements[i].timestamp;
      Serial.println("  Device " + String(measurements[i].deviceId) + 
                    ": D=" + String(measurements[i].distance, 2) + 
                    "m, R=" + String(measurements[i].reliability, 2) + 
                    ", Age=" + String(age) + "ms");
    } else {
      Serial.println("  Device " + String(i + 1) + ": No data");
    }
  }
  
  if (positionValid) {
    Serial.println("Calculated Position: X=" + String(calculatedX, 2) + 
                  "m, Y=" + String(calculatedY, 2) + "m");
  } else {
    Serial.println("Position: Not available (insufficient data)");
  }
  Serial.println("===================\n");
}
