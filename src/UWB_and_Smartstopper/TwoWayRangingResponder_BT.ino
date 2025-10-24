/*
 * MIT License
 * 
 * Copyright (c) 2018 Michele Biondi, Andrea Salvatori
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file TwoWayRangingResponder_BT.ino
 * 
 * Modified version of TwoWayRangingResponder that sends 4 values via Bluetooth to Master ESP32:
 * - Distance (m)
 * - First Path Power (dBm)
 * - Receive Power (dBm) 
 * - Receive Quality
 * 
 * Data format: "DEVICE_ID,DISTANCE,FP_POWER,RX_POWER,RX_QUALITY\n"
 * Example: "1,3.45,-45.2,-42.1,0.85\n"
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include "BluetoothSerial.h"

// Bluetooth configuration
BluetoothSerial SerialBT;

// Device configuration
#define DEVICE_ID 1  // Change this for each UWB device (1, 2, 3, 4)
#define MASTER_DEVICE_NAME "MasterESP32"  // Master ESP32 Bluetooth name

// connection pins for DW1000
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

// message flow state
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;

// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;

uint64_t timeComputedRange;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

// Bluetooth connection status
bool btConnected = false;
unsigned long lastBTConnectionAttempt = 0;
const unsigned long BT_RECONNECT_INTERVAL = 5000; // 5 seconds

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true,
    true,
    true,
    false,
    true
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("### DW1000Ng-arduino-ranging-anchor-BT ###"));
    
    // Initialize Bluetooth
    SerialBT.begin("UWB_Device_" + String(DEVICE_ID)); // Bluetooth device name
    Serial.println("Bluetooth initialized with name: UWB_Device_" + String(DEVICE_ID));
    
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println(F("DW1000Ng initialized ..."));
    
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    DW1000Ng::setDeviceAddress(DEVICE_ID);
    DW1000Ng::setAntennaDelay(16436);
    
    Serial.println(F("Committed configuration ..."));
    
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    
    // attach callback for (successfully) sent and received messages
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    
    // anchor starts in receiving mode, awaiting a ranging poll message
    receiver();
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = millis();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // anchor listens for POLL
    expectedMsgId = POLL;
    receiver();
    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitPollAck() {
    data[0] = POLL_ACK;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeReport(float curRange) {
    data[0] = RANGE_REPORT;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeFailed() {
    data[0] = RANGE_FAILED;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
}

// Function to check Bluetooth connection
void checkBluetoothConnection() {
    if (!btConnected) {
        if (millis() - lastBTConnectionAttempt > BT_RECONNECT_INTERVAL) {
            Serial.println("Attempting to connect to Master ESP32...");
            if (SerialBT.connect(MASTER_DEVICE_NAME)) {
                btConnected = true;
                Serial.println("Connected to Master ESP32!");
            } else {
                Serial.println("Failed to connect to Master ESP32");
                lastBTConnectionAttempt = millis();
            }
        }
    } else {
        // Check if still connected
        if (!SerialBT.connected()) {
            btConnected = false;
            Serial.println("Bluetooth connection lost");
        }
    }
}

// Function to send data via Bluetooth
void sendDataViaBluetooth(double distance, float fpPower, float rxPower, float rxQuality) {
    if (btConnected) {
        // Create formatted string: "DEVICE_ID,DISTANCE,FP_POWER,RX_POWER,RX_QUALITY\n"
        String dataString = String(DEVICE_ID) + "," + 
                           String(distance, 3) + "," + 
                           String(fpPower, 1) + "," + 
                           String(rxPower, 1) + "," + 
                           String(rxQuality, 2) + "\n";
        
        // Send via Bluetooth
        SerialBT.print(dataString);
        
        // Print to Serial for debugging
        Serial.print("BT Data sent: ");
        Serial.print(dataString);
    } else {
        Serial.println("Bluetooth not connected - data not sent");
    }
}

void loop() {
    // Check Bluetooth connection
    checkBluetoothConnection();
    
    int32_t curMillis = millis();
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Ng::getTransmitTimestamp();
            noteActivity();
        }
        DW1000Ng::startReceive();
    }
    
    if (receivedAck) {
        receivedAck = false;
        // get message and parse
        DW1000Ng::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
        }
        if (msgId == POLL) {
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;
            timePollReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = RANGE;
            transmitPollAck();
            noteActivity();
        }
        else if (msgId == RANGE) {
            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = POLL;
            if (!protocolFailed) {
                timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
                timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
                timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
                
                // (re-)compute range as two-way ranging is done
                double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
                /* Apply simple bias correction */
                distance = DW1000NgRanging::correctRange(distance);
                
                // Get the 4 values to send via Bluetooth
                float fpPower = DW1000Ng::getFirstPathPower();
                float rxPower = DW1000Ng::getReceivePower();
                float rxQuality = DW1000Ng::getReceiveQuality();
                
                // Send data via Bluetooth
                sendDataViaBluetooth(distance, fpPower, rxPower, rxQuality);
                
                // Print to Serial for debugging
                String rangeString = "Device " + String(DEVICE_ID) + " - Range: "; 
                rangeString += distance; rangeString += " m";
                rangeString += "\t FP power: "; rangeString += fpPower; rangeString += " dBm";
                rangeString += "\t RX power: "; rangeString += rxPower; rangeString += " dBm";
                rangeString += "\t RX quality: "; rangeString += rxQuality;
                rangeString += "\t Sampling: "; rangeString += samplingRate; rangeString += " Hz";
                rangeString += "\t BT: "; rangeString += (btConnected ? "Connected" : "Disconnected");
                Serial.println(rangeString);
                
                // update sampling rate (each second)
                transmitRangeReport(distance * DISTANCE_OF_RADIO_INV);
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            }
            else {
                transmitRangeFailed();
            }

            noteActivity();
        }
    }
}
