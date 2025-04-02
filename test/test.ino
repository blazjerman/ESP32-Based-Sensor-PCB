#include <Wire.h>
#include <ModbusMaster.h>

#define LIGHT_SENSOR_PIN 27

#define BOOSTER_ENABLE_PIN 19

#define RS485_DI 17
#define RS485_RO 16
#define RS485_REDE 18
#define RS485_BOUND_RATE 19200
#define RS485_CONF SERIAL_8E1
#define RS485_DEVICE_ADRESS 1
#define RS485_READ_REGISTER 0x4


#define EXTERNAL_SENSOR_TX 25
#define EXTERNAL_SENSOR_RX 26

#define SOUND_SENSOR_TX 14
#define SOUND_SENSOR_RX 12

#define RGB_RED 13
#define RGB_GREEN 2
#define RGB_BLUE 32

#define ANA_PIN 33

ModbusMaster node;

HardwareSerial mySerial(1);

void preTransmission() {
    digitalWrite(RS485_REDE, HIGH);
}

void postTransmission() {
    digitalWrite(RS485_REDE, LOW);
}

// Function to switch UART pins dynamically
void switchUartPins(uint8_t rxPin, uint8_t txPin, unsigned long bound, uint32_t conf) {
    mySerial.begin(bound, conf, rxPin, txPin);
    Serial.println("UART pins changed.");
}

void setup() {
    
    Serial.begin(115200);  // USB serial communication (for debug)
    
    pinMode(RS485_DI, OUTPUT);
    pinMode(RS485_RO, INPUT);
    pinMode(RS485_REDE, OUTPUT);
    pinMode(SOUND_SENSOR_TX, OUTPUT);
    pinMode(SOUND_SENSOR_RX, INPUT);
    pinMode(RGB_RED, OUTPUT);
    pinMode(RGB_GREEN, OUTPUT);
    pinMode(RGB_BLUE, OUTPUT);
    pinMode(ANA_PIN, INPUT);
    
    // Power and sensor-related pins
    pinMode(BOOSTER_ENABLE_PIN, OUTPUT);
    digitalWrite(BOOSTER_ENABLE_PIN, HIGH);

    Wire.begin();
    
    node.begin(RS485_DEVICE_ADRESS, mySerial);  // Start Modbus communication on mySerial
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
    
    Serial.println("Setup complete.");

}


void scanI2C() {
    Serial.println("Scanning I2C bus...");
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found I2C device at 0x");
            Serial.println(address, HEX);
        }
    }
    Serial.println("I2C scan complete.");
}

void loop() {

  Serial.println("-----------------------------------------------");
  
  // I2C scanner
  scanI2C();
  Serial.println();

  
  // Light sensor
  int light_value = analogRead(LIGHT_SENSOR_PIN);
  Serial.print("Light Sensor Value: ");
  Serial.println(light_value);

  Serial.println("Cycling RGB LED");
  analogWrite(RGB_RED, 10);
  analogWrite(RGB_GREEN, 0);
  analogWrite(RGB_BLUE, 0);
  delay(200);
  analogWrite(RGB_RED, 0);
  analogWrite(RGB_GREEN, 10);
  analogWrite(RGB_BLUE, 0);
  delay(200);
  analogWrite(RGB_RED, 0);
  analogWrite(RGB_GREEN, 0);
  analogWrite(RGB_BLUE, 10);
  delay(200);
  analogWrite(RGB_RED, 0);
  analogWrite(RGB_GREEN, 0);
  analogWrite(RGB_BLUE, 0);

  Serial.println();


  // Testing external uart 
  Serial.println("Testing UART Loopback");
  switchUartPins(EXTERNAL_SENSOR_RX, EXTERNAL_SENSOR_TX, 9600, SERIAL_8N1);
  mySerial.println("UART Loopback Test");
  
  delay(100);

  Serial.print("Received: ");
  while (mySerial.available()) Serial.write(mySerial.read());
  Serial.println();

  Serial.println();


  // Testing sound sensor uart 
  Serial.println("Testing sound sensor UART Loopback");
  switchUartPins(SOUND_SENSOR_RX, SOUND_SENSOR_TX, 9600, SERIAL_8N1);
  mySerial.println("UART sound sensor Loopback Test");
  delay(100);
  Serial.print("Received: ");
  while (mySerial.available()) Serial.write(mySerial.read());
  Serial.println();

  Serial.println();


  // Testing pin ana
  int ana_value = analogRead(ANA_PIN);
  Serial.print("Analog Input Value: ");
  Serial.println(ana_value);

  Serial.println();


  // Use UART for RS485 communication (switch UART pins to RS485)
  Serial.println("Modbus RS485 Communication");
  switchUartPins(RS485_RO, RS485_DI, RS485_BOUND_RATE, RS485_CONF);

  uint16_t data;
  uint8_t result = node.readInputRegisters(RS485_READ_REGISTER, 2);
  
  if (result == node.ku8MBSuccess) {
      data = node.getResponseBuffer(0);
      Serial.print("Read Successful, Value: ");
      Serial.println(data);
  } else {
      Serial.print("Read Failed, Code: ");
      Serial.println(result, HEX);
  }

  delay(1000);

}
