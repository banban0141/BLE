#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SENSOR0 14
#define SENSOR1 25
#define SENSOR2 33
#define LED0 18
#define LED1 19
#define LED2 21

#define device_name  "RotRuou"
int timeDelay = 500;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;


bool sensor0[3];
bool sensor1[3];
bool sensor2[3];

bool sensor_state[3] = { 0, 0, 0};
bool sensor_change[3] = { 0, 0, 0};
bool tam[3] = { 0, 0, 0};

uint32_t lastDebounceTime[3] = { 0, 0, 0};
bool  lastSensorState[3] = { 0, 0, 0};

bool ui2_BTreceive = false;
bool ui2_start = false;
bool ui2_BTcomplete = false;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0, value_x2 = 0;
unsigned long timer;
uint8_t txValue = 0;
String rx_String;
char rx_array[50];
boolean rx_complete = false;
unsigned int rx_length = 0;
unsigned char tx_array[6] = {0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA};
uint16_t count = 0;
uint16_t count1 = 0;

uint32_t ui32_timecho_guitrangthai ;
uint32_t ui32_timedoccambien;

#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
/**********************Nhận dữu liệu từ Characteristic************************/
class MyCallbacks: public BLECharacteristicCallbacks {
  /* Nhận dữu liệu từ Characteristic:*/
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      rx_String = "";
      if (rxValue.length() > 0) {
        rx_length = rxValue.length();
        Serial.println("*********");
        Serial.print("Received Value from Characteristic: ");
        for (int i = 0; i < rxValue.length(); i++){
          Serial.print(rxValue[i]);
          rx_array[i]=rxValue[i];
        }
        rx_complete = true;
        Serial.println();
        Serial.println("*********");
      }
    } 
};

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR0,INPUT_PULLUP);
  pinMode(SENSOR1,INPUT_PULLUP);
  pinMode(SENSOR2,INPUT_PULLUP);

  pinMode(LED0,OUTPUT);
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);

  // Create the BLE Device
  BLEDevice::init(device_name);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  timer = millis();

  ui32_timecho_guitrangthai = millis();
  ui32_timedoccambien = millis();

   
}
/**********************Ham con**************************/
void ham_con(){
  Serial.print("So byte nhan duoc: ");
  Serial.println(rx_length);
  Serial.print("Data nhan duoc: ");
  for(int i = 0; i<rx_length; i++){
    Serial.print(rx_array[i]);
    
  }
  Serial.println();
  Serial.print("Data nhan duoc dang HEX: ");
  for(int i = 0; i<rx_length; i++){
    Serial.print((unsigned char)rx_array[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

/**********************Đọc cảm biến************************/
// Đọc cảm biến 3 lần cách nhau 10ms để tránh nhiễu//
void readSensor(){
  
//   if ((millis() - ui32_timedoccambien) > 10){
//     sensor0[2] = sensor0[1];
//     sensor0[1] = sensor0[0];
//     sensor0[0] = !digitalRead(SENSOR0);
//     if (sensor0[0] == sensor0[1] && sensor0[1] == sensor0[2]){
//       if(sensor_state[0] != sensor0[0]){
//         sensor_state[0] = sensor0[0];
//         sensor_change[0] = 1;
//       } 
//     }
  
//     sensor1[2] = sensor1[1];
//     sensor1[1] = sensor1[0];
//     sensor1[0] = !digitalRead(SENSOR1);
//     if (sensor1[0] == sensor1[1] && sensor1[1] == sensor1[2]){
//       if(sensor_state[1] != sensor1[0]){
//         sensor_state[1] = sensor1[0];
//         sensor_change[1] = 1;
//       } 
//     }
    
//     sensor2[2] = sensor2[1];
//     sensor2[1] = sensor2[0];
//     sensor2[0] = !digitalRead(SENSOR2);
//     if (sensor2[0] == sensor2[1] && sensor2[1] == sensor2[2]){
//       if(sensor_state[2] != sensor2[0]){
//         sensor_state[2] = sensor2[0];
//         sensor_change[2] = 1;
//       }
//     }
//     ui32_timedoccambien = millis();
//   }

  readSensorDebounce(SENSOR0, 0, timeDelay);
  readSensorDebounce(SENSOR1, 1, timeDelay);
  readSensorDebounce(SENSOR2, 2, timeDelay); 


}
/**********************Đọc trang thái cảm biến************************/

void readSensorDebounce(int pin_cambien, int index, int delay){
  bool reading = !digitalRead(pin_cambien);

  if (reading != lastSensorState[index]) {		
		lastDebounceTime[index] = millis();	
    lastSensorState[index] = reading; 
  }

  if ((millis() - lastDebounceTime[index]) > delay) {
    sensor_state[index] = reading;
  }
  if(sensor_state[index] != tam[index]){
    tam[index] = sensor_state[index];
    sensor_change[index] = 1;	
  }
  

}


/**********************Truyền trạng thái cảm biến************************/
void send_state( bool state0, bool state1, bool state2){
  if (deviceConnected){
    std::string myString = "SENSOR" + std::to_string(state0) + std::to_string(state1) + std::to_string(state2);
    pCharacteristic->setValue((uint8_t *)myString.c_str(), myString.length());
    pCharacteristic->notify();
  }
  
}
/**********************Set trạng thái đèn************************/
void setLED( bool state0, bool state1, bool state2){
digitalWrite(LED0, state0);
digitalWrite(LED1, state1);
digitalWrite(LED2, state2);
}

/**********************Xử lý truyền************************/
void process(){
  if(sensor_change[0] || sensor_change[1] || sensor_change[2]){
    send_state(sensor_state[0], sensor_state[1], sensor_state[2]);
    // setLED(sensor_state[0], sensor_state[1], sensor_state[2]);
    sensor_change[0] = 0;
    sensor_change[1] = 0;
    sensor_change[2] = 0;
    ui32_timecho_guitrangthai = millis();
    count++;
    Serial.printf("\n Timecount: %u \n", count);
  }
  else if(millis() - ui32_timecho_guitrangthai > 20000){
    send_state(sensor_state[0], sensor_state[1], sensor_state[2]);
    count1++;
    Serial.printf("\n Timecount1: %u \n", count1);
    ui32_timecho_guitrangthai = millis();
  }
}


/**********************LOOP************************/
void loop() {
  readSensor();
  process();
  setLED(lastSensorState[0], lastSensorState[1], lastSensorState[2]);

  //
  //
  //  send_state(1,1,1);
  //  delay(2000);
  //  send_state(0,0,0);
  //  delay(2000);


  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
  
}

