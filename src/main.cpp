/*********
  
  Kamran R.
  Waqas M.
  Nabil E.

  Robbin Law
  Rui Santos
  
*********/

#include <Arduino.h>          // base library
#include <esp_now.h>          // esp_now for communication
#include <WiFi.h>             // for WiFi communiation to fetch time
#include <Adafruit_Sensor.h>  // default sensor library
#include <Adafruit_BME280.h>  // for BME280 sensor
#include <Adafruit_MPU6050.h> // for MPU6050 sensor
#include <Wire.h>             // i2c base library
#include "time.h"             // for current time and date
#include "TaskScheduler.h"    // for mimic delay

// for setting pressure
#define SEALEVELPRESSURE_HPA (1013.25)

// defining the pins need to control and monitor
#define INPIN_1 14
#define INPIN_2 25
#define INPIN_3 26
#define INPIN_4 27

// scheduler times
#define dataSendTime 20000

// declaration of functions for setup
void pinSetup();
void bmeSetup();
void mpu6050Setup();
void timeSetup();
void espNowSetup();

// declaration of functions for performing
void readPins();
void OnDataSent();
void printLocalTime();
void espNowSend();
void refresh_readings();
void detectChange();

// Local WiFi Credentials
const char *WIFI_SSID = "YOUR WIFI SSID";
const char *WIFI_PASS = "YOUR WIFI PASS";

// time variable setup
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -25362;
const int daylightOffset_sec = 3600;

// sensor variable
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

// RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xF9, 0x0E, 0x98};

// Must match the sender receiver structure
typedef struct struct_message
{
  int id;            // must be unique for each sender board
  int pinStatus[4];  // for peripheral status
  float temperature; // for storing temperature
  float humidity;    // for storing himmidity
  float pressure;    // for storing pressure
  float altitude;    // for storing altitude

  float temp6050;    // for storing onboard temperature
  float A_values[3]; // for storing accelrometer values
  float G_values[3]; // for storing gyroscope values
} struct_message;

//Create a struct_message called myData
struct_message myData;

// for getting values from MPU6050 sensor
float AcX, AcY, AcZ, tmp, GyX, GyY, GyZ;

// Register peer
esp_now_peer_info_t peerInfo;

// Setup tasks for the task scheduler
Task dataCommunication(dataSendTime, TASK_FOREVER, &espNowSend);

// Create the scheduler
Scheduler runner;

// main setup function
void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);

  bmeSetup();
  mpu6050Setup();
  timeSetup();
  pinSetup();
  espNowSetup();

  // Start the task scheduler
  runner.init();

  // Add the task to the scheduler
  runner.addTask(dataCommunication);

  // Enable the task
  dataCommunication.enable();
}

// the loop functions
void loop()
{
  // Execute the scheduler runner
  runner.execute();

  // detects any change on the inputs and respond accordingly
  detectChange();
}

// time function
void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "\n%A, %B %d, %Y %H:%M:%S");
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// setting up pin in input mode
void pinSetup()
{
  // Setting pin for read mode
  pinMode(INPIN_1, INPUT);
  pinMode(INPIN_2, INPUT);
  pinMode(INPIN_3, INPUT);
  pinMode(INPIN_4, INPUT);

  for (int i = 0; i < 4; i++)
  {
    myData.pinStatus[i] = 0;
  }
}

// time setup function
void timeSetup()
{
  // connect to WiFi
  Serial.printf("Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// setting up bme i2c
void bmeSetup()
{
  if (bme.begin(BME280_ADDRESS_ALTERNATE))
  {
    Serial.println("Found BME280 Chip...");
  }
  else
  {
    Serial.println("Could not find BME280 Chip, check wiring!");
    while (1)
    {
      Serial.print(".");
    }
  }
}

// setting up MPU6050 i2c
void mpu6050Setup()
{
  if (mpu.begin())
  {
    Serial.println("Found MPU6050 Chip...");
  }
  else
  {
    Serial.println("Could not find MPU6050 Chip, check wiring!");
    while (1)
    {
      Serial.print(".");
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

// setting up esp NOW
void espNowSetup()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

// reading Pin Status
void readPins()
{
  // Set values to send
  myData.id = 1;
  myData.pinStatus[0] = digitalRead(INPIN_1);
  myData.pinStatus[1] = digitalRead(INPIN_2);
  myData.pinStatus[2] = digitalRead(INPIN_3);
  myData.pinStatus[3] = digitalRead(INPIN_4);

  Serial.println();
  Serial.println("*** Controller Values ***");

  Serial.printf("Control 14 is %3s.", myData.pinStatus[0] ? "ON" : "OFF");
  Serial.println();

  for (int i = 1; i < 4; i++)
  {
    Serial.printf("Control %d is %3s.", i + 24, myData.pinStatus[i] ? "ON" : "OFF");
    Serial.println();
  }
}

// esp NOW send message
void espNowSend()
{
  Serial.println();
  Serial.println("****************************************");

  Serial.print("My MAC Address:  ");
  Serial.println(WiFi.macAddress());

  // display time
  printLocalTime();

  // reading Pins
  readPins();

  // reading from BME
  refresh_readings();

  // displaying BME280 values on the serial output
  Serial.println();
  Serial.println("*** BME280 Values ***");
  Serial.printf("Temperature:   %-6.2f °C", myData.temperature);
  Serial.println();
  Serial.printf("Humidity:      %-6.2f %%", myData.humidity);
  Serial.println();
  Serial.printf("Pressure:      %-6.2f hPa", myData.pressure);
  Serial.println();
  Serial.printf("Altitude:      %-6.2f m", myData.altitude);
  Serial.println();

  // displaying MPU6050 values on the serial output
  Serial.println();
  Serial.println("*** MPU6050 Values ***");
  Serial.printf("Temperature:   %-6.2f °C", myData.temp6050);
  Serial.println();
  Serial.printf("Acceleration   X: %5.2f, Y: %5.2f, Z: %5.2f   m/s^2", myData.A_values[0], myData.A_values[1], myData.A_values[2]);
  Serial.println();
  Serial.printf("Rotation       X: %5.2f, Y: %5.2f, Z: %5.2f   rad/s", myData.G_values[0], myData.G_values[1], myData.G_values[2]);
  Serial.println();

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
}

// reading from sensor
void refresh_readings()
{
  // reading the readings from the BME280 Chip
  myData.temperature = bme.readTemperature();
  myData.humidity = bme.readHumidity();
  myData.pressure = bme.readPressure() / 100.0F;
  myData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // reading the readings from the MPU6050 Chip
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  myData.temp6050 = temp.temperature;

  myData.A_values[0] = a.acceleration.x;
  myData.A_values[1] = a.acceleration.y;
  myData.A_values[2] = a.acceleration.z;

  myData.G_values[0] = g.gyro.x;
  myData.G_values[1] = g.gyro.y;
  myData.G_values[2] = g.gyro.z;
}

// kind of interrupt function
void detectChange()
{
  // checking the input change
  if (myData.pinStatus[0] != digitalRead(INPIN_1) ||
      myData.pinStatus[1] != digitalRead(INPIN_2) ||
      myData.pinStatus[2] != digitalRead(INPIN_3) ||
      myData.pinStatus[3] != digitalRead(INPIN_4))
  {
    Serial.println("Change Detected...");
    // Disable the tasks
    dataCommunication.disable();

    // Enable the task
    dataCommunication.enable();

    // Update the change
    delay(1000);
  }
}