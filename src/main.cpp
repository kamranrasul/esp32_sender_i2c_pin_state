/*********
  
  Kamran R.
  Waqas M.
  Nabil E.

  Robbin Law
  Rui Santos
  
*********/

#include <Arduino.h>         // base library
#include <esp_now.h>         // esp_now for communication
#include <WiFi.h>            // for WiFi communiation to fetch time
#include <Adafruit_BME280.h> // for BME280 sensor
#include <TFT_eSPI.h>        // for tft display
#include "SPIFFS.h"          // for file flash system upload
#include "time.h"            // for current time and date
#include "TaskScheduler.h"   // for mimic delay

// for setting pressure
#define SEALEVELPRESSURE_HPA (1013.25)

// defining the pins need to control and monitor
#define INPIN_1 14
#define INPIN_2 25
#define INPIN_3 26
#define INPIN_4 27

// declaration of functions for setup
void initSPIFFS();
void pinSetup();
void bmeSetup();
void timeSetup();
void espNowSetup();
void tftSetup();

// declaration of functions for performing
void countDownTimer();
void readPins();
void OnDataSent();
void printLocalTime();
void espNowSend();
void refresh_readings();
void tftDisplay();
void detectChange();

// Local WiFi Credentials
const char *ssid = "Hidden_network";
const char *password = "pak.awan.pk";

// time variable setup
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -25362;
const int daylightOffset_sec = 3600;

// count down timer
int count = 10;

// sensor variable
Adafruit_BME280 bme;

// RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xF9, 0x0E, 0x98};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message
{
  int id; // must be unique for each sender board
  int pinStatus[4];
  float temperature;
  float humidity;
  float pressure;
  float altitude;
} struct_message;

//Create a struct_message called myData
struct_message myData;

// Register peer
esp_now_peer_info_t peerInfo;

// tft is global to this file only
TFT_eSPI tft = TFT_eSPI();

// for tft background and font-background color
uint16_t bg = TFT_BLACK;
uint16_t fg = TFT_WHITE;

// Setup tasks for the task scheduler
Task dataCommunication(20000, TASK_FOREVER, &espNowSend);
Task dataDisplayTFT(9800, TASK_FOREVER, &tftDisplay);
Task dataScheduler(1000, TASK_FOREVER, &countDownTimer);

// Create the scheduler
Scheduler runner;

// main setup function
void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);

  initSPIFFS();
  bmeSetup();
  timeSetup();
  pinSetup();
  tftSetup();
  espNowSetup();

  // Start the task scheduler
  runner.init();

  // Add the task to the scheduler
  runner.addTask(dataCommunication);
  runner.addTask(dataDisplayTFT);
  runner.addTask(dataScheduler);

  // Enable the task
  dataCommunication.enable();
  dataDisplayTFT.enable();
  dataScheduler.enable();
}

// the loop functions
void loop()
{
  // Execute the scheduler runner
  runner.execute();

  // detects any change on the inputs and respond accordingly
  detectChange();
}

// SPIFFS Initialization
void initSPIFFS()
{
  if (!SPIFFS.begin())
  {
    Serial.println("Cannot mount SPIFFS volume...");
    while (1)
      ; // infinite loop
  }
  else
  {
    Serial.println("SPIFFS volume mounted properly");
  }
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
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
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
  bool status = bme.begin(BME280_ADDRESS_ALTERNATE);
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
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

// setting up tft
void tftSetup()
{
  // Setup the TFT
  tft.begin();
  tft.setRotation(3);
  tft.loadFont("NotoSansBold20");
  tft.setTextColor(fg, bg);
  tft.fillScreen(bg);
  tft.setCursor(0, 0);
  tft.println("Hello!");
  tft.println("Searching for the sensor...");
}

// count down timer on the tft
void countDownTimer()
{
  tft.fillRect(279, 4, 13, 18, bg);
  tft.setCursor(280, 5);
  tft.print(--count == -1 ? 9 : count);
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

  // display time
  printLocalTime();

  // reading Pins
  readPins();

  // reading from BME
  refresh_readings();

  // displaying on the serial output
  Serial.println();
  Serial.printf("Temperature: %-6.2f °C", myData.temperature);
  Serial.println();
  Serial.printf("Humidity:    %-6.2f %%", myData.humidity);
  Serial.println();
  Serial.printf("Pressure:    %-6.2f hPa", myData.pressure);
  Serial.println();
  Serial.printf("Altitude:    %-6.2f m", myData.altitude);
  Serial.println();
  Serial.println();

  Serial.print("My MAC Address:  ");
  Serial.println(WiFi.macAddress());

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
  // reading the readings from the sensor
  myData.temperature = bme.readTemperature();
  myData.humidity = bme.readHumidity();
  myData.pressure = bme.readPressure() / 100.0F;
  myData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

// displaying on tft
void tftDisplay()
{
  // If you set this, the TFT will not work!!!
  count = 10;

  uint16_t bg = TFT_BLACK;
  uint16_t fg = TFT_WHITE;

  // displaying on the TFT
  tft.setCursor(5, 5);
  tft.setTextColor(fg, bg);
  // Create TTF fonts using instructions at https://pages.uoregon.edu/park/Processing/process5.html
  tft.loadFont("NotoSansBold20");
  tft.print("Right now, next update in: ");
  tft.fillRect(5, 100, 30, 30, bg);
  //  tft.setCursor(5, 100);
  //  tft.println(count);

  tft.setTextColor(TFT_YELLOW, bg);

  // Temperature
  tft.fillRect(10, 30, 250, 30, bg);
  tft.setCursor(10, 30);
  tft.printf("Temperature:");
  tft.setCursor(160, 30);
  tft.print(myData.temperature);
  tft.println("  °C");

  // Humidity
  tft.fillRect(10, 45, 250, 30, bg);
  tft.setCursor(10, 55);
  tft.print("Humidity:");
  tft.setCursor(160, 55);
  tft.print(myData.humidity);
  tft.println("   %");

  // Pressure
  tft.fillRect(10, 80, 250, 30, bg);
  tft.setCursor(10, 80);
  tft.print("Pressure:");
  tft.setCursor(160, 80);
  tft.print(myData.pressure);
  tft.println(" hPa");

  // Appx altitude
  tft.fillRect(10, 105, 250, 30, bg);
  tft.setCursor(10, 105);
  tft.print("Altitude:");
  tft.setCursor(160, 105);
  tft.print(myData.altitude);
  tft.println("   m");

  // INPUT 14
  tft.fillRect(10, 130, 250, 30, bg);
  tft.setCursor(10, 130);
  tft.print("Light:");
  tft.setCursor(160, 130);
  tft.println(myData.pinStatus[0] ? "ON" : "OFF");

  // INPUT 210
  tft.fillRect(10, 155, 250, 30, bg);
  tft.setCursor(10, 155);
  tft.print("Furnace:");
  tft.setCursor(160, 155);
  tft.println(myData.pinStatus[1] ? "ON" : "OFF");

  // INPUT 26
  tft.fillRect(10, 180, 250, 30, bg);
  tft.setCursor(10, 180);
  tft.print("Exhaust:");
  tft.setCursor(160, 180);
  tft.println(myData.pinStatus[2] ? "ON" : "OFF");

  // INPUT 27
  tft.fillRect(10, 205, 250, 30, bg);
  tft.setCursor(10, 205);
  tft.print("Humidifier:");
  tft.setCursor(160, 205);
  tft.println(myData.pinStatus[3] ? "ON" : "OFF");
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
    dataDisplayTFT.disable();
    dataScheduler.disable();

    // Enable the task
    dataCommunication.enable();
    dataDisplayTFT.enable();
    dataScheduler.enable();

    // Update the change
    delay(1000);
  }
}