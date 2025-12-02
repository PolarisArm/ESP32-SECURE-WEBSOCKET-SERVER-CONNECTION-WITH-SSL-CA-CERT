/*#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <stdint.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <string.h>
#include <time.h>

#define I2C_SLAVE_ADDR 8
#define RX 16
#define TX 17

using namespace websockets;

WebsocketsClient socket;

// WiFi credentials
const char *ssid = "Home";
const char *password = "353Arm52@89";

// WebSocket server configuration
const char* websocket_server = "192.168.0.109";  // Your XAMPP server IP
const int websocket_port = 82;                    // Your server port

// Connection and sensor variables
boolean connected = false;
float PV1, PV2, PV3, PI1, PI2, PI3, TEMP = 0;
int16_t RPM = 0;

String inputString = "";    
bool newData = false;        // Flag to indicate a complete message

String parseData(String data, int index)
{
  int separatorIndex = data.indexOf(':');
  if (separatorIndex == -1)
  {
    return ""; // Return empty string if no ':' found
  }

  if (index == 1)
  {
    return data.substring(0, separatorIndex); // Return part before ':'
  }
  else if (index == 2)
  {
    return data.substring(separatorIndex + 1); // Return part after ':'
  }

  return ""; // Return empty string for out-of-bounds index
}

void handleMessage(WebsocketsMessage message)
{
  String data = message.data();
  Serial.println("Received: " + data);
  Serial2.println(data);

  // Handle welcome message from PHP server
  if (data == "Welcome to server") {
    Serial.println("Successfully connected to PHP WebSocket server");
    return;
  }

  String status = parseData(data, 1);
  String sensor = parseData(data, 2);
}
  
void handleEvent(WebsocketsEvent event, WSInterfaceString data)
{
  switch (event)
  {
  case WebsocketsEvent::ConnectionOpened:
    Serial.println("WebSocket connection established");
    socket.send("ESP32 connected: " + WiFi.localIP().toString());
    connected = true;
    break;
  case WebsocketsEvent::ConnectionClosed:
    Serial.println("WebSocket connection closed");
    connected = false;
    break;
  case WebsocketsEvent::GotPing:
    Serial.println("Received ping");
    break;
  case WebsocketsEvent::GotPong:
    Serial.println("Received pong");
    break;
  default:
    break;
  }
}

void connectToWebSocket()
{
  Serial.println("Attempting to connect to PHP WebSocket server...");
  Serial.print("Server: ");
  Serial.print(websocket_server);
  Serial.print(":");
  Serial.println(websocket_port);
  
  // Build WebSocket URL for local server (no SSL)
  String websocketUrl = "ws://" + String(websocket_server) + ":" + String(websocket_port) + "/";
  
  connected = socket.connect(websocketUrl.c_str());
  if (connected)
  {
    Serial.println("Connected to PHP WebSocket server!");
  }
  else
  {
    Serial.println("Connection failed.");
  }
}

void connectWiFi()
{
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.print("Connected to: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void requestVoltageData()
{
  int dataindex = 0;
  char incomingData[32];
  char buf[20];
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('V');
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLAVE_ADDR, 32);

  while(Wire.available())
  {
    char c = Wire.read();

    if(dataindex < 32 - 1)
    {
      incomingData[dataindex++] = c;
    }
  }
  incomingData[dataindex] = '\0';

  dataindex = 0;
  char *token = strtok(incomingData, ",");
  if(token != NULL)
  {
    PV1 = atoi(token);
    token = strtok(NULL,",");
  }
  if(token != NULL)
  {
    PV2 = atoi(token);
    token = strtok(NULL,",");
  }
  if(token != NULL)
  {
    PV3 = atoi(token);
    token = strtok(NULL,",");
  }
}

void requestCurrentData()
{
  int dataindex = 0;
  char incomingData[32];
  char buf[20];
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('I');
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLAVE_ADDR,32);

  while (Wire.available() && dataindex < 32 - 1)
  {
    char c = Wire.read();
      incomingData[dataindex++] = c;
  }
    incomingData[dataindex] = '\0';

    dataindex = 0;

  char *token = strtok(incomingData, ",");
  if(token != NULL)
  {
    PI1 = atof(token);
    token = strtok(NULL,",");
  }
  if(token != NULL)
  {
    PI2 = atof(token);
    token = strtok(NULL,",");
  }
  if(token != NULL)
  {
    PI3 = atof(token);
    token = strtok(NULL,",");
  }
}

void requestRpmData()
{
  int dataindex = 0;
  char incomingData[16];

  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('R');
  Wire.endTransmission();

  Wire.requestFrom(I2C_SLAVE_ADDR, 8);

  while(Wire.available())
  {
    char c = Wire.read();
    if(dataindex < 8-1)
    {
      incomingData[dataindex++] = c;
    }
  }

  incomingData[dataindex] = '\0';
  dataindex = 0;
  RPM = atoi(incomingData);
}

void requestTempData()
{
  int dataindex = 0;
  char incomingData[16];

  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('T');
  Wire.endTransmission();

  Wire.requestFrom(I2C_SLAVE_ADDR, 4);

  while(Wire.available())
  {
    char c  = Wire.read();
    if(dataindex < 4 -1)
    {
      incomingData[dataindex++] = c;
    }
  }

  incomingData[dataindex] = '\0';
  dataindex = 0;
  TEMP = atoi(incomingData);
}

void readSerialData() {
  while (Serial2.available() > 0) {
    char incomingChar = Serial2.read();  // Read each incoming character

    // Check for end-of-line (newline character) or another delimiter
    if (incomingChar == '\n') {
      newData = true;  // Set flag indicating a complete message is received
      break;
    } else {
      inputString += incomingChar;  // Accumulate the character in the string
    }
  }
}

unsigned long previousTempMillis, previousVMillis, previousIMillis, previousRMillis, previousSMillis = 0;
unsigned long tempInterval = 500;
unsigned long voltageInterval = 1500;
unsigned long IInterval = 1000;
unsigned long RInterval = 100;
unsigned long SendInterval = 100;

void setup()
{
  Wire.begin(21,22); // SCL, SDA
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);  // RX=16, TX=17

  Serial.println("\n\n=== ESP32 WebSocket Client Starting ===");
  Serial.println("Connecting to PHP WebSocket Server");
  
  connectWiFi();
  
  socket.onMessage(handleMessage);
  socket.onEvent(handleEvent);
  
  connectToWebSocket();
}

void loop()
{
  unsigned long currentMillis = millis();
  readSerialData();
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
  }
  
  // Check WebSocket connection
  if (!connected)
  {
    Serial.println("Connecting to WebSocket server");
    connectToWebSocket();
  }
  
  socket.poll();

  if (newData) {
    Serial.println("Received from Serial2: " + inputString);
    inputString.trim();
    
    // Send data to PHP WebSocket server
    if (connected) {
      socket.send(inputString);
      Serial.println("Sent to WebSocket: " + inputString);
    }
    
    // Example: Check if the message is "SDF:0"
    if (inputString == "SDF:0") {
      Serial.println("Command recognized: SDF:0");
      // Add any additional handling code here for "SDF:0"
    }

    // Clear the string and reset the flag for the next message
    inputString = "";
    newData = false;
  }


  if(currentMillis - previousVMillis >= voltageInterval)
  {
    requestVoltageData();
    previousVMillis = currentMillis;
  }
  if(currentMillis - previousIMillis >= IInterval)
  {
    requestCurrentData();
    previousIMillis = currentMillis;
  }
  if(currentMillis - previousTempMillis >= tempInterval)
  {
    requestTempData();
    previousTempMillis = currentMillis;
  }
  if(currentMillis - previousRMillis >= RInterval)
  {
    requestRpmData();
    previousRMillis = currentMillis;
  }

  if(currentMillis - previousSMillis >= 250)
  {
     String sensorData = String(TEMP)+","+"0"+","+String(RPM)+","+String(PV1)+","
                      +  String(PI1)+","+String(PV2)+","+String(PI2)+","
                      +  String(PV3)+","+String(PI3);
    if (connected) {
      socket.send(sensorData + ":SENSORDATA");
      Serial.println("Sent sensor data: " + sensorData);
    }
    previousSMillis = currentMillis;
  }

}*/
