#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <LiquidCrystal_I2C.h>
#include <arduinoFFT.h>

#define MPU_ADDR 0x68
#define I2C_SLAVE_ADDR 8
#define RX 16
#define TX 17

#define NORMAL_VOLT 220
#define UNDER_VOLT 200
#define OVER_VOLT 250

#define NORMAL_CURRENT 4550 // For 1kW motor at 220V in mA
#define OVER_CURRENT 5000   // Overcurrent threshold in mA

using namespace websockets;

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// WiFi
const char *ssid = "Home";
const char *password = "353Arm52@89";

// WebSocket server
const char *websocket_server = "192.168.0.109";
const int websocket_port = 82;

WebsocketsClient socket;
bool connected = false;

// Sensor variables
float PV1, PV2, PV3 = 0;
float PI1, PI2, PI3 = 0;
float TEMP = 0;
int16_t RPM = 0;

String PVI = "NORMAL";

// MPU6050 data
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

String inputString = "";
bool newData = false;

unsigned long lastVTime = 0, lastITime = 0, lastTTime = 0, lastRTime = 0, lastSendTime = 0, lastLCDTime = 0;

void connectWiFi();
void connectToWebSocket();
void requestVoltageData();
void requestCurrentData();
void requestRpmData();
void requestTempData();
void readSerialData();
void readMPU6050();
void updateLCD();
String checkPhaseStatus();
String checkRPMStatus();

// --- Data Parser ---
String parseData(String data, int index) {
  int separatorIndex = data.indexOf(':');
  if (separatorIndex == -1) return "";
  return (index == 1) ? data.substring(0, separatorIndex) : data.substring(separatorIndex + 1);
}

// --- WebSocket Handlers ---
void handleMessage(WebsocketsMessage message) {
  String data = message.data();
  Serial.println("Received: " + data);
  Serial2.println(data);
}

void handleEvent(WebsocketsEvent event, WSInterfaceString data) {
  switch (event) {
    case WebsocketsEvent::ConnectionOpened:
      Serial.println("WebSocket connected");
      socket.send("ESP32 connected: " + WiFi.localIP().toString());
      connected = true;
      break;
    case WebsocketsEvent::ConnectionClosed:
      Serial.println("WebSocket closed");
      connected = false;
      break;
    default:
      break;
  }
}

// --- Connect WiFi ---
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
}

// --- Connect WebSocket ---
void connectToWebSocket() {
  String wsUrl = "ws://" + String(websocket_server) + ":" + String(websocket_port) + "/";
  connected = socket.connect(wsUrl.c_str());
  if (connected) {
    Serial.println("WebSocket connected");
  } else {
    Serial.println("WebSocket connection failed");
  }
}

// --- Request Data from I2C Slave ---
void requestVoltageData() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('V');
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLAVE_ADDR, 32);

  char buffer[32] = {0};
  Wire.readBytes(buffer, sizeof(buffer) - 1);
  sscanf(buffer, "%f,%f,%f", &PV1, &PV2, &PV3);
}

void requestCurrentData() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('I');
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLAVE_ADDR, 32);

  char buffer[32] = {0};
  Wire.readBytes(buffer, sizeof(buffer) - 1);
  sscanf(buffer, "%f,%f,%f", &PI1, &PI2, &PI3);
}

void requestRpmData() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('R');
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLAVE_ADDR, 8);

  char buffer[8] = {0};
  Wire.readBytes(buffer, sizeof(buffer) - 1);
  RPM = atoi(buffer);
}

void requestTempData() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('T');
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLAVE_ADDR, 4);

  char buffer[4] = {0};
  Wire.readBytes(buffer, sizeof(buffer) - 1);
  TEMP = atoi(buffer);
}

// --- Read Serial2 Commands ---
void readSerialData() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      newData = true;
      break;
    } else {
      inputString += c;
    }
  }
}

// --- MPU6050 Read Raw ---
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// --- Check RPM Status ---
String checkRPMStatus(){
  String RPMStatus = "";
  if(RPM < 0){
    RPMStatus = "R";
  }else if (RPM > 0){
    RPMStatus = "F";
  }else{
    RPMStatus = "OFF";
  }
  return RPMStatus;
}

// --- Check Phase Status ---
String checkPhaseStatus() {
  bool p1VoltageIssue = false, p2VoltageIssue = false, p3VoltageIssue = false;
  bool p1CurrentIssue = false, p2CurrentIssue = false, p3CurrentIssue = false;
  String voltageIssues = "";
  String currentIssues = "";
  
  // Check voltage issues
  if (PV1 > OVER_VOLT || PV1 < UNDER_VOLT) {
    p1VoltageIssue = true;
    voltageIssues += (PV1 > OVER_VOLT) ? "P1OV " : "P1UV ";
  }
  if (PV2 > OVER_VOLT || PV2 < UNDER_VOLT) {
    p2VoltageIssue = true;
    voltageIssues += (PV2 > OVER_VOLT) ? "P2OV " : "P2UV ";
  }
  if (PV3 > OVER_VOLT || PV3 < UNDER_VOLT) {
    p3VoltageIssue = true;
    voltageIssues += (PV3 > OVER_VOLT) ? "P3OV " : "P3UV ";
  }
  
  // Check current issues (convert to mA for comparison)
  if (PI1 * 1000 > OVER_CURRENT) {
    p1CurrentIssue = true;
    currentIssues += "P1OI ";
  }
  if (PI2 * 1000 > OVER_CURRENT) {
    p2CurrentIssue = true;
    currentIssues += "P2OI ";
  }
  if (PI3 * 1000 > OVER_CURRENT) {
    p3CurrentIssue = true;
    currentIssues += "P3OI ";
  }
  
  // Count voltage and current issues
  int voltageIssueCount = (p1VoltageIssue ? 1 : 0) + (p2VoltageIssue ? 1 : 0) + (p3VoltageIssue ? 1 : 0);
  int currentIssueCount = (p1CurrentIssue ? 1 : 0) + (p2CurrentIssue ? 1 : 0) + (p3CurrentIssue ? 1 : 0);
  
  String status = "";
  
  // Handle voltage issues
  if (voltageIssueCount >= 2) {
    // Check if all are overvoltage or all are undervoltage
    bool allOverVolt = (PV1 > OVER_VOLT && PV2 > OVER_VOLT && PV3 > OVER_VOLT);
    bool allUnderVolt = (PV1 < UNDER_VOLT && PV2 < UNDER_VOLT && PV3 < UNDER_VOLT);
    
    if (allOverVolt) {
      status += "APOV ";
    } else if (allUnderVolt) {
      status += "APUV ";
    } else {
      status += "APOV "; // Mixed issues, show as all phases overvoltage
    }
  } else if (voltageIssueCount == 1) {
    status += voltageIssues;
  }
  
  // Handle current issues
  if (currentIssueCount >= 2) {
    status += "APOI ";
  } else if (currentIssueCount == 1) {
    status += currentIssues;
  }
  
  // If no issues, show NORMAL
  if (voltageIssueCount == 0 && currentIssueCount == 0) {
    status = "NORMAL";
  }
  
  // Trim any trailing space
  status.trim();
  
  return status;
}

// --- LCD Display Update ---
void updateLCD() {
  static char prevLine1[21] = "";
  static char prevLine2[21] = "";
  static char prevLine3[21] = "";
  static char prevLine4[21] = "";

  char line1[21], line2[21], line3[21], line4[21];

  // Get phase status
  String phaseStatus = checkPhaseStatus();
  String rpmStatus   = checkRPMStatus();
  if(rpmStatus == "R"){
    RPM = RPM*(-1);
  }

  // Format the lines (always null-terminated)
  snprintf(line1, sizeof(line1), "PV:%.1f %.1f %.1f", PV1, PV2, PV3);
  snprintf(line2, sizeof(line2), "PI:%.1f %.1f %.1f", PI1, PI2, PI3);
  snprintf(line3, sizeof(line3), "RPM:%4d %s T:%3.0f", RPM,rpmStatus, TEMP);
  snprintf(line4, sizeof(line4), "ST: %s", phaseStatus.c_str());

  // Pad each string to 20 chars (LCD width) if needed
  while (strlen(line1) < 20) strcat(line1, " ");
  while (strlen(line2) < 20) strcat(line2, " ");
  while (strlen(line3) < 20) strcat(line3, " ");
  while (strlen(line4) < 20) strcat(line4, " ");

  // Only update if the line has changed
  if (strcmp(line1, prevLine1) != 0) {
    lcd.setCursor(0, 0);
    lcd.print(line1);
    strcpy(prevLine1, line1);
  }

  if (strcmp(line2, prevLine2) != 0) {
    lcd.setCursor(0, 1);
    lcd.print(line2);
    strcpy(prevLine2, line2);
  }

  if (strcmp(line3, prevLine3) != 0) {
    lcd.setCursor(0, 2);
    lcd.print(line3);
    strcpy(prevLine3, line3);
  }

  if (strcmp(line4, prevLine4) != 0) {
    lcd.setCursor(0, 3);
    lcd.print(line4);
    strcpy(prevLine4, line4);
  }
}

// --- Setup ---
void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);
  lcd.init(); lcd.backlight();

  // MPU6050 init
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  connectWiFi();
  socket.onMessage(handleMessage);
  socket.onEvent(handleEvent);
  connectToWebSocket();

  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
}

void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!connected) connectToWebSocket();

  socket.poll();
  readSerialData();

  if (newData) {
    if (connected) socket.send(inputString);
    Serial.println("Serial2 Msg: " + inputString);
    inputString = "";
    newData = false;
  }

  if (now - lastVTime > 1500) {
    requestVoltageData();
    lastVTime = now;
  }

  if (now - lastITime > 1000) {
    requestCurrentData();
    lastITime = now;
  }

  if (now - lastTTime > 500) {
    requestTempData();
    lastTTime = now;
  }

  if (now - lastRTime > 100) {
    requestRpmData();
    readMPU6050(); // Also read vibration data here
    lastRTime = now;
  }

  if (now - lastSendTime > 250 && connected) {
    String payload = String(TEMP) + ",0," + String(RPM) + "," +
                     String(PV1) + "," + String(PI1) + "," +
                     String(PV2) + "," + String(PI2) + "," +
                     String(PV3) + "," + String(PI3);
    socket.send(payload + ":SENSORDATA");
    Serial.println("Sent WS: " + payload);
    lastSendTime = now;
  }

  if (now - lastLCDTime > 1000) {
    updateLCD();
    lastLCDTime = now;
  }
}