#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <LiquidCrystal_I2C.h>
#include <arduinoFFT.h>
#include <ArduinoJson.h>

#define MPU_ADDR 0x68
#define I2C_SLAVE_ADDR 0x08
#define I2C_RPM_ADDR 0x07
#define RX 16
#define TX 17

#define NORMAL_VOLT 220
#define UNDER_VOLT 200
#define OVER_VOLT 230

#define NORMAL_CURRENT 500 // For 1kW motor at 220V in mA
#define OVER_CURRENT 600   // Overcurrent threshold in mA
#define MULTIPLIER_CURRENT_OV 1

// FFT parameters
const uint16_t FFT_SAMPLES = 256; // Must be power of 2
const double SAMPLING_FREQ = 2500.0; // Hz
const double ACC_SCALE = 16384.0; // For ±2g
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

using namespace websockets;

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// WiFi
const char *ssid = "Home"; // ESP
const char *password = "353Arm52@89"; // tls1985op 2.4GHz

// WebSocket server
const char *websocket_server = "192.168.0.109"; //
const int websocket_port = 82;

WebsocketsClient socket;
bool connected = false;

// Sensor variables
float PV1, PV2, PV3 = 0;
float PI1, PI2, PI3 = 0;
float TEMP = 0;
int16_t RPM = 0;
double PF = 0;

String PVI = "NORMAL";

// MPU6050 data
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// FFT objects
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQ);
unsigned long lastFFTMicros = 0;
unsigned long lastFFTSend = 0;
const unsigned long FFT_SEND_INTERVAL = 500; // Send FFT data every 500ms

String inputString = "";
bool newData = false;

unsigned long lastVTime = 0, lastITime = 0, lastTTime = 0, lastRTime = 0, lastSendTime = 0, lastLCDTime = 0, lastPFtime = 0;

// Function declarations
void connectWiFi();
void connectToWebSocket();
void requestVoltageData();
void requestCurrentData();
void requestPowerFactorData();
void requestRpmData();
void requestTempData();
void readSerialData();
void readMPU6050();

void updateLCD();

String checkPhaseStatus();
String checkRPMStatus();
String parseData(String data, int index);

void handleMessage(WebsocketsMessage message);
void handleEvent(WebsocketsEvent event, WSInterfaceString data);

void collectFFTSamples();
void sendFFTData();

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

void requestPowerFactorData() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write('P');
  Wire.endTransmission();
  Wire.requestFrom(I2C_SLAVE_ADDR, 8);

  char buffer[8] = {0};
  Wire.readBytes(buffer, sizeof(buffer) - 1);
  PF = strtof(buffer,NULL);
}

void requestRpmData() {
  Wire.beginTransmission(I2C_RPM_ADDR);
  Wire.write('R');
  Wire.endTransmission();
  Wire.requestFrom(I2C_RPM_ADDR, 8);

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
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// --- FFT Sample Collection ---
void collectFFTSamples() {
  for (int i = 0; i < FFT_SAMPLES; i++) {
    while (micros() - lastFFTMicros < (1000000 / SAMPLING_FREQ)); // Wait for sampling interval
    lastFFTMicros = micros();
    
    // Read accelerometer data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (bool)true);
    
    int16_t axRaw = Wire.read() << 8 | Wire.read();
    int16_t ayRaw = Wire.read() << 8 | Wire.read();
    int16_t azRaw = Wire.read() << 8 | Wire.read();
    
    // Convert to g values
    double ax = axRaw / ACC_SCALE;
    double ay = ayRaw / ACC_SCALE;
    double az = azRaw / ACC_SCALE;
    
    // Calculate acceleration magnitude
    double accMagnitude = sqrt(ax * ax + ay * ay + az * az);
    
    vReal[i] = accMagnitude;
    vImag[i] = 0;
  }
}

// --- Send FFT Data ---
void sendFFTData() {
  // Perform FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  
  // Create JSON object with integer frequencies and magnitude data
  DynamicJsonDocument doc(8192);
  JsonArray frequencies = doc.createNestedArray("frequencies");
  JsonArray magnitudes = doc.createNestedArray("magnitudes");
  double SCALE_FACTOR = 3.5;
  // Send frequency bins from 1Hz up to 1000Hz with integer frequencies
  for (int i = 1; i < FFT_SAMPLES / 2; i++) {
    int freq = (int)(i * SAMPLING_FREQ / FFT_SAMPLES); // Convert to integer
    if (freq > 1000) break;
    double scaledMagnitude = vReal[i] / SCALE_FACTOR;
    frequencies.add(freq);
    magnitudes.add(scaledMagnitude);
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Send FFT data with a different identifier
  socket.send(jsonString + ":FFTDATA");
  Serial.println("Sent FFT data");
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
/* String checkPhaseStatus() {
  bool p1VoltageIssue = false, p2VoltageIssue = false, p3VoltageIssue = false;
  bool p1CurrentIssue = false, p2CurrentIssue = false, p3CurrentIssue = false;
  String voltageIssues = "";
  String currentIssues = "";
  
  // Check voltage issues
  if (PV1 > OVER_VOLT || PV1 < UNDER_VOLT) {
    p1VoltageIssue = true;
    voltageIssues += (PV1 > OVER_VOLT) ? "P1OV " : "P1UV "; // ternary operator
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
  if (PI1 * MULTIPLIER_CURRENT_OV > OVER_CURRENT) {
    p1CurrentIssue = true;
    currentIssues += "P1OI ";
  }
  if (PI2 * MULTIPLIER_CURRENT_OV> OVER_CURRENT) {
    p2CurrentIssue = true;
    currentIssues += "P2OI ";
  }
  if (PI3 * MULTIPLIER_CURRENT_OV > OVER_CURRENT) {
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
 */

 String checkPhaseStatus() {
  bool p1VoltageIssue = false, p2VoltageIssue = false, p3VoltageIssue = false;
  bool p1CurrentIssue = false, p2CurrentIssue = false, p3CurrentIssue = false;
  String voltageIssues = "";
  String currentIssues = "";
  
  // --- 1. DETECT ISSUES (Original Logic) ---

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
  
  // Check current issues
  if (PI1 * MULTIPLIER_CURRENT_OV > OVER_CURRENT) {
    p1CurrentIssue = true;
    currentIssues += "P1OI ";
  }
  if (PI2 * MULTIPLIER_CURRENT_OV > OVER_CURRENT) {
    p2CurrentIssue = true;
    currentIssues += "P2OI ";
  }
  if (PI3 * MULTIPLIER_CURRENT_OV > OVER_CURRENT) {
    p3CurrentIssue = true;
    currentIssues += "P3OI ";
  }
  
  // Count issues
  int voltageIssueCount = (p1VoltageIssue ? 1 : 0) + (p2VoltageIssue ? 1 : 0) + (p3VoltageIssue ? 1 : 0);
  int currentIssueCount = (p1CurrentIssue ? 1 : 0) + (p2CurrentIssue ? 1 : 0) + (p3CurrentIssue ? 1 : 0);
  
  // --- 2. BUILD DETAILED STRING FOR LCD (Original Logic) ---
  String lcdStatus = "";
  
  // Format Voltage String
  if (voltageIssueCount >= 2) {
    bool allOverVolt = (PV1 > OVER_VOLT && PV2 > OVER_VOLT && PV3 > OVER_VOLT);
    bool allUnderVolt = (PV1 < UNDER_VOLT && PV2 < UNDER_VOLT && PV3 < UNDER_VOLT);
    
    if (allOverVolt) lcdStatus += "APOV ";
    else if (allUnderVolt) lcdStatus += "APUV ";
    else lcdStatus += "APOV "; // Mixed
  } else if (voltageIssueCount == 1) {
    lcdStatus += voltageIssues;
  }
  
  // Format Current String
  if (currentIssueCount >= 2) {
    lcdStatus += "APOI ";
  } else if (currentIssueCount == 1) {
    lcdStatus += currentIssues;
  }
  
  if (voltageIssueCount == 0 && currentIssueCount == 0) {
    lcdStatus = "NORMAL";
  }
  
  lcdStatus.trim();

  // --- 3. SEND SHORT CODE TO SERIAL2 (New Logic) ---
  // Determine the simple code: VOF (Voltage Fault), IOF (Current Fault), or NORM
  String serialCode = "NORM";

  if (voltageIssueCount > 0 && currentIssueCount < 1) {
    serialCode = "VOF";
  } 
  if (currentIssueCount > 0 && voltageIssueCount  < 1) {
    serialCode = "IOF";
  }
  if (voltageIssueCount > 0 && currentIssueCount > 0) {
    serialCode = "BOTH";
  } 


  // Anti-jamming: Only send if the code has changed
  static String lastSentCode = "";

  if (serialCode != lastSentCode) {
    Serial2.println(serialCode); // Send to external device
    Serial.println("Sent Status: " + serialCode); // Debug on main serial
    lastSentCode = serialCode;
  }
  
  // --- 4. RETURN DETAILED STRING FOR LCD ---
  return lcdStatus;
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
  snprintf(line3, sizeof(line3), "PF:%.2f T:%3.0f",PF, TEMP);
  snprintf(line4, sizeof(line4), "ST:%sRPM:%d", phaseStatus.c_str(),RPM);

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

  lcd.init(); 
  lcd.backlight();

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
  
  // Initialize FFT timing
  lastFFTMicros = micros();
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

  if (now - lastVTime > 1000) {
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
    lastRTime = now;
  }

   if (now - lastPFtime > 1000) {
    requestPowerFactorData();
    lastPFtime = now;
  }

  // FFT data collection and sending
  if (now - lastFFTSend > FFT_SEND_INTERVAL && connected) {
    collectFFTSamples();
    sendFFTData();
    lastFFTSend = now;
  }

  if (now - lastSendTime > 250 && connected) {
    String payload = String(TEMP)+","+String(RPM)+","+ String(PF) + "," +
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