#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

WebsocketsClient socket;
WiFiClient espClient;

const char *ssid = "Home";
const char *password = "13227296";

boolean connected = false;

bool SWITCH_STATE = false;

unsigned long interval = 1000;
unsigned long previous, current, previous_two = 0;
unsigned long SDFMILLIS_CURR, SDFMILLIS_PREV = 0;

const char ssl_cert_one[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIEXjCCA0agAwIBAgITB3MSTNQG0mfAmRzdKZqfODF5hTANBgkqhkiG9w0BAQsF\n"
    "ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
    "b24gUm9vdCBDQSAxMB4XDTIyMDgyMzIyMjYwNFoXDTMwMDgyMzIyMjYwNFowPDEL\n"
    "MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEcMBoGA1UEAxMTQW1hem9uIFJT\n"
    "QSAyMDQ4IE0wMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALd/pVko\n"
    "8vuM475Tf45HV3BbCl/B9Jy89G1CRkFjcPY06WA9lS+7dWbUA7GtWUKoksr69hKM\n"
    "wcMsNpxlw7b3jeXFgxB09/nmalcAWtnLzF+LaDKEA5DQmvKzuh1nfIfqEiKCQSmX\n"
    "Xh09Xs+dO7cm5qbaL2hhNJCSAejciwcvOFgFNgEMR42wm6KIFHsQW28jhA+1u/M0\n"
    "p6fVwReuEgZfLfdx82Px0LJck3lST3EB/JfbdsdOzzzg5YkY1dfuqf8y5fUeZ7Cz\n"
    "WXbTjujwX/TovmeWKA36VLCz75azW6tDNuDn66FOpADZZ9omVaF6BqNJiLMVl6P3\n"
    "/c0OiUMC6Z5OfKcCAwEAAaOCAVowggFWMBIGA1UdEwEB/wQIMAYBAf8CAQAwDgYD\n"
    "VR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjAdBgNV\n"
    "HQ4EFgQUVdkYX9IczAHhWLS+q9lVQgHXLgIwHwYDVR0jBBgwFoAUhBjMhTTsvAyU\n"
    "lC4IWZzHshBOCggwewYIKwYBBQUHAQEEbzBtMC8GCCsGAQUFBzABhiNodHRwOi8v\n"
    "b2NzcC5yb290Y2ExLmFtYXpvbnRydXN0LmNvbTA6BggrBgEFBQcwAoYuaHR0cDov\n"
    "L2NydC5yb290Y2ExLmFtYXpvbnRydXN0LmNvbS9yb290Y2ExLmNlcjA/BgNVHR8E\n"
    "ODA2MDSgMqAwhi5odHRwOi8vY3JsLnJvb3RjYTEuYW1hem9udHJ1c3QuY29tL3Jv\n"
    "b3RjYTEuY3JsMBMGA1UdIAQMMAowCAYGZ4EMAQIBMA0GCSqGSIb3DQEBCwUAA4IB\n"
    "AQAGjeWm2cC+3z2MzSCnte46/7JZvj3iQZDY7EvODNdZF41n71Lrk9kbfNwerK0d\n"
    "VNzW36Wefr7j7ZSwBVg50W5ay65jNSN74TTQV1yt4WnSbVvN6KlMs1hiyOZdoHKs\n"
    "KDV2UGNxbdoBYCQNa2GYF8FQIWLugNp35aSOpMy6cFlymFQomIrnOQHwK1nvVY4q\n"
    "xDSJMU/gNJz17D8ArPN3ngnyZ2TwepJ0uBINz3G5te2rdFUF4i4Y3Bb7FUlHDYm4\n"
    "u8aIRGpk2ZpfXmxaoxnbIBZRvGLPSUuPwnwoUOMsJ8jirI5vs2dvchPb7MtI1rle\n"
    "i02f2ivH2vxkjDLltSpe2fiC\n"
    "-----END CERTIFICATE-----\n";

String mpu();

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

String generateRandomDhtData()
{
  // Generate random values for each part of the dhtData string
  int temperature1 = random(10, 70);          // First temperature, random between 60 and 70
  float temperature2 = random(1, 350) / 10.0; // Second temperature, random between 30.0 and 35.0
  int humidity = random(70, 90);              // Humidity, random between 70 and 90
  int sensor1 = random(210, 220);             // First sensor value, random between 210 and 220
  float value1 = random(20, 30) / 10.0;       // First floating value, random between 2.0 and 3.0
  int sensor2 = random(210, 220);             // Second sensor value, random between 210 and 220
  float value2 = random(20, 30) / 10.0;       // Second floating value, random between 2.0 and 3.0
  int sensor3 = random(210, 220);             // Third sensor value, random between 210 and 220
  float value3 = random(20, 30) / 10.0;       // Third floating value, random between 2.0 and 3.0

  // Create the dhtData string
  String sensorData = String(temperature1) + "," +
                      String(temperature2, 1) + "," +
                      String(humidity) + "," +
                      String(sensor1) + "," +
                      String(value1, 1) + "," +
                      String(sensor2) + "," +
                      String(value2, 1) + "," +
                      String(sensor3) + "," +
                      String(value3, 1);

  return sensorData;
}

String mpu()
{
  // Generate random values for each part of the dhtData string
  int mpuD = random(0, 10); // First temperature, random between 60 and 70

  // Create the dhtData string
  String sensorData = String(mpuD);

  return sensorData;
}

void handleMessage(WebsocketsMessage message)
{
  // Serial.println(message.data());
  String data = message.data();

  String status = parseData(data, 1);
  String sensor = parseData(data, 2);

  Serial.print("Status: ");
  Serial.println(status);
  Serial.print("Sensor: ");
  Serial.println(sensor);
  if (sensor == "SDF")
  {
    SWITCH_STATE = status.toInt();
  }
  
}

void handleEvent(WebsocketsEvent event, WSInterfaceString data)
{
  switch (event)
  {
  case WebsocketsEvent::ConnectionOpened:
    Serial.println("WebSocket connection established");
    socket.send("Hello, server!");
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
  socket.setCACert(ssl_cert_one);

  connected = socket.connect("wss://websockettestesp32.glitch.me/");
  if (connected)
  {
    Serial.println("Connected");
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
  // This line hides the viewing of ESP as wifi hotspot
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.print("connected to : ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  connectWiFi();
  connectToWebSocket();

  socket.onMessage(handleMessage);
  socket.onEvent(handleEvent);
}

void loop()
{
  current = millis();

  if (!connected)
  {
    Serial.println("Connecting to WebSocket server");
    connectToWebSocket();
  }
  socket.poll();

  if (current - previous >= interval)
  {
    String sensorData = generateRandomDhtData();
    socket.send(sensorData + ":SENSORDATA");
    previous = current;
  }
  if (current - previous_two >= 100)
  {
    String sensorData = mpu();
    socket.send(sensorData + ":VIB");
    previous_two = current;
  }
}
