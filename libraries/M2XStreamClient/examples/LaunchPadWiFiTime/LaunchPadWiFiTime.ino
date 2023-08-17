#include <aJSON.h>
#include "SPI.h"
#include "WiFi.h"

#define ENERGIA_PLATFORM
#include "M2XStreamClient.h"

char ssid[] = "<ssid>"; //  your network SSID (name)
char pass[] = "<WPA password>";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

char* deviceId = "<device id>"; // Device you want to push to
char* streamName = "<stream name>"; // Stream you want to push to
char* m2xKey = "<M2X access key>"; // Your M2X access key

char timestamp[25];

const int temperaturePin = A0;

WiFiClient client;
M2XStreamClient m2xClient(&client, m2xKey);
TimeService timeService(&m2xClient);

void printFloat(float value, int places);

float temp;

void setup() {
    Serial.begin(9600);

    // attempt to connect to Wifi network:
    Serial.print("Attempting to connect to Network named: ");
    // print the network name (SSID);
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.begin(ssid, pass);
    while ( WiFi.status() != WL_CONNECTED) {
      // print dots while we wait to connect
      Serial.print(".");
      delay(300);
    }

    Serial.println("\nYou're connected to the network");
    Serial.println("Waiting for an ip address");

    while (WiFi.localIP() == INADDR_NONE) {
      // print dots while we wait for an ip addresss
      Serial.print(".");
      delay(300);
    }

    Serial.println("\nIP Address obtained");

    // you're connected now, so print out the status
    printWifiStatus();

    if (!m2x_status_is_success(timeService.init())) {
      Serial.println("Cannot initialize time service!");
      while(1) {}
    }
}

void loop() {

  float voltage, degreesC, degreesF;

  voltage = getVoltage(temperaturePin);
  degreesC = (voltage - 0.5) * 100.0;
  degreesF = degreesC * (9.0 / 5.0) + 32.0;

  Serial.print("voltage: ");
  Serial.print(voltage);
  Serial.print("  deg C: ");
  Serial.print(degreesC);
  Serial.print("  deg F: ");
  Serial.println(degreesF);

  int length = 25;
  timeService.getTimestamp(timestamp, &length);

  Serial.print("Current timestamp: ");
  Serial.println(timestamp);

  char *timestamps[1];
  timestamps[0] = timestamp;

  int count = 1;
  int response = m2xClient.postDeviceUpdates<float>(
      deviceId, 1, (const char **) &streamName,
      &count, (const char **) timestamps, &degreesC);
  Serial.print("M2X client response code: ");
  Serial.println(response);

  if (response == -1)
    while (1)
      ;

  delay(5000);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

float getVoltage(int pin) {
  return (analogRead(pin) * 0.004882814 * 0.5); // normalize the voltage from LM35
}
