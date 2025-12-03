#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
//#include <HardwareSerial.h>

HardwareSerial GPS(2);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  GPS.begin(9600, SERIAL_8N1, 16, 17);
}

void sendAlert(double lat, double lng) {
    StaticJsonDocument<200> doc;

    doc["latitude"] = lat;
    doc["longitude"] = lng;

    String link = "https://www.google.com/maps?q=" + String(lat, 6) + "," + String(lng, 6);
    doc["google_maps"] = link;

    String output;
    serializeJsonPretty(doc, output);

    Serial.println(output);
}

void loop() {
  while (GPS.available()) {
    char c=GPS.read();
    gps.encode(c);
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    sendAlert(lat, lng);
  delay(1000);
  }
  
  }
}