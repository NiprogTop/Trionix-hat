#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);

}

String parseSDDBT(String sentence) {
  if (sentence.startsWith("$SDDBT")) {
      // Parse the NMEA sentence
      Serial.println(sentence);
        int commaIndex1 = sentence.indexOf(',');
        int commaIndex2 = sentence.indexOf(',', commaIndex1 + 1);
        int commaIndex3 = sentence.indexOf(',', commaIndex2 + 1);
        int commaIndex4 = sentence.indexOf(',', commaIndex3 + 1);
        int commaIndex5 = sentence.indexOf(',', commaIndex4 + 1);

        String depthStr = sentence.substring(commaIndex3+1, commaIndex4);
        return depthStr;
     
    }
 return "Invalid data";  
}



void loop() {
  if (Serial2.available()) {
    // Read the incoming data
    String nmeaSentence = Serial2.readStringUntil('\n');


    // Check if the sentence starts with "$SDDBT"
    if (nmeaSentence.startsWith("$SDDBT")) {
      // Parse the NMEA sentence
      String depthMeters = parseSDDBT(nmeaSentence);
      Serial.println(nmeaSentence);

      //Print the depth in meters to the Serial monitor
      //if (depthMeters >= 0) {
        Serial.print("Depth: ");
        Serial.print(depthMeters);
        Serial.println(" meters");
     
    }
  }
}

String parseSDDBT(String sentence) {
  // Split the sentence into parts based on commas
  int commaIndex1 = sentence.indexOf(',');
  int commaIndex2 = sentence.indexOf(',', commaIndex1 + 1);
  int commaIndex3 = sentence.indexOf(',', commaIndex2 + 1);
  int commaIndex4 = sentence.indexOf(',', commaIndex3 + 1);
  int commaIndex5 = sentence.indexOf(',', commaIndex4 + 1);

  // Extract the depth in meters (between the 4th and 5th commas)
  String depthStr = sentence.substring(commaIndex3+1, commaIndex4);

  // Convert the depth string to a float
  return depthStr;
}