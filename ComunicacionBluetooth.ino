
char oneT = '1';
char zeroT = '0';
float motionProfiles[10] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);

  // put your setup code here, to run once:

}

void loop() {
  readSerial();
  delay(500);
  // put your main code here, to run repeatedly:
}

void sendCommand(const char * command){
  Serial.print("Command send :");
  Serial.println(command);
  Serial2.println(command);
  delay(100);

  char reply[100];
  int i = 0;
  while(Serial2.available()){
    reply[i] = Serial2.read();
    i += 1;
  }
  reply[i] = '\0';
  Serial.print(reply);
  Serial.println("Reply end");
}

void readSerial(){
  String replyStr = "";
  char reply[200];
  int i = 0;
  while(Serial2.available()){
    reply[i] = Serial2.read();
    i = i+1;
  }
  bool startFlagIsDetected = false;
  String startFlag = "1010101000";
  String endFlag = "00100011";
  int startValuesFound = 0;
  int endValuesFound = 0;
  bool foundFirstOne = false;
  String data = "";
  for (int i=0; i<sizeof reply/sizeof reply[0]; i++) {
    //Serial.print(reply[i], BIN);
    String binaryString = "";
    for(int j = 0, mask = 128; j < 8; j++, mask = mask >> 1){
        bool bitValue;
        if ((reply[i] & mask) && !foundFirstOne){
            foundFirstOne = true;
            bitValue = true;
        }else if (reply[i] & mask){
            binaryString += "1";
            bitValue = true;
        }else if (foundFirstOne){
            binaryString += "0";
            bitValue = false;
        }
        if (bitValue){
          binaryString += oneT;
          if (startFlagIsDetected){
            data += oneT;
          }
          if (startFlag[startValuesFound] == oneT && !startFlagIsDetected) {
            startValuesFound += 1;
          }else{
            startValuesFound = 0;
          }
          if (endFlag[endValuesFound] == oneT && startFlagIsDetected) {
            endValuesFound += 1;
          }else{
            endValuesFound = 0;
          }
        }else {
          if (startFlagIsDetected){
            data += zeroT;
          }
          binaryString += zeroT;
          if (startFlag[startValuesFound] == zeroT && !startFlagIsDetected) {
            startValuesFound += 1;
          }else{
            startValuesFound = 0;
          }
          if (endFlag[endValuesFound] == zeroT && startFlagIsDetected) {
            endValuesFound += 1;
          }else{
            endValuesFound = 0;
          }
        }
        if (startValuesFound ==  6){
          startFlagIsDetected = true;
          //Serial.print("Flag");
          startValuesFound = 0;
        }
        if (endValuesFound == 8){
          sendDataToController(data);
          //Serial.println("End");
          endValuesFound = 0;
          startFlagIsDetected = false;
        }
    }
    
    if (!foundFirstOne) {
      binaryString = "0";
    }
    replyStr += binaryString;
  }
  //Serial.println(replyStr);
  
}

void sendDataToController(String data) {
  String configurationData = data.substring(6,16);
  //Serial.print(configurationData);
  String names[10] = {" 1p: "," 1v: "," 1a: "," 2p: "," 2v: "," 2a: "," 3p: "," 3v: "," 3a: "," 4p: "};
  int numbersSaved = 0;
  for (int i = 0; i<10; i++){
    if(configurationData[i] == oneT){
      //Serial.print(names[i]);
      String s = data.substring(16+16*numbersSaved,32+16*numbersSaved);
      //String s = "1111111111111111"; 
      int value = 0;
      //Serial.print("l");
      //Serial.print(s.length());
      for(int j = 0; j < s.length(); j++){
        //Serial.print("v");
        //Serial.print(value);
        //Serial.print("INTM");
        //Serial.print((int(s[j])-48));
        //Serial.print("pou");
        //Serial.print(pow(2,s.length()-1-j));
        value += pow(2,s.length()-1-j)*(int(s[j])-48);
      }
      motionProfiles[i] = (value-32767) * 0.01;
      Serial.print(names[i]);
      //Serial.print(s);
      //Serial.print(" ");
      //Serial.print(value);
      //Serial.print(" ");
      Serial.print(motionProfiles[i]);
      numbersSaved += 1;
    }
  }
  Serial.println();
}
