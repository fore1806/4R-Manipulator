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
  char reply[50];
  int i = 0;
  while(Serial2.available()){
    reply[i] = Serial2.read();
    i += 1;
  }
  reply[i] = '\0';
  if(strlen(reply) > 0){
    Serial.print(reply);
    Serial.println("We have just rad some data");
  }
}
