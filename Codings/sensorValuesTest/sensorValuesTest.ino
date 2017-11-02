int read;

void setup() {
  for(int i=0;i<12;i+=2){
    pinMode(22+i,INPUT);
  }
  Serial.begin(9600);
  delay(100);
  Serial.println("OK");
}

void loop() {
  for(int i=22;i<=32;i+=2){
    read=digitalRead(i);
    Serial.print(read);
  }
  Serial.println("");
  delay(50);
}
