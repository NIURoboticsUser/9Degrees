char ch;


void setup(){
  
  Serial.begin(9600);
}

void loop(){
  while(Serial.available()>0){
    ch = Serial.read();   //must typecast input as char
    Serial.print(ch);
  }
}

