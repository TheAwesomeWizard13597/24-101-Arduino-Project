float distance(){
  float reading = analogRead(A0);
  float voltage = reading * (5/1023);
  float distance = ((137500)/(voltage - 1125));
  return(distance); 
}

//Testing Github with Arduino

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
