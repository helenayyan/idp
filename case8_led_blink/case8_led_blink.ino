int led_pin = 5;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(led_pin, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {

  digitalWrite(led_pin,LOW);
  delay(3000);
  bool find_victim = true;
  if (find_victim == true){
  digitalWrite(led_pin,HIGH);
  delay(2000);
}
//  digitalWrite(led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(500);                       // wait for a second
//  digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
//  delay(500);                       // wait for a second
}
