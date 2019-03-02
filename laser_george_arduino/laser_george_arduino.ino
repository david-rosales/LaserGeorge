#include <Servo.h> 

Servo theta_servo; 
Servo phi_servo;
void setup() {
  // define attachment pin for servos 
  theta_servo.attach(9); 
  phi_servo.attach(8); 

  // Define serial 
  pinMode(1, OUTPUT);
  Serial.begin(19200); 
  Serial.println("Ready");
}

/* 
 *  Idea here is that there will be a stream of values 
 *  controls for theta_servo will send, say, 90t for 90
 *  degrees and for the phi_servo will send 60p for 60
 *  degrees. Hopefully there won't be issues with mutex. 
 *  90t60p89t61p will have the servos do 90 degrees theta 
 *  60 degrees phi and 89 degrees theta and 61 degrees phi
 */
 
void loop() {
  static int v = 0; 
  
  if (Serial.available()) {
    char ch = Serial.read();

    switch(ch) {
      case '0'...'9':
        v = v*10 + ch - '0';
        break;
      case 't':
        theta_servo.write(v);
        v = 0; 
        break; 
       case 'p':
        phi_servo.write(v);
        v = 0;
        break;
    }
  }

}
