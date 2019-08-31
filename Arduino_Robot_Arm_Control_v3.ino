#include <SoftwareSerial.h>
#include <Servo.h>
Servo servo01; // servo01 not in use
Servo servo02;
Servo servo03;
Servo servo04;
Servo servo05;
Servo servo06;
Servo servo07;
Servo servo08;
SoftwareSerial Bluetooth(3, 4); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)

#define led 14
int servo1Pos, servo2Pos, servo3Pos, servo4Pos, servo5Pos, servo6Pos, servo7Pos, servo8Pos; // current position
int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos, servo7PPos, servo8PPos; // previous position
int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50], servo07SP[50], servo08SP[50]; // for storing positions/steps
int speedDelay = 20;
int index = 0;
int dataIn;
int m = 0;
int i;

void setup() {
  pinMode(led, OUTPUT);
  servo01.attach(5); // servo01 not in use
  servo02.attach(6);
  servo03.attach(7);
  servo04.attach(8);
  servo05.attach(9);
  servo06.attach(10);
  servo07.attach(11);
  servo08.attach(12);
  Bluetooth.begin(9600); // Default baud rate of the Bluetooth module
  Bluetooth.setTimeout(5);
  delay(20);
  //Serial.begin(38400); //grey out when uploading to board
  // Move robot arm to initial position
  servo1PPos = 90;
  servo01.write(servo1PPos); // servo01 not in use
  servo2PPos = 100;
  servo02.write(servo2PPos);
  servo3PPos = 120;
  servo03.write(servo3PPos);
  servo4PPos = 95;
  servo04.write(servo4PPos);
  servo5PPos = 60;
  servo05.write(servo5PPos);
  servo6PPos = 110;
  servo06.write(servo6PPos);
  servo7PPos = 60;
  servo07.write(servo7PPos);
  servo8PPos = 60;
  servo08.write(servo8PPos);
}

void loop() {
  // Check for incoming data
  if (Bluetooth.available() > 0) {
    dataIn = Bluetooth.read();  // Read the data
    if (dataIn == 0) {
      m = 0;
    }
    if (dataIn == 1) {
      m = 1;
    }
    if (dataIn == 2) {
      m = 2;
    }
    if (dataIn == 3) {
      m = 3;
    }
    if (dataIn == 4) {
      m = 4;
    }
    if (dataIn == 5) {
      m = 5;
    }
    if (dataIn == 6) {
      m = 6;
    }
    if (dataIn == 7) {
      m = 7;
    }
    if (dataIn == 8) {
      m = 8;
    }
    if (dataIn == 9) {
      m = 9;
    }
    if (dataIn == 10) {
      m = 10;
    }
    if (dataIn == 11) {
      m = 11;
    }
    if (dataIn == 12) {
      m = 12;
    }
    if (dataIn == 14) {
      m = 14;
    }
    if (dataIn == 16) {
      m = 16;
    }
    if (dataIn == 17) {
      m = 17;
    }
    if (dataIn == 18) {
      m = 18;
    }
    if (dataIn == 19) {
      m = 19;
    }
    if (dataIn == 20) {
      m = 20;
    }
    if (dataIn == 21) {
      m = 21;
    }
    if (dataIn == 22) {
      m = 22;
    }
    if (dataIn == 23) {
      m = 23;
    }
    if (dataIn == 24) {
      m = 24;
    }
    if (dataIn == 25) {
      m = 25;
    }
    if (dataIn == 26) {
      m = 26;
    }
    if (dataIn == 27) {
      m = 27;
    }
    if (dataIn == 28) {
      m = 28;
    }
    if (dataIn == 29) {
      m = 29;
    }
    if (dataIn == 30) {
      m = 30;
    }
    if (dataIn == 31) {
      m = 31;
    }

    // Move robot arm
    /*// Move servo 1 in positive direction. NOTE: servo01 not in use
    while (m == 16) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo01.write(servo1PPos);
      servo1PPos++;
      delay(speedDelay);
    }
    // Move servo 1 in negative direction
    while (m == 17) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo01.write(servo1PPos);
      servo1PPos--;
      delay(speedDelay);
    }*/
    // Move servo 2 +ve
    while (m == 19) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo02.write(servo2PPos);
      servo2PPos++;
      delay(speedDelay);
    } // Move servo 2 -ve
    while (m == 18) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo02.write(servo2PPos);
      servo2PPos--;
      delay(speedDelay);
    }
    // Move servo 3 +ve
    while (m == 20) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo03.write(servo3PPos);
      servo3PPos++;
      delay(speedDelay);
    } // Move servo 3 -ve
    while (m == 21) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo03.write(servo3PPos);
      servo3PPos--;
      delay(speedDelay);
    }
    // Move servo 4 +ve
    while (m == 23) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo04.write(servo4PPos);
      servo4PPos++;
      delay(speedDelay);
    }// Move servo 4 -ve
    while (m == 22) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo04.write(servo4PPos);
      servo4PPos--;
      delay(speedDelay);
    }
    // Move servo 5 +ve
    while (m == 25) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo05.write(servo5PPos);
      servo5PPos++;
      delay(speedDelay);
    } // Move servo 5 -ve
    while (m == 24) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo05.write(servo5PPos);
      servo5PPos--;
      delay(speedDelay);
    }
    // Move servo 6 +ve
    while (m == 26) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo06.write(servo6PPos);
      servo6PPos++;
      delay(speedDelay);
    } // Move servo 6 -ve
    while (m == 27) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo06.write(servo6PPos);
      servo6PPos--;
      delay(speedDelay);
    }
    // Move servo 7 +ve
    while (m == 28) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo07.write(servo7PPos);
      servo7PPos++;
      delay(speedDelay);
    } // Move servo 7 -ve
    while (m == 29) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo07.write(servo7PPos);
      servo7PPos--;
      delay(speedDelay);
    }
    // Move servo 8 +ve
    while (m == 30) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo08.write(servo8PPos);
      servo8PPos++;
      delay(speedDelay);
    } // Move servo 8 -ve
    while (m == 31) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo08.write(servo8PPos);
      servo8PPos--;
      delay(speedDelay);
    }
    
    // If arm speed slider is changed
    if (dataIn > 101 & dataIn < 250) {
      speedDelay = dataIn / 10; // Change servo speed (delay time)
    }

      /* Servo 1 is not in use!
      if (servo01SP[i] == servo01SP[i + 1]) {
      }
      if (servo01SP[i] > servo01SP[i + 1]) {
        for ( int j = servo01SP[i]; j >= servo01SP[i + 1]; j--) {
          servo01.write(j);
          delay(speedDelay);
        }
      }
      if (servo01SP[i] < servo01SP[i + 1]) {
        for ( int j = servo01SP[i]; j <= servo01SP[i + 1]; j++) {
          servo01.write(j);
          delay(speedDelay);
        }
      }*/
      // Servo 2
      if (servo02SP[i] == servo02SP[i + 1]) {
      }
      if (servo02SP[i] > servo02SP[i + 1]) {
        for ( int j = servo02SP[i]; j >= servo02SP[i + 1]; j--) {
          servo02.write(j);
          delay(speedDelay);
        }
      }
      if (servo02SP[i] < servo02SP[i + 1]) {
        for ( int j = servo02SP[i]; j <= servo02SP[i + 1]; j++) {
          servo02.write(j);
          delay(speedDelay);
        }
      }
      // Servo 3
      if (servo03SP[i] == servo03SP[i + 1]) {
      }
      if (servo03SP[i] > servo03SP[i + 1]) {
        for ( int j = servo03SP[i]; j >= servo03SP[i + 1]; j--) {
          servo03.write(j);
          delay(speedDelay);
        }
      }
      if (servo03SP[i] < servo03SP[i + 1]) {
        for ( int j = servo03SP[i]; j <= servo03SP[i + 1]; j++) {
          servo03.write(j);
          delay(speedDelay);
        }
      }
      // Servo 4
      if (servo04SP[i] == servo04SP[i + 1]) {
      }
      if (servo04SP[i] > servo04SP[i + 1]) {
        for ( int j = servo04SP[i]; j >= servo04SP[i + 1]; j--) {
          servo04.write(j);
          delay(speedDelay);
        }
      }
      if (servo04SP[i] < servo04SP[i + 1]) {
        for ( int j = servo04SP[i]; j <= servo04SP[i + 1]; j++) {
          servo04.write(j);
          delay(speedDelay);
        }
      }
      // Servo 5
      if (servo05SP[i] == servo05SP[i + 1]) {
      }
      if (servo05SP[i] > servo05SP[i + 1]) {
        for ( int j = servo05SP[i]; j >= servo05SP[i + 1]; j--) {
          servo05.write(j);
          delay(speedDelay);
        }
      }
      if (servo05SP[i] < servo05SP[i + 1]) {
        for ( int j = servo05SP[i]; j <= servo05SP[i + 1]; j++) {
          servo05.write(j);
          delay(speedDelay);
        }
      }
      // Servo 6
      if (servo06SP[i] == servo06SP[i + 1]) {
      }
      if (servo06SP[i] > servo06SP[i + 1]) {
        for ( int j = servo06SP[i]; j >= servo06SP[i + 1]; j--) {
          servo06.write(j);
          delay(speedDelay);
        }
      }
      if (servo06SP[i] < servo06SP[i + 1]) {
        for ( int j = servo06SP[i]; j <= servo06SP[i + 1]; j++) {
          servo06.write(j);
          delay(speedDelay);
        }
      }
      // Servo 7
      if (servo07SP[i] == servo07SP[i + 1]) {
      }
      if (servo07SP[i] > servo07SP[i + 1]) {
        for ( int j = servo07SP[i]; j >= servo07SP[i + 1]; j--) {
          servo07.write(j);
          delay(speedDelay);
        }
      }
      if (servo07SP[i] < servo07SP[i + 1]) {
        for ( int j = servo07SP[i]; j <= servo07SP[i + 1]; j++) {
          servo07.write(j);
          delay(speedDelay);
        }
      }
      // Servo 8
      if (servo08SP[i] == servo08SP[i + 1]) {
      }
      if (servo08SP[i] > servo08SP[i + 1]) {
        for ( int j = servo08SP[i]; j >= servo08SP[i + 1]; j--) {
          servo08.write(j);
          delay(speedDelay);
        }
      }
      if (servo08SP[i] < servo08SP[i + 1]) {
        for ( int j = servo08SP[i]; j <= servo08SP[i + 1]; j++) {
          servo08.write(j);
          delay(speedDelay);
        }
      }
    }
}
