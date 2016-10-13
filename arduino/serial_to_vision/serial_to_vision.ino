/*
 * Code for all vision-related movement.
 * 
 * Must be written to as follows (in remote code, not here):
 *    Serial.write(command);
 *    Serial.write(value);
 *    
 * Numerical commands allow for addressing of individual motors.
 *    1 is for the kinect servo. Commands are from -90 to 90, corresponding to degrees
 *    2 is for the camera pan servo. Commands are from -180 to 180
 *    3 is for the camera tilt servo. Commands are from -90 to 90
 *    
 */

#include <math.h>
#include <Servo.h>

char buf[255];
Servo kinectServo;
Servo cameraPanServo;
Servo cameraTiltServo;
Servo cameraLiftServo;
Servo excavationFlapperServo;
byte high_byte, low_byte;
bool flapperDown = false;

#define FLAPPERSIGNALPIN 12
#define FLAPPERSERVO 11
#define KINECTMULT 10
#define CAMERAPANMULT 2.79
#define CAMERATILTMULT 10
#define CAMERABASE 600
#define CONTINUOUSSTOP 93
#define PANMID 55
#define PANFULL 110

int kinectTheta = 45;
int cameraPanTheta = 55; // Multiply by 2.5ish for true output
int cameraTiltTheta = 90;
int cameraLift = CONTINUOUSSTOP;

void setup() {
  Serial.begin(115200);
  
  // Servo pins for camera system (need to be reassigned upon reconnection)
  kinectServo.attach(4);
  cameraPanServo.attach(3);
  cameraTiltServo.attach(6);
  cameraLiftServo.attach(5);
  cameraLiftServo.write(CONTINUOUSSTOP);

  // Servo pin for excavation servo
  excavationFlapperServo.attach(FLAPPERSERVO);
  pinMode(13,OUTPUT);
  pinMode(FLAPPERSIGNALPIN, INPUT);
}
  
void loop() {
  if(digitalRead(FLAPPERSIGNALPIN)) {
    flapperDown = true;
  } else {
    flapperDown = false;
  }

  if (flapperDown) {
    excavationFlapperServo.write(0); // Down
  } else {
    excavationFlapperServo.write(130); // Up
  }
  
  if(Serial.available() > 3) {
    char stat [20];
    
    while (Serial.available() > 3) {
      Serial.read();
    }

    high_byte = Serial.read();    // Read potential high byte

    // Check if byte is high byte
    if (!((high_byte & 0x80) == 0x80)) { 
      high_byte = Serial.read(); // Read next byte if not high byte
    }

    // Check if byte is high byte
    if ((high_byte & 0x80) == 0x80) { 
      low_byte = Serial.read();
      
      // Check if byte is low byte
      if (!((low_byte & 0x0080) == 0x0080)) { 
        
        //sprintf(stat, "high: %#X\t low:%#X@", high_byte, low_byte);
        //Serial.write(stat);

        //Extracts Multipliers from bytes
        byte Kmult = (high_byte & 0x60) >> 5;
        byte Tmult = high_byte & 0x1F;
        byte Pmult = (low_byte & 0x7E) >> 1;
        byte Lmult = low_byte & 0x01;

        // Checks for invalid kinect value
        if (Kmult > 2){
          Serial.write("Invalid Kinect Angle@");
          digitalWrite(13, LOW);
        } else {
          
          // Set Kinect angle
          kinectTheta = 45.0*(Kmult+1);

          // Set Camera Tilt Angle
          cameraTiltTheta = 180.0 / 32.0 * Tmult;

          // Set Cammera Pan Angle
          if (Pmult == 0) {
            // Allows for camera to go to full angle and 0 angle depending on what the previous angle was
            if (cameraPanTheta < PANMID) {
              cameraPanTheta = 0;
            } else {
              cameraPanTheta = PANFULL;
            }
          } else {
            cameraPanTheta = PANFULL / 64.0 * Pmult;
          }
          
          // Sets if camera is lifting
          cameraLift = (180.0 - CONTINUOUSSTOP) * Lmult + CONTINUOUSSTOP;

          // Write out recieved angle back to Teensy for publishing
          char ret_msg[3];
          ret_msg[0] = Pmult;
          ret_msg[1] = Tmult;
          ret_msg[2] = '@';
          Serial.write(ret_msg);
          digitalWrite(13, HIGH);
        }
      } else {
        // Error if low byte cannot be found
        sprintf(stat, "No low: %#X@", low_byte); 
        Serial.write("No low\n");
        digitalWrite(13, LOW);
      }
    } else {
      //Error if high byte cannot be found
      sprintf(stat, "No high: %#X@", high_byte); 
      Serial.write(stat);
      digitalWrite(13, LOW);
    }
  } else {
    digitalWrite(13, LOW);
  }

  // Write to all servos
  kinectServo.write(kinectTheta);
  cameraPanServo.write(cameraPanTheta);
  cameraTiltServo.write(cameraTiltTheta);
  cameraLiftServo.write(cameraLift);
  
}
