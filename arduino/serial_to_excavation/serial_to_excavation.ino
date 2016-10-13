//
//   Code for all excavation-related controls.
//
//   Must be written with a high byte that contains autonomous commands
//   and a low byte that contains manual commands.
//
//

#include "BMSerial.h"
#include "RoboClaw.h"
#include "Servo.h"
#include <math.h>

#define FLAPPERSIGNALPIN 12
#define led_pin 13

// Roboclaw Addresses
#define exc_robo 0x80 //Excavation Roboclaw

// Current limit, in units of 10mA (current_limit of 100 --> 1A actual limit)
#define current_limit 2000 // 2000mA --> 20A.

#define DOWN 0
#define UP 1
#define OFF 0
#define ON 1

// Define variables
unsigned long msgtime;
bool disc = true;
bool manual = true;
bool routine_finished = false;


int bucketLadderCurrent; // Roboclaw sends current values as a 16 bit character
int dumpingBucketCurrent;
int ladder_potr = 0; // Right actuator potentiometer variable
int ladder_potl = 0; // Left actuator potentiometer variable
int high_pot_limit = 960; // Upper potentiometer limit for the actuators (bottom)
int low_pot_limit = 320; // Lower potentiometer limit for the actuators (top)
int dumping_bucket_high = 840; // Upper Potentiometer limit for the bucket (top)
int dumping_bucket_low = 135; // Lower Potentiometer limit for the bucket (bottom)
int flapper_up = 130; 
int flapper_down = 0;
int flapper_pos = flapper_up;
byte auto_msg, cmd_msg;

int load_threshold = 20;
int ladder_current_fault = 4000;
int ladder_current_ideal = 1500;
int current_ladder_position = 0;
int digstate = 0;
int bkloc = 0;
const int CYCLE_PERIOD = 500;
const int DUTY_CYCLE = 50;

int16_t current1;
int16_t current2;

bool ledState = 0;

// Right Actuator
int actr_fwd = 5; // White cable
int actr_rev = 4; // Red cable

// Left Actuator
int actl_fwd = 7; // White cable
int actl_rev = 6; // Red cable

// Conveyor Belt
int conv_fwd = 8; // White cable
int conv_rev = 9; // Red cable

unsigned long current_time;
unsigned long wait_time = 20000;

//Setup communcations with roboclaw. Use pins 10 and 11 with 10ms timeout. Pin 10 goes to S2 on roboclaw; Pin 11 to S1
RoboClaw roboclaw(10, 11, 10000);


bool targetFindingMode() {
  int desiredHeight = 800;
  byte cmd = 0x00;

  bool bucket_ladder_done = setBucketLadder(desiredHeight);
  bool dumping_bucket_done = setDumpBucket(dumping_bucket_low + 10);

  return (bucket_ladder_done && dumping_bucket_done);
}

bool drivingMode() {
  bool bucket_ladder_done = setBucketLadder(low_pot_limit);
  bool dumping_bucket_done = setDumpBucket(dumping_bucket_low + 10);
  return (bucket_ladder_done && dumping_bucket_done);
}

// Autonomous digging routine. Returns 1 for success, 0 for error
bool digRoutine() {
  byte cmd;
  bool return_val = false;
  bool dumping_bucket_done = false;

  switch (digstate) {
    case 0:
      //Initialize
      dumping_bucket_done = setDumpBucket(dumping_bucket_low);
      if (dumping_bucket_done) {
        digstate = 1;
      }
      break;

    case 1:
      //Run and drop ladder
      // Start bucket ladder and conveyor
      roboclaw.BackwardM1(exc_robo, 127);
      digitalWrite(conv_fwd, HIGH);
      current_ladder_position = moveBucketLadder(ON, DOWN, 100);

      if (current_ladder_position < (high_pot_limit - 2)) {
        // Check to verify that ladder current draw does not exceed thresholds
        if (bucketLadderCurrent > ladder_current_fault) {
          // If fault threshold exceeded, raise ladder
          //break;
          digstate = 3;
          current_ladder_position = moveBucketLadder(ON, UP, 100);
        } else if (bucketLadderCurrent > ladder_current_ideal) {
          // Ladder is ideal depth for digging, allow it to dig at this level
          current_ladder_position = moveBucketLadder(ON, DOWN, DUTY_CYCLE);
        } else {
          // Move ladder down until current draw hits current threshold indicating ideal digging conditions
          current_ladder_position = moveBucketLadder(ON, DOWN, 100);
        }
      } else { // Bucket ladder has reached lowest acceptable position, advance state
        digstate = 2;
        current_time = millis();
      }
      break;

    case 2:
      // Pause at bottom
      if (millis() - current_time > wait_time) { // Evaluates to true when wait_time has elapsed
        digstate = 3;
      }
      break;

    case 3:
      // Return up
      current_ladder_position = setBucketLadder(low_pot_limit);

      if (current_ladder_position < low_pot_limit + 2) {
        digstate = 4;
      }

      break;

    case 4:
      // Stop conveyor
      roboclaw.ForwardM1(exc_robo, 0);
      digitalWrite(conv_fwd, LOW);
      digstate = 5;
      break;

    case 5:
      return_val = true;
      break;

  }

  return return_val;

} // End autonomous excavation routine

// Put bucket ladder at shallow digging height and runs it + conveyor (bucket at low point)
bool shallowDig() {
  // Needs Implementation, could be useful
  bool return_val = true;
  return return_val;
}


// Brings bucketto highest potition for dumping
bool dumpBucket() {
  bool return_val = false;
  int bucket_position = setDumpBucket(dumping_bucket_high);
  if (bucket_position >= dumping_bucket_high) {
    return_val = true;
  }
  return return_val;
}




void setup() {
  // Set up serial communication w/ computer
  Serial.begin(115200);
  pinMode(13, OUTPUT);


  //  scale.set_scale();
  //  scale.tare();  //Reset the scale to 0
  //  scale.set_scale(calibration_factor);

  // Initailize pins
  //pinMode(A0, INPUT);
  //pinMode(A1, INPUT);
  pinMode(actr_fwd, OUTPUT);
  pinMode(actr_rev, OUTPUT);
  pinMode(actl_fwd, OUTPUT);
  pinMode(actl_rev, OUTPUT);
  pinMode(conv_fwd, OUTPUT);
  pinMode(conv_rev, OUTPUT);

  // Write low values to all pins for startup
  digitalWrite(actr_fwd, LOW);
  digitalWrite(actr_rev, LOW);
  digitalWrite(actl_fwd, LOW);
  digitalWrite(actl_rev, LOW);
  digitalWrite(conv_fwd, LOW);
  digitalWrite(conv_rev, LOW);

  // Init roboclaw
  roboclaw.begin(9600);
  roboclaw.SetM2EncoderMode(exc_robo, 0x01);
  roboclaw.SetM2PositionPID(exc_robo, 2000, 0, 0, 0, 10, dumping_bucket_low, dumping_bucket_high);
  //Set up motor 2 as position based pid
  //Set high
  //Set low

  //  flapper.attach(flapper_pin);
  flapper_pos = flapper_up;
  digitalWrite(FLAPPERSIGNALPIN, LOW);

  // Set hard current limits
  roboclaw.SetM1MaxCurrent(exc_robo, current_limit); // Bucket Ladder Motor
  roboclaw.SetM2MaxCurrent(exc_robo, current_limit); // Bucket Dump Linear Actuator

  // Initlaize current varibles
  bucketLadderCurrent = 0; // Roboclaw sends current values as a 16 bit character
  dumpingBucketCurrent = 0;
}

void loop() {

  //roboclaw.ReadCurrents(exc_robo, bucketLadderCurrent, dumpingBucketCurrent);
  //Serial.println(bucketLadderCurrent);

  //Read and publish currents
  //int16_t current1, current2;
  //roboclaw.ReadCurrents(exc_robo, current1, current2);
  //int pot_bucket = roboclaw.ReadEncM2(exc_robo);
  //char outmsg[50];
  //sprintf(outmsg, "%d,%d,%d,%d,%d\n", current1*10, current2*10, ladder_potr, ladder_potl, pot_bucket);
  //Serial.write(outmsg);

  // Read diagnostic information from the robot
  //ladder_potl = analogRead(0); // Left actuator potentiometer position
  //ladder_potr = analogRead(1); // Right actuator potentiometer position

  //Serial.println(scale.get_units(), 1);

  //Serial.println(ladder_potr);

  // Read the serial input from ROS
  if (Serial.available() > 3) {
    while (Serial.available() > 3) {
      Serial.read();
    }
    disc = false;
    msgtime = millis();
    byte temp1, temp2;

    temp1 = Serial.read();    // Possible commands: Bitcodes in this order

    if (!((temp1 & 0x80) == 0x80)) {
      temp1 = Serial.read();
    }

    if ((temp1 & 0x80) == 0x80) {
      temp2 = Serial.read();
      if (!((temp2 & 0x0080) == 0x0080)) {
        auto_msg = temp1;
        cmd_msg = temp2;

        //char stat [20];
        //sprintf(stat, "Auto: %#X\t Man:%#X\n", auto_msg, cmd_msg);
        //Serial.write(stat);
        if ((auto_msg & 0x0040) == 0x0040) {
          manual = true;
        } else {
          manual = false;
          if ((auto_msg & 0x0F) != 3) {
            digstate = 0;
          }
          switch (auto_msg & 0x7F) {
            case 0:
              manual = true;
              // Manual control;
              break;
            case 1:
              // Lower bucketladder for target finding
              routine_finished = targetFindingMode();
              break;
            case 2:
              // Raise bucket ladder for driving
              routine_finished = drivingMode();
              break;
            case 3:
              // Dig
              routine_finished = digRoutine();
              break;
            case 4:
              // Drive-n-dig
              routine_finished = shallowDig();
              break;
            case 5:
              // Dump
              routine_finished = dumpBucket();
              break;
            default:
              Serial.write("Unknown State\n");
              break;
          }

        }
        //Read and publish currents
        //int16_t current1, current2;
        roboclaw.ReadCurrents(exc_robo, current1, current2);
        int pot_bucket = roboclaw.ReadEncM2(exc_robo);
        char outmsg[60];
        sprintf(outmsg, "%d,%d,%d,%d,%d\n", current1*10, current2*10, ladder_potr, ladder_potl, pot_bucket);
        Serial.write(outmsg);



      } else {
        Serial.write("No Man\n");
      }
    } else {
      Serial.write("No Auto\n");
    }
    // 0x Null,conpwr, blpwr, bldir, bkpwr, bkdir, actpwr, actdir

    if (cmd_msg == 0x00) {
      digitalWrite(13, LOW);
    } else {
      digitalWrite(13, HIGH);
    }


  } else {
    if ((millis() - msgtime) > 250) {
      disc = true;
    }
  }

  if (disc) {
    manual = true;
    cmd_msg = 0x00;
  }
  
  //Always run
  if (manual) {
    manualControl(cmd_msg);
  } else {
    //leftFollower(false, false);
  }

  if (flapper_pos == flapper_down) {
    digitalWrite(FLAPPERSIGNALPIN, HIGH);
  } else {
    digitalWrite(FLAPPERSIGNALPIN, LOW);
  }

}


// Bucket Ladder Raise/Lower code controls the leader actuator (make sure leader is slower actuator)
int moveBucketLadder (bool actpwr, bool actdir, int duty) {
  // Read diagnostic information from the robot
  ladder_potl = analogRead(0); // Left actuator potentiometer position
  ladder_potr = analogRead(1); // Right actuator potentiometer position

  bool revr_lim = (ladder_potr < high_pot_limit);
  bool fwdr_lim = (ladder_potr > low_pot_limit);


  // Write to right leftFollower(false, false)

  // PWM shutoff actuators
  if (!actdir && (millis() % CYCLE_PERIOD > duty * CYCLE_PERIOD / 100)) {
    actpwr = false;
  }

  if (abs(ladder_potr - ladder_potl) < 20) {
    digitalWrite(actr_fwd, actpwr && !actdir && fwdr_lim);
    digitalWrite(actr_rev, actpwr && actdir && revr_lim);
  } else {
    digitalWrite(actr_rev, OFF);
    digitalWrite(actr_fwd, OFF);
  }
  leftFollower(actpwr, actdir);
  return (ladder_potr);
}

// Dumpbucket Raise/Lower Code
int moveDumpBucket (bool power, bool directn) {
  // Run bucket
  bkloc = roboclaw.ReadEncM2(exc_robo);
  if (power) {
    if (directn && (bkloc > dumping_bucket_low)) {
      //Down
      roboclaw.ForwardM2(exc_robo,  127);
    } else if (!directn && (bkloc < dumping_bucket_high)) {
      //Up
      roboclaw.BackwardM2(exc_robo, 127);
    } else {
      roboclaw.ForwardM2(exc_robo, 0);
    }
  } else {
    roboclaw.ForwardM2(exc_robo, 0);
  }

  if (bkloc < (dumping_bucket_low)) {
    flapper_pos = flapper_down;
  } else {
    flapper_pos = flapper_up;
  }

  return bkloc;
}

// Bucket Ladder Raise/Lower code
bool setBucketLadder (int posn) {
  // Read diagnostic information from the robot
  bool return_val = false;
  ladder_potr = analogRead(1); // Right actuator potentiometer position

  if ((ladder_potr > posn + 2) && (ladder_potr < high_pot_limit)) {
    moveBucketLadder(ON, DOWN, 100);
  } else if ((ladder_potr < posn - 2) && (ladder_potr > low_pot_limit)) {
    moveBucketLadder(ON, UP, 100);
  } else {
    moveBucketLadder(OFF, DOWN, 100);
    return_val = true;
  }

  return return_val;
}

bool setDumpBucket (int posn) {
  // Sets position of dump bucket with precision of 5
  bool return_val = false;

  bkloc = roboclaw.ReadEncM2(exc_robo);

  if ((bkloc > posn + 5) && (bkloc > dumping_bucket_low)) {
    moveDumpBucket(ON, DOWN);
  } else if ((bkloc < posn - 5) && (bkloc < dumping_bucket_high)) {
    moveDumpBucket(ON, UP);
  } else {
    moveDumpBucket(OFF, DOWN);
    return_val = true;
  }

  // Returns true once done
  if (abs(bkloc - posn) <= 5) {
    return_val = true;
  }

  return return_val;
}

// Makes fast actuator follow slow one
int leftFollower(bool actpwr, bool actdir) {
  bool fwdl_lim = (ladder_potl > low_pot_limit);
  bool revl_lim = (ladder_potl < high_pot_limit);
  ladder_potl = analogRead(0); // Left actuator potentiometer position
  ladder_potr = analogRead(1); // Right actuator potentiometer position

  int tol;
  if (actpwr) {
    tol = 10;
  } else {
    tol = 2;
  }

  if (ladder_potl > ladder_potr + tol) {
    // Extend the left actuator if it is below the right one
    digitalWrite(actl_rev, OFF);
    digitalWrite(actl_fwd, ON);
  } else if (ladder_potl < ladder_potr - tol) {
    // Retract the left actuator if it is above the right one
    digitalWrite(actl_rev, ON);
    digitalWrite(actl_fwd, OFF);
  } else {
    // Match the left actuator to the right one
    digitalWrite(actl_rev, actpwr && !actdir && revl_lim);
    digitalWrite(actl_fwd, actpwr && actdir && fwdl_lim);
  }

  return ladder_potl;
}

int manualControl(byte cmd) {

  // Break command byte into individual commands
  // Bucket Ladder Actuators
  bool actpwr = ((cmd & (0x02)) == 0x02);
  bool actdir = ((cmd & (0x01)) == 0x01);
  bool actspwr =((cmd & (0x40)) == 0x40);

  // Bucket Actuator
  bool bkpwr =  ((cmd & (0x08)) == 0x08);
  bool bkdir =  ((cmd & (0x04)) == 0x04);

  // Bucket Ladder Movement
  bool blpwr =  ((cmd & (0x20)) == 0x20);
  bool bldir =  ((cmd & (0x10)) == 0x10);

  

  // Moves bucket ladder slowly, if needed  
  if(actspwr)
    moveBucketLadder(actpwr, actdir, 100);
  else
    moveBucketLadder(actpwr, actdir, DUTY_CYCLE);

  // Moves bucket according to commands
  moveDumpBucket(bkpwr, bkdir);

  // Run bucket ladder according to commands
  if (blpwr) {
    if (bldir) {
      roboclaw.ForwardM1(exc_robo, 127);
    } else {
      roboclaw.BackwardM1(exc_robo, 127);
    }
  } else {
    roboclaw.ForwardM1(exc_robo, 0);
  }
  // Run conveyor (currently tied to bucket ladder)
  digitalWrite(conv_rev, blpwr && bldir); 
  digitalWrite(conv_fwd, blpwr && !bldir);

}

