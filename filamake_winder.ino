// Filament Winder based on first design of Ian Johnson

#include <EEPROM.h>
#include <AccelStepper.h>				// include the library for accelstepper

// Define a stepper and the pins it will use
const int stepP = 10;										// Pin for step (default: D10, Expansion port, 3rd pin on Filawinder PCB), connect to Pololu e.g. DRV8825
const int dirP = 6;									        // Pin for dir (default: D06, Servo S on Filawinder PCB), connect to Pololu e.g. DRV8825
AccelStepper stepper(AccelStepper::DRIVER, stepP, dirP); 	// constructor for stepper, connection type = via driver

//Filament Sensor
#include "QTRSensors.h"
#define NUM_SENSORS   4					// number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4		// average 4 analog samples per sensor reading
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     

// sensors 0 through 3 are connected to analog pins pins 3 through 0 , respectively
QTRSensorsAnalog qtra((unsigned char[]) {
  3, 2, 1, 0}
, 
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


// Spooler PID Setup
#include "PID_v1.h"
double Setpoint, Input, Output;									//Define PID Variables
PID pullPID(&Input, &Output, &Setpoint, .1, 0, 0, REVERSE);		//Specify the links and initial tuning parameters

// Digital Pins
const int hall_a_Pin = 7;			// Hall sensor A
const int sensor_setPin = 4;		// Guide Min Calibration Button
const int motor_spoolerPin = 5;		// PWM pin for spooler motor
// int guidePin = 6;				// Guide servo  ---------  functions for servo disabled
const int guide_EleftPin = 3;		// Endstop left
const int guide_ErightPin = 8;		// Endstop right
const int toggle = 2;				// Auto / Manual toggle switch

// Analog Pins
const int knob_Pin = 5;				// Pot that controls the fast puller speed to raise the loop
const int sensor_1 = 0;				// 4 filament sensor
const int sensor_2 = 1;				// 3 filament sensor
const int sensor_3 = 2;				// 2 filament sensor
const int sensor_4 = 3;				// 1 filament sensor
int cal_led = A4;              //Auto / Manual 2 states switch


int hall_a_status = HIGH;			// The last reading from the sensor
int hall_b_status = HIGH;			// The last reading from the other sensor
int hall_a_mode = 0;				// Has sensor been triggered?
int Revolutions = -1;				// Rotation counter
 float Last_Revolution = 0;			// For checking if a new rotation has been counted

const int steigung = 150;			// Thread pitch in hundreds of a millimetre, M8 usually 125, M10 usually 150
const int fullsteps = 200;			// Fullsteps for one revolution (360°) of the stepper motor
const int maximumspeed = 100;		// Maximum speed for the stepper motor, steps per second
const int accel = 400;				// Acceleration of stepper motor, steps per second per second 
const int drahtbreite1 = 300;		// Filament width (hundredths of a millimetre) with jumper on pin 12 set
const int drahtbreite2 = 175;		// Filament width (hundredths of a millimetre) without jumper on pin 12
const int lockn = 5;				// number of guide movements before a direction change is accepted
const int startpoint = 1;			// set -1 or 1 for start at left or right side, seen from spool

int weg = 0;                		// variable for way of movement, used by program
int lock = 0;						// mechanism for change of direction, used by program
float sspeed = 0;					// for backup purpose of speed


//Variables for smoothing the potentiometer reading
const int numReadings = 5;

int readings[numReadings];      	// the readings from the analog input
int index = 0;                  	// the index of the current reading
int total = 0;                  	// the running total
int average = 0;                	// the average
int inputPin = knob_Pin;			// Pot that controls the fast puller speed to raise the loop

//Variables for puller speed control via the photo sensors

unsigned int line_position; 
int spooler_speed = 200;
int puller_speed = 255;
int rotation_status = 0;
int puller_speed_old = 0;


void setup (){
  Serial.begin (9600);
  
  pinMode(motor_spoolerPin, OUTPUT);			// PWM pin for spooler motor
  pinMode(hall_a_Pin, INPUT);					//  sensor A
  pinMode(guide_EleftPin, INPUT);				// Endstop left, active LOW
  pinMode(guide_ErightPin, INPUT);			// Endstop right, active LOW

  pinMode(sensor_setPin, INPUT);				// Guide Min Calibration Button
  pinMode(toggle, INPUT);						// Auto / Manual toggle switch
  pinMode(sensor_1, INPUT);					// Top filament sensor
  pinMode(sensor_2, INPUT);					// Middle filament sensor
  pinMode(sensor_3, INPUT);					// Bottom filament sensor
  pinMode(sensor_4, INPUT);					// filament sensor
  pinMode(13, OUTPUT);						// used as verification signal
  pinMode(9, INPUT);							// test, wenn low per Jumper dann test()
  pinMode(11, INPUT);							// troubleshoot, wenn low per Jumper wird über serial gesendet
  pinMode(12, INPUT);							// Filament width, when low (Jumper on Pin 12) 3mm else 1,75mm if You did not change the basic setting above

  digitalWrite(hall_a_Pin, HIGH);				// Pullup resistor for Hall A
  digitalWrite(guide_EleftPin, HIGH);			// Pullup resistor for guide endstop switch
  digitalWrite(guide_ErightPin, HIGH);		// Pullup resistor for guide endstop switch
  digitalWrite(sensor_setPin, HIGH);			// Pullup resistor for pid setup switch
  digitalWrite(toggle, HIGH);					// Pullup resistor for toggle
  digitalWrite(9, HIGH);						// Pullup resistor for test
  digitalWrite(10, HIGH);						// Pullup resistor for -------- unbenutzt
  digitalWrite(11, HIGH);						// Pullup resistor for troubleshoot
  digitalWrite(12, HIGH);						// Pullup resistor for Filamentdicke


  //initialize the variables
  Input = line_position; 
  Setpoint = 1500;									//The value PID tries to maintain.  The number controls the amount of tension on the spool.
  pullPID.SetMode(AUTOMATIC);							//turn the PID on
  pullPID.SetControllerDirection(REVERSE);

  // initialize all the knob readings to 0: 
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;

  // setup for stepper
  stepper.setMaxSpeed(maximumspeed);							// set max speed of stepper motor
  stepper.setAcceleration(accel);								// set acceleration of stepper motor

  int sfullsteps = fullsteps / 10;

  if (digitalRead(12) == HIGH)								// If there is no jumper on Pin 12 use drahtbreite2
    weg = drahtbreite2 * 10 / steigung * sfullsteps;

  if (digitalRead(12) == LOW)									// If there is a jumper on Pin 12 use drahtbreite1
    weg = drahtbreite1 * 10 / steigung * sfullsteps;

 								// Move away from the startpoint	
  nguide_start();												// Set guide to start position left or right
}

void loop (){
  analogWrite(cal_led, 1023);



  if (digitalRead(11) == 0)



  if (digitalRead(9) == 0)
    test();

  // Calibrate the filament sensor  
  if (digitalRead(sensor_setPin) == LOW && digitalRead(toggle) == HIGH )
    set_sensor();

  nguide_control();											// Check if anything to do with the guide

  if (digitalRead(toggle) == HIGH)
    manual_control();										// Go to the manual_control function for manual pull control
    if (digitalRead(toggle) == LOW)
    pull_control();											// Go to the spool_control function for auto control

}

void serial_output()
{
//disabled
}

void manual_control(){  

  int knob_reading = analogRead(knob_Pin);					// Get value from Puller Max Speed Knob
  puller_speed = knob_reading / 4.011;						// convert reading from pot to 0-255
  analogWrite(motor_spoolerPin,puller_speed);					// Set motor to the speed

}

void pull_control()
{
  qtra.readCalibrated(sensorValues);              
  unsigned int line_position = qtra.readLine(sensorValues, QTR_EMITTERS_OFF, 1);  
  

  Input = line_position;                         //Get line position from sensors


  if (!pullPID.Compute()) return;              //Run the PID 

  int ScaledOutput = (Output * 1.7);             //Scale the Output from 0-150 to 0-255)       
  if (ScaledOutput <= 0) {ScaledOutput = 1;}     //Limit the output to the range of 0-255) 
  if (ScaledOutput >= 255){ScaledOutput = 255;}
  puller_speed = ScaledOutput;
    
  if ( (puller_speed - puller_speed_old) > 25) puller_speed = puller_speed_old + 25;
  if ( (puller_speed - puller_speed_old) < -25) puller_speed =  puller_speed_old - 25;
  
  analogWrite(motor_spoolerPin,puller_speed);    //Set the spool speed to the PID result
  puller_speed_old = puller_speed;

}

void set_sensor(){									//Calibrate the reflectance sensors  Add LED indication that calibration is happening
  analogWrite(cal_led, 0);

  for (int i = 0; i < 200; i++){					// make the calibration take about 10 seconds

    qtra.calibrate();							// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++){
  }


  for (int i = 0; i < NUM_SENSORS; i++){

  }

  delay(1000);
}

void test (){
  while (digitalRead(9) == 0) {
    analogWrite(motor_spoolerPin, 255);

    if (digitalRead(hall_a_Pin) == 0 && 
      digitalRead(sensor_setPin) == 0 && 
      
    digitalRead(toggle) == 0 && 
      digitalRead(9) == 0 &&
      digitalRead(10) == 0 &&
      digitalRead(11) == 0 &&
      digitalRead(12) == 0 &&
      analogRead(knob_Pin) > 900 && 
      analogRead(sensor_1) > 900 && 
      analogRead(sensor_2) > 900 &&
      analogRead(sensor_3) > 900 &&
      analogRead(sensor_4) > 900 &&
      analogRead(4) > 900 &&
      analogRead(5) > 900 &&
      analogRead(7) > 900){
      digitalWrite(13, HIGH);

    }
    else {
      digitalWrite(13, LOW);
    }
  } 
}

void nguide_start(){																				// Sets guide to endpoint
  while(digitalRead(guide_EleftPin) == HIGH && digitalRead(guide_ErightPin) == HIGH){				// as long we do not touch endstop
    stepper.move(startpoint*30000);																// set relative move
    while(digitalRead(guide_EleftPin) == HIGH && digitalRead(guide_ErightPin) == HIGH && stepper.distanceToGo() != 0)   // no endstop triggered and remaining way
      stepper.run();																			// move it ...
    if(digitalRead(guide_EleftPin) == LOW || digitalRead(guide_ErightPin) == LOW){
      stepper.stop();																			// Stop as fast as possible: sets new target
      stepper.runToPosition();																// Now stopped after quickstop
    }
  }

  sspeed = stepper.speed();																		// because setCurrentPosition deletes speed we have to backup it
  stepper.setCurrentPosition(0);																	// might be usefull for later development or display
  stepper.setSpeed(sspeed);																		// reconstruct speed 

}	

void nguide_control(){								// function controls movement and direction change of guide
  // needs rotation and endstop 

  if (digitalRead(hall_a_Pin) == LOW)				// Keep rotation status at 0 as long as hall isn't triggered
    rotation_status = 0;


  if (digitalRead(hall_a_Pin) == HIGH && rotation_status == 0 && stepper.distanceToGo() == 0){	// ... and when old move ended initiate new move    

    stepper.move(weg);
    lock--;
    rotation_status = 1;      

Revolutions=Revolutions + 1;
  } 

  if ((digitalRead(guide_EleftPin) == LOW || digitalRead(guide_ErightPin) == LOW) && lock < 1) {
    weg = - weg;									
    lock = 5;										
    stepper.stop();									

  stepper.run();   


}}

