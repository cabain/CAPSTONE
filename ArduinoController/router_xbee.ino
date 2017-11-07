//Include Statements for the BNO055 Sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>
//include statements for the timer
#include <avr/io.h>
#include <avr/interrupt.h>
//Definitions
Adafruit_BNO055 bno = Adafruit_BNO055();

//First coil's pins/states
const int FrontLogicPin = 2;
const int FrontForwardPin = 3;
const int FrontReversePin = 4;
int FrontLogicState = LOW;
int FrontForwardState = LOW;
int FrontReverseState = LOW;
//Second coil's pins/states
const int SideLogicPin = 5;
const int SideForwardPin = 6;
const int SideReversePin = 7;
int SideLogicState = LOW;
int SideForwardState = LOW;
int SideReverseState = LOW;

int report_heading = 0; //variable used to determine whether it is time to report heading.  Based off Timer1.
bool doneCalibrating = false;
double highestDegree = 0;
double highestMagnetometer = 0;
double targetYaw = 90; //abstract value to begin
double currentYaw;



void setup()
{
  //set the pin modes
  pinMode(FrontLogicPin, OUTPUT);
  pinMode(FrontForwardPin, OUTPUT);
  pinMode(FrontReversePin, OUTPUT);
  pinMode(SideLogicPin, OUTPUT);
  pinMode(SideForwardPin, OUTPUT);
  pinMode(SideReversePin, OUTPUT);
  InitializeTimer1();  //Initialize Timer1

  //begin baudrate frequency 9600
  Serial.begin(9600);  //Physical Serial, "portOne"
  //setup for the BNO055 sensor
  bno.begin();
  bno.setExtCrystalUse(true);
  //calibration
  Serial.println("Calibrating...");
  //assignMagtometerData();
  Serial.print("Relative to initialization "); Serial.print(highestDegree); Serial.println(" is magnetic north");
}



void loop()  //Use the main loop to execute pending functions, and LISTEN for commands
{
  printHeadingReport();
  detectIncomingCommand();
  //controlSys();
}



void controlSys()
{
  //Four Different Initial Cases regarding the current/target yaw
  if (currentYaw > targetYaw && (abs(currentYaw - targetYaw) < 180))
  {
    //turn counter-clockwise
    //calculate the difference between the two points
    double difference = currentYaw - targetYaw;
  }
  if (currentYaw > targetYaw && (abs(currentYaw - targetYaw) > 180))
  {
    //turn clockwise
    //calculate the difference between the two points
    double difference = (360 - currentYaw) + targetYaw;
  }
  if (currentYaw < targetYaw && (abs(currentYaw - targetYaw) < 180))
  {
    //turn clockwise
    //calculate the difference between the two points
    double difference = targetYaw - currentYaw;
  }
  if (currentYaw < targetYaw && (abs(currentYaw - targetYaw) > 180))
  {
    //turn counter-clockwise
    //calculate the difference between the two points
    double difference = currentYaw + (360 - targetYaw);
  }
}



void detectIncomingCommand()
{
  //Read the incoming serial data
  if (Serial.available())
  {
    int nextVar = Serial.read();
    if (nextVar == 65) //ASCII Value for 'A'
    {
      Serial.write('A');
      //Assign states to turn the front coil FORWARD - ON
      FrontLogicState = HIGH;
      FrontForwardState = HIGH;
      FrontReverseState = LOW;
      //Write the states
      digitalWrite(FrontLogicPin, FrontLogicState);
      digitalWrite(FrontForwardPin, FrontLogicState);
      digitalWrite(FrontReversePin, FrontReverseState);
    }
    
    if (nextVar == 66) //ASCII Value for 'B'
    {
      Serial.write('B');
      //Assign states to turn the front coil REVERSE - ON
      FrontLogicState = HIGH;
      FrontForwardState = LOW;
      FrontReverseState = HIGH;
      //Write the states
      digitalWrite(FrontLogicPin, FrontLogicState);
      digitalWrite(FrontForwardPin, FrontForwardState);
      digitalWrite(FrontReversePin, FrontReverseState);
    }
    if (nextVar == 67) //ASCII Value for 'C'
    {
      Serial.write('C');
      //Assign states to turn the front coil OFF
      FrontLogicState = LOW;
      FrontForwardState = LOW;
      FrontReverseState = LOW;
      //Write the states
      digitalWrite(FrontLogicPin, FrontLogicState);
      digitalWrite(FrontForwardPin, FrontForwardState);
      digitalWrite(FrontReversePin, FrontReverseState);
    }
    
    if (nextVar == 68) //ASCII Value for 'D'
    {
      Serial.write('D');
      //Assign states to turn the side coil FORWARD - ON
      SideLogicState = HIGH;
      SideForwardState = HIGH;
      SideReverseState = LOW;
      //Write the states
      digitalWrite(SideLogicPin, SideLogicState);
      digitalWrite(SideForwardPin, SideForwardState);
      digitalWrite(SideReversePin, SideReverseState);
    }
    if(nextVar == 69) //ASCII Value for 'E'
    {
      Serial.write('E');
      //Assign states to turn the side coil REVERSE - ON
      SideLogicState = HIGH;
      SideForwardState = LOW;
      SideReverseState = HIGH;
      //Write the states
      digitalWrite(SideLogicPin, SideLogicState);
      digitalWrite(SideForwardPin, SideForwardState);
      digitalWrite(SideReversePin, SideReverseState);
    }

    if (nextVar == 70) //ASCII Value for 'F'
    {
      Serial.write('F');
      //Assign states to turn the side coil OFF
      SideLogicState = LOW;
      SideForwardState = LOW;
      SideReverseState = LOW;
      //Write the states
      digitalWrite(SideLogicPin, SideLogicState);
      digitalWrite(SideForwardPin, SideForwardState);
      digitalWrite(SideReversePin, SideReverseState);
    }
  }
}



void assignMagtometerData()
{
  boolean zeroAgain = false;
  boolean leftZero = false;
  while (zeroAgain == false)
  {
    //get the IMU Heading
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    double x_degree = euler.x();
    //get the Magnetometer reading
    imu::Vector<3> magn = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    double x_magnetometer = magn.x();
    if (((x_degree > 10 && x_degree < 180) || (x_degree < 350 && x_degree > 180)) && leftZero == false)
    {
      Serial.println("Left initializing range.");
      leftZero = true;
    }
    if ((x_degree < 1 || x_degree > 359 ) && leftZero == true)
    {
      zeroAgain = true;
      doneCalibrating = true;
    }
    if (x_magnetometer > highestMagnetometer)
    {
      //assign new highest degree
      highestDegree = x_degree;
      highestMagnetometer = x_magnetometer;
    }
  }
}



void printHeadingReport()
{
  //See if it is time to report heading
  if (report_heading == 1)
  {
    //report the heading and reset report_heading back to default value of 0.
    currentYaw = getHeading();
    char buff[10];      //load the heading into the buffer
    sprintf(buff, "%d.%2d\n", int(currentYaw), frac(currentYaw));
    printbuffer(buff);      //use the print buffer to send one character at a time
    report_heading = 0;
  }
}


float getHeading()//self-explanatory
{
  //get the IMU Heading
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double x_degree = euler.x();
  double deviation = 360 - highestDegree;
  float heading = x_degree + 90;
  if (heading > 360)
  {
    heading = heading - 360;
  }
  return heading;
  //THE SYSTEM IS CONSTANTLY CALIBRATING.  TURN OFF WHEN USING ELECTROMAGENTICS (change operation mode)
}



//Print Buffer for XBee.  Send ASCII values, a char at a time.
void printbuffer(char *buffer)
{
  while (*buffer) {
    int val = *buffer++;
    if (val == 32)
    {
      Serial.write(48);
    }
    else
    {
      Serial.write(val);
    }
  }
  //Send an end of XMISSION that we've reached the end
  //Serial.write("E");
}

// this little function will return the first two digits after the decimal
// point of a float as an int to help with sprintf() (won't work for negative values)
// the .005 is there for rounding.
int frac(float num) {
  return ( ((num + .005) - (int)num) * 100);
}


void InitializeTimer1()
{
  //Initialize Timer1
  cli(); //disable global interupts
  TCCR1A = 0; //set the entire TCCR1A register to 0
  TCCR1B = 0;

  //set compare match register to desired timer count
  /*
    (# timer counts + 1) = (target time) / (timer resolution)
    (# timer counts + 1) = (1 s) / (6.4e-5 s)
    (# timer counts + 1) = 15625
    (# timer counts) = 15625 - 1 = 15624
  */
  OCR1A = 15624;
  //turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  //enable Timer1 COMPARE interupt
  TIMSK1 = (1 << OCIE1A);
  sei(); //enable global interupts
}


//runs when TIMER1 overflows
ISR(TIMER1_COMPA_vect)
{
  //set value to report heading
  report_heading = 1;
  //report_gyro = 1;
}



