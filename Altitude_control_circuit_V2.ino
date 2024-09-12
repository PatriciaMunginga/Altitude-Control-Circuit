/* This programme is an mini-drone altitude control circuit using an Ultrasonic sensor and Arduino uno
  
   Patricia Munginga
   2023

   Reference: Ben, Finio, Science Buddies,2021
*/

#include <Servo.h>

//define global variables 

Servo ESC1, ESC2;         //create servo object to control the ESC

//define constants
#define MAX_SIGNAL 180
#define MIN_SIGNAL 0

const int trigPin = 7; //pin for the trigger 
const int echoPin = 8; //pin for the echo signal
const int but_pin = 3; //pin for start/stop button
const int min_height = 10; // minimum desired height in centimeters
const int max_height = 50; // maximum desired height in centimeters
const int ramp_time = 40; //delay for motor ramp up/ramp down

// PID variables
unsigned long last_time = 0; 
double Kp = 3; //proportional  gain
double Kd = 0; //derivative gain
double Ki = 0; // integral gain
double error = 0;//difference between target altitude and measured altitude
double errSum, lastErr, dErr ; 
double output ; 
int SampleTime = 100; //0.1 sec, can change this value if system is not producing desired results 


//define other variables
int but_status = 1; //button status
int target_dist = 0; //target distance in centimetres (this will be set by the potentiometer)
int pot_reading = 0; //potentiometer reading 
long duration;// establish variable for the duration of the echo signal and the distance result
long cm; //measured distance in centimeters:
int PWM_signal = 0; //PWM Value
int PWM_offset = 50; // PWM required for drone to hover

void setup() {
  //initialize serial communication
  Serial.begin(9600);

  //Attach the ESC on pin 9
  ESC1.attach(9,1000,2000); //(pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(10,1000,2000);

 //Start ESC calibration process
  Serial.print("ESC calibration process");
  Serial.println(" ");
  delay(1500);
  Serial.println("Starting...");
  delay(1000);
  Serial.println("Now writing maximum output");
  
  //write maximum signal to ESC
  ESC1.write(MAX_SIGNAL);
  ESC2.write(MAX_SIGNAL);

  //wait 2 seconds before sending minimum signal
  delay(5000);

  //write minimum signal to ESC
  Serial.println("Now writing minimum output");

  ESC1.write(MIN_SIGNAL);
  ESC2.write(MIN_SIGNAL);

  pinMode(trigPin, OUTPUT); //set trigger pin as an output
  pinMode(echoPin, INPUT); //set echo pin as an input to receive echo signal
  pinMode(but_pin, INPUT); //set button to input 
  
  Serial.println("Press button to start minidrone");
  WaitForStart(); //wait for button to be pressed to start
  delay(1000);
}

void loop() { 
  //check for button press to stop
  but_status = digitalRead(but_pin); //read button pin
  if(but_status == LOW) //if the button is pressed again to stop
  {
     Land(); // Land slowly to avoid damage
     delay(1000);
     but_status = digitalRead(but_pin);//wait for 1 second then monitor for another button press
     WaitForStart(); //wait for button to be pressed to start again
     LiftOff(); //give drone a short boost to get off the ground and prevent bad sensor readings
  }
  
  //The trigger pin on the ultrasonic sensor is set HIGH for 10 microseconds.
  //In response, the sensor transmits an ultrasonic burst of 8 pulses at 40kHz
  //Meanwhile , the echoPin on the sensor goes HIGH to initiate the reflected signal 
  //and goes LOW once the reflected signal, if any, is received

  //Give the trigger pin a short LOW pulse beforehand to ensure a clean HIGH pulse:

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); //receive duration of echo signal in microseconds
                                     //ie. the time it takes for the ultrasonic signal to hit a surface and come back
  cm  = microsecondsToCentimetres(duration); //convert the duration to a distance in cm
 
  pot_reading = analogRead(A0); //read analog value from the potentiometer, returns a value between 0-1023
  target_dist = map(pot_reading,0,1023,min_height,max_height); //map the potentiometer reading to a height between the minimum and maximum desired height
  
  //PID control
  unsigned long current_time = millis(); //returns number of milliseconds that have passed since the arduino started running the program
  int delta_t = current_time - last_time; //delta time

  if(delta_t >= SampleTime)
  {
    //Compute all the working error variables
    error = target_dist - cm ;
    errSum += error; 
    dErr = (error - lastErr);

    //Compute PID Output

    output = (Kp*error) + (Ki*errSum*SampleTime) + (Kd*dErr)/SampleTime + PWM_offset; 
    
    //Update variables
    lastErr = error;
    last_time = current_time; 

    PWM_signal = (int)output;
  }

  Serial.print(" Error: ");
  Serial.println(error);

  if(PWM_signal>180)
  {
    PWM_signal = 180;
  }
  else if(PWM_signal<=0)
  { 
    PWM_signal = PWM_offset;
  }

  ESC1.write(PWM_signal);  //send the signal to the ESC
  ESC2.write(PWM_signal);

  //Print information for debugging purposes
  Serial.print(" Target distance: ");
  Serial.println(target_dist);
  Serial.print(" Measured distance: ");
  Serial.println(cm);
  Serial.print(" PWM: ");
  Serial.println(PWM_signal);
  Serial.print(" Button Status: ");
  Serial.println(but_status);
  Serial.println();

  //delay(1000);

}

long microsecondsToCentimetres(long microseconds)
{
  // The speed of sound is 340 m/s or 0.034 cm/microsecond
  //Use equation distance = speed*time/2 , divide by two because signal travels to the object and back
  
  return microseconds*0.034/2;
}

void WaitForStart() //wait for button to be pressed to start
{
  while(but_status == HIGH)  //loop until the button is pressed, the but_pin is HIGH when button is not pressed due to the incomplete circuit between the resistor and pushbutton. Once the button is pressed, the pin goes LOW and the loop is exited
  {
    but_status = digitalRead(but_pin); //read button pin
  }
}

void LiftOff()   //slowly ramp up  motor speed to lift off
{
  while(PWM_signal<PWM_offset) //slowly ramp up motor speed for smooth takeoff
  {
    ESC1.write(PWM_signal); //send PWM signal to output pin
    ESC2.write(PWM_signal);
    PWM_signal++;
    delay(ramp_time);
  }
}

void Land()  //slowly ramp down motor speed to land safely
{
  while(PWM_signal>0)
  {
    PWM_signal--;
    ESC1.write(PWM_signal); //send the signal to the ESC
    ESC2.write(PWM_signal);
    delay(ramp_time);
  }
}



