#include "MeMCore.h" // reference from https://github.com/Makeblock-official/Makeblock-Libraries

MeLineFollower lineFinder(PORT_1); // linefollower sensor connected to port 1
MeUltrasonicSensor ultraSensor(PORT_2); // ultrasonic sensor connected to port 2

MeDCMotor motor1(M1);// left motor connected to M1
MeDCMotor motor2(M2);// right motor connected to M1

int lineSensor; //input for linesensor
int ir_val; //input value for ir
float dist; // input for ultrasonic sensor
float ir_dist; // input for calculated ir distance


uint8_t motorSpeed = 200; // motor speed when moving forward and turing left and right 90 degree
uint8_t slow_speed = 70; // speed for the the side of the motor which the robot is turing while trying to maintain a straight path
uint8_t fast_speed = 150; //speed for the the side of the motor which the robot is too close to the wall
int right_forward_delay = 770; // time required for robot to move forward to reach next grid in between the successive right turns
int left_forward_delay = 790; // time required for robot to move forward to reach next grid in between the successive left turns
int left_turn_delay = 328; // time required for left turn to complete
int right_turn_delay = 335; // // time required for left turn to complete
int turn_180_delay = right_turn_delay * 2; // time required for 180 degree turn to complete

int ir_count = 0;
int base_ir;
int ir_input = A0; //port 4 s1
int color_input = A1; // port 4 s2
int A = A2; // port 3 s1
int B = A3; //port 3 s2

// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 500 //in milliseconds 

// Define time delay before taking another LDR reading
#define LDRWait 10 //in milliseconds 

#define LDR 1  //LDR sensor pin at A1 // Port 4 S2
#define LED 13  //Check Indicator to signal Calibration Completed
//placeholders for colour detected
int red = 0;
int green = 0;
int blue = 0;

//floats to hold colour arrays
float colourArray[] = {0, 0, 0}; // float for R G B value for colour detected
float whiteArray[] = {986, 980, 970}; // float for maximum average possible readings for r,g and b
float blackArray[] = {853.00, 729, 654}; //float for minimum average possible readings for r,g and b
float greyDiff[] = {133, 251, 316}; // float for maximum possible range

char colourStr[3][5] = {"R = ", "G = ", "B = "};

// For sound at the exit
MeBuzzer buzzer;
char i;
int note, duration;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); //begin serial communication

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  //setBalance();  //calibration to obtain grey diff for colour sensor before the actual run

}

void loop() {
  lineSensor = lineFinder.readSensors(); // read inputs from line sensors
  record_baseline_voltage(); // record baseline voltage when emitter is turned off every 10 loops

  if (lineSensor == S1_IN_S2_IN)// both sensors detect black_line.
  {
    motor_stop();
    read_ldr(); // obtain value for r,g and b respectively
    execute_turn(); // differentiate the colour based on r,g,b values and execute instruction
  }
  else
  {
    turn_off_led(); //turn on ir emitter.
    ir_val = analogRead(ir_input) ; // ir_input is A0;
    dist = ultraSensor.distanceCm(); // read the input from ultrasonic sensor
    ir_dist = calc_ir_dist(ir_val - base_ir); // calcuate distance detected by IR after taking into account of ambient light

    if (dist > 25) { // ultrasonic detects empty wall, use ir to maintain straight path
      if (ir_dist < 6.5) { //ir detects mbot close to right side of the wall
        turn_left_slowly(); // move left
      }
      else if (ir_dist > 10.5 && ir_dist < 11.5) { //ir detects mbot close to left side of the wall
        turn_right_slowly(); //move left
      }
      else {
        move_forward(); // both side detects no wall, move forward
      }
    }
    else if (dist < 7) {  // ultrasonic detects mbot close to left side of the wall
      turn_right_slowly();// move right
    }
    else { //ultrasonic detects mbot close to right side of the wall
      turn_left_slowly(); // move left
    }
  }
}


float calc_ir_dist(int input) // calculate distance detected by the ir using equation obtained
{
  float output = 0.00009 * input * input - 0.0969 * input + 29.745;
  return output;
}

void record_baseline_voltage() // record base line voltage every 10 loops
{
  if (ir_count == 0)
  {
    turn_on_led(1);// turn off ir emitter
    base_ir = analogRead(ir_input); // record baseline voltage
  }
  else if (ir_count == 9) // if is 9, restart the cycle.
  {
    ir_count = 0;
  }
  else //else add 1 each time
  {
    ir_count += 1;
  }
}

void execute_turn() // detects colour and turns acoordingly
{
  if (colourArray[0] > 220) // red, orange and purple has red vlaue higher than 220.
  {
    if (colourArray[1] > 230 && colourArray[2] > 230) // white has green and blue value higher than 230.
    {
      // robot detects white: stops moving and plays a celebratory tone
      motor_stop();
      play_tune();
    }
    else if (colourArray[1] > 90) //green value of orange is much higher than 90
    {
      turn_180(); //detects orange, turn 180 degree
    }
    else //green value of red is much less than 90
    {
      turn_left(0); // detects red: turn left
    }
  }
  else if (colourArray[1] > 195) //separate blue and green from purple
  {
    if (colourArray[2] > colourArray[1]) // blue value higher than green value for blue
    {
      successive_right_turns(); //detects blue: two successive right turns
    }
    else //detects green
    {
      turn_right(0);
    }
  }
  else if (colourArray[0] < 205 && colourArray[1] < 205) // both green and blue value of purple are much lower than 205
  {
    successive_left_turns(); //detects purple: two successive left turns
  }
  else
  {
    read_ldr(); //colour is not confirm, recheck the rgb reading to determine the colour again
  }
}

void read_ldr()
{
  //turn on one colour at a time and LDR reads 5 times
  for (int c = 0; c <= 2; c++)
  {
    turn_on_led(c);  //turn ON the LED, red, green or blue, one colour at a time.
    delay(RGBWait);

    //get the average of 5 consecutive readings for the current colour and return an average
    colourArray[c] = getAvgReading(5);

    //the average reading returned minus the lowest value divided by the maximum possible range,
    //multiplied by 255 will give a value between 0-255, representing the value for the
    //current reflectivity (i.e. the colour LDR is exposed to)
    colourArray[c] = (colourArray[c] - blackArray[c]) / (greyDiff[c]) * 255;
    turn_off_led(); //turn off the current LED colour
    delay(RGBWait); //time delay before the next RGB colour turns ON to allow LDR to stabilize
  }
}

int getAvgReading(int times)  //find the average reading for the requested number of times of scanning LDR
{
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++)
  {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  //calculate the average and return it
  return total / times;
}

void turn_off_led() // turn off led and enable ir emitter
{
  digitalWrite(B, LOW);
  digitalWrite(A, LOW);
}

void turn_on_led(long i) // turn on led and disable ir emitter
{
  if (i == 0) // turn on red led
  {
    digitalWrite(B, HIGH);
    digitalWrite(A, HIGH);
  }
  else if (i == 1) // turn on green led
  {
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
  }
  else // turn on blue led
  {
    digitalWrite(B, HIGH);
    digitalWrite(A, LOW);
  }
}

void turn_left_slowly() // right motor moves faster than right motor, turn right while moving forward
{
  motor1.run(-slow_speed);
  motor2.run(fast_speed);
}

void turn_right_slowly() // left motor moves faster than right motor, turn right while moving forward
{
  motor1.run(-fast_speed);
  motor2.run(slow_speed);
}


void move_forward() // motor moves forward
{
  motor1.run(-motorSpeed);
  motor2.run(motorSpeed);
}

void turn_left(int num) // execute left turn with param[in] num to accomdate for the different time reuqired to complete a left turn
{
  motor1.run(motorSpeed);
  motor2.run(motorSpeed);
  delay(left_turn_delay + num);
  motor_stop();
}

void turn_right(int num) // execute right turn with param[in] num to accomdate for the different time reuqired to complete a right turn
{
  motor1.run(-motorSpeed);
  motor2.run(-motorSpeed);
  delay(right_turn_delay + num);
  motor_stop();
}

void turn_180() // execute 180 degree turn within the same grid
{
  motor1.run(-motorSpeed);
  motor2.run(-motorSpeed);
  delay(turn_180_delay);
  motor_stop();
}


void successive_left_turns() // execute two successive right-turns in two grids
{
  turn_left(0);
  delay(500);
  move_forward();
  delay(left_forward_delay);
  motor_stop();
  delay(500);
  turn_left(20);

}

void successive_right_turns() // execute two successive left-turns in two grids
{
  turn_right(0);
  delay(500);
  move_forward();
  delay(right_forward_delay);
  motor_stop();
  delay(500);
  turn_right(-10);
}

void motor_stop() // stop the robot
{
  motor1.stop();
  motor2.stop();
}

void play_tune() // plays the tune required when detecting white after stopping in front of black paper
{                // Reference from mBot lecture by Prof Ravi
  for (i = 0; i < 10; i ++)
  {
    note = random(100, 1500); // Freq range of numbers
    duration = random(50, 300); // Duration for each notes
    buzzer.tone(note, duration);
  }
}

void setBalance(){ //Reference from Week 9 Studio 1 colour sensor code by Dr Henry Tan
//set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);           //delay for five seconds for getting sample ready
  digitalWrite(LED,LOW); //Check Indicator OFF during Calibration
//scan the white sample.
//go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  for(int i = 0;i<=2;i++){
     turn_on_led(i);
     delay(RGBWait);
     whiteArray[i] = getAvgReading(5); //scan 5 times and return the average, 
     turn_off_led();
     delay(RGBWait);
  }
//done scanning white, time for the black sample.
//set black balance
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);     //delay for five seconds for getting sample ready 
//go through one colour at a time, set the minimum reading for red, green and blue to the black array
  for(int i = 0;i<=2;i++){
     turn_on_led(i);
     delay(RGBWait);
     blackArray[i] = getAvgReading(5);
     turn_off_led();
     delay(RGBWait);
//the differnce between the maximum and the minimum gives the range
     greyDiff[i] = whiteArray[i] - blackArray[i];
  }
//delay another 5 seconds for getting ready colour objects
  Serial.println("Colour Sensor Is Ready.");
  print_calibration(); // print to the serial monitor the set of white, black and grey for each led
}

void print_calibration() //print the maximum, least possible value and range of value for each led to the serial monitor
{
  for (int i = 0; i < 3; i += 1)
  {
     Serial.print("whiteArray: ");
     Serial.print(whiteArray[i]);
     Serial.print(" blackArray: ");
     Serial.print(blackArray[i]);
     Serial.print(" greyDiff: ");
     Serial.println(greyDiff[i]);
  }
}
