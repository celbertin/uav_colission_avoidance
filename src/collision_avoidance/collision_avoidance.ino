#include <Maxbotix.h>
#include <PID_v1.h>
#include "Kalman.h"

int ppmRead[8];  //array for storing up to 8 signals
//#define DEBUG

//- - - - SETUP - - - - 
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000Âµs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 0 is positive, 1 is negative
#define multiplier F_CPU/8000000  //just leave this

#define PPM_Pin 18  //input
#define sigPin 19  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppmWrite[chanel_number];

const int anPin1 = 6;
const int anPin2 = 7;
const int anPin3 = 8;
const int anPin4 = 9;
const int anPin5 = 10;
const int anPin6 = 11;

int count_sensors = 3; //sensors to read  - read up until sensor number 

boolean sensorActive[] = {false, false, true, false, false, false}; //avoidance active or not for that sensor
double sensorSafetyDist[] = {60.0, 60.0, 80.0, 60.0, 60.0, 100.0};  //safety distance per sensor

int triggerPin1 = 5; //trigger ping ultrasonic chain sensors

double safetyField=60.0; // safety distance in centimeters

double distance[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //latest measured distance
double kalman_distance[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //latest kalman value

//Kalman(double process_noise, double sensor_noise, double estimated_error, double intial_value) 
//Kalman(Q,R,P,X) 

Kalman kalman_us_1 = Kalman(0.3,2,3,70); //front
Kalman kalman_us_2 = Kalman(0.3,2,3,70); //left
Kalman kalman_us_3 = Kalman(0.3,2,3,70); //top
Kalman kalman_us_4 = Kalman(0.3,2,3,70); //right
Kalman kalman_us_5 = Kalman(0.3,2,3,70); //back
Kalman kalman_us_6 = Kalman(0.3,2,3,70); //bottom

double crash[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //if zero then no crash detected, else crash[i] = distance

double Setpoint; //safety distance for PID
double Input1, Input2, Input3, Input4, Input5, Input6;
double Output1, Output2, Output3, Output4, Output5, Output6;

PID PID1(&Input1, &Output1, &Setpoint,1.2,1.75,0.6, P_ON_M, REVERSE); //front
PID PID2(&Input2, &Output2, &Setpoint,1.2,1.75,0.6, P_ON_M, REVERSE); //left
PID PID3(&Input3, &Output3, &Setpoint,1.2,1.75,0.6, P_ON_M, REVERSE); //top
PID PID4(&Input4, &Output4, &Setpoint,1.2,1.75,0.6, P_ON_M, DIRECT); //right
PID PID5(&Input5, &Output5, &Setpoint,1.2,1.75,0.6, P_ON_M, DIRECT); //back
PID PID6(&Input6, &Output6, &Setpoint,1.2,1.75,0.6, P_ON_M, DIRECT); //bottom

Maxbotix rangeSensor1(anPin1, Maxbotix::AN, Maxbotix::LV, Maxbotix::BEST);
Maxbotix rangeSensor2(anPin2, Maxbotix::AN, Maxbotix::LV, Maxbotix::BEST);
Maxbotix rangeSensor3(anPin3, Maxbotix::AN, Maxbotix::LV, Maxbotix::BEST);

unsigned long start;

void setup() {

  Serial.begin(115200);
  Serial.println("ready");
  Serial2.begin(57600);
  
 //PID Setings
  Setpoint = 0.0; //safety distance for PID
  Input1 = kalman_distance[0] - sensorSafetyDist[0];
  Input2 = kalman_distance[1] - sensorSafetyDist[1];
  Input3 = kalman_distance[2] - sensorSafetyDist[2];
  
  PID1.SetOutputLimits(1300, 1520); // 1500 es neutro, 1300 es retroceso 1510 para contrarrestar momentum
  PID1.SetMode(AUTOMATIC);
  PID1.SetSampleTime(25);

  PID2.SetOutputLimits(1300, 1520); // 1500 es neutro, 1300 es derecha
  PID2.SetMode(AUTOMATIC);
  PID2.SetSampleTime(25);

  PID3.SetOutputLimits(1350, 1425); // ~1450 neutro
  PID3.SetMode(AUTOMATIC);
  PID3.SetSampleTime(25);

/*
  PID4.SetOutputLimits(1500, 1700); // 1500 es neutro, 1700 es retroceso
  PID4.SetMode(AUTOMATIC);
  PID4.SetSampleTime(25);

  PID5.SetOutputLimits(1500, 1700); // 1500 es neutro, 1700 es izquierda
  PID5.SetMode(AUTOMATIC);
  PID5.SetSampleTime(25);

  PID6.SetOutputLimits(1500, 1600); // 1500 es neutro, 1700 es izquierda
  PID6.SetMode(AUTOMATIC);
  PID6.SetSampleTime(25);
  */
  pinMode(triggerPin1, OUTPUT);

////////////////// reading ppm ///////////////////////
  pinMode(PPM_Pin, INPUT);
  attachInterrupt(5, read_ppm, CHANGE);
  //attachInterrupt(PPM_Pin - 2, read_ppm, CHANGE);
////////////////////////////////////////
  
//////////////// generating ppm ///////////////////////
  for(int i=0; i<chanel_number; i++){
    ppmWrite[i]= default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
////////////////////////////////////////////////////

rangeSensor1.setADSampleDelay(2);
rangeSensor2.setADSampleDelay(2);
rangeSensor3.setADSampleDelay(2);
}

void loop() {
  start=millis();
  avoidance();  
  read_all_sensors();
  print_all();
  check_all_collisions();
  check_all_PID();  
  Serial2.println(millis() - start);
}

// --- FUNCTIONS --- 

void check_all_PID(){
  Input1 = kalman_distance[0] - sensorSafetyDist[0];
  PID1.Compute();
  Input2 = kalman_distance[1] - sensorSafetyDist[1];
  PID2.Compute();
  Input3 = kalman_distance[2] - sensorSafetyDist[2];
  PID3.Compute();
  /*
  Input4 = kalman_distance[3] - safetyField;
  PID4.Compute();
  Input5 = kalman_distance[4] - safetyField;
  PID5.Compute();
  Input6 = kalman_distance[5] - safetyField;
  PID6.Compute();
  */
  
  }
  
void check_all_collisions(){ //checks collision on all sensors, sets avoidance values
  for(int i=0;i<count_sensors;i++){
       collision(sensorSafetyDist[i],kalman_distance[i],(i+1)); //
    }
  }

void collision(double area, double distance, int direction_sensor){ //sets avoidance values in crash[]
  double delta = area-distance;
  if(distance<area && distance>0.0){
    crash[direction_sensor-1] = delta;
    }
  else{
    crash[direction_sensor-1] = 0.0;
    }
  }

 void avoidance(){
  bool collision_detected = false;
  int sensor_avoid=0;
  int num_detected = 0;
  for(int k=0;k<count_sensors;k++){ //check if there are any collision
    if(crash[k]>0.0){
      collision_detected = true;
      num_detected++;
      sensor_avoid = k+1;
    }
  }
    
  if(collision_detected && ppmRead[6]>1650){ //there is collision and ACA turned on with channel 7 high        
    if(crash[0]!=0.0 && sensorActive[0]){
      ppmWrite[1]=int(Output1); //pitch backwards PID
      }
    else{
      ppmWrite[1]=ppmRead[1];
      }
    if(crash[1]!=0.0 && sensorActive[1]){
      ppmWrite[0]= int(Output2); //roll to the right
      }
    else{
      ppmWrite[0]=ppmRead[0];
      }
    if(crash[2]!=0.0 && sensorActive[2]){
      ppmWrite[2]= int(Output3); //throttle lower
      }
    else{
      ppmWrite[2]=ppmRead[2];
      }
    
    ppmWrite[3]=ppmRead[3]; //yaw
    for(int j=4; j<chanel_number; j++){ //send to drone RC values from channel 5 to chanel_number
      ppmWrite[j] = ppmRead[j];
      }
    }  
  else{
    for(byte i=0; i<chanel_number; i++){ //send to drone RC values for all channels
      ppmWrite[i] = ppmRead[i];
      }
    }
  }

void read_all_sensors(){
  start_sensor();
  read_sensors();
  //delay(50); //wait for all sensors to have been read in sequence         count_sensors*
  }
  
void start_sensor(){
digitalWrite(triggerPin1,HIGH);
delay(1);
digitalWrite(triggerPin1,LOW);
  }
  
void read_sensors(){
  /*
  Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
  Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
  */
  distance[0] = rangeSensor1.getRange();
  kalman_distance[0] = kalman_us_1.getFilteredValue(distance[0]);   //calc and store kalman value

  distance[1] = rangeSensor2.getRange();
  kalman_distance[1] = kalman_us_2.getFilteredValue(distance[1]);
  
  distance[2] = rangeSensor3.getRange();
  kalman_distance[2] = kalman_us_3.getFilteredValue(distance[2]);

  /*
  distance[3] = long(analogRead(anPin4)/2);
  distance[3] = distance[3]*2.54;
  kalman_distance[3] = kalman_us_4.getFilteredValue(distance[3]);
  
  distance[4] = long(analogRead(anPin5)/2);
  distance[4] = distance[4]*2.54;
  kalman_distance[4] = kalman_us_5.getFilteredValue(distance[4]);
  
  distance[5] = long(analogRead(anPin6)/2);
  distance[5] = distance[5]*2.54;
  kalman_distance[5] = kalman_us_6.getFilteredValue(distance[5]);
  */
  
}

void read_ppm(){  //reading ppm (interrupt service routine)
  static unsigned int pulse;
  static unsigned long counter;
  static byte channel;
  static unsigned long last_micros;

  counter = micros() - last_micros;
  last_micros = micros();

  if(counter < 510){  //must be a pulse
    pulse = counter;
  }
  else if(counter > 1910){  //sync pulse
    channel = 0;
    
    #if defined(DEBUG)
    static unsigned long last_micros2;
    Serial.print("PPM Frame Len: ");
    Serial.println(micros() - last_micros2);
    last_micros2 = micros();
    #endif
  }
  else{  //servo values
    ppmRead[channel] = counter + pulse;
    //ppmWrite[channel] = ppmRead[channel];
    #if defined(DEBUG)
    Serial.print(ppmRead[channel]);
    Serial.print("  ");
    #endif
    
    channel++;
  }
}


ISR(TIMER1_COMPA_vect){  //ppm generation
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppmWrite[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppmWrite[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

void print_all(){
  //front
  Serial2.print(distance[0]);//front
  Serial2.print(",");
  Serial2.print(kalman_distance[0]);
  Serial2.print(",");
  Serial2.print(Output1);
  Serial2.print(",");
  Serial2.print(ppmWrite[1]);
  Serial2.print(",");
  Serial2.print(distance[1]); //left
  Serial2.print(",");
  Serial2.print(kalman_distance[1]);
  Serial2.print(",");
  Serial2.print(Output2);
  Serial2.print(",");
  Serial2.print(ppmWrite[0]);
  Serial2.print(",");
  Serial2.print(distance[2]); //top
  Serial2.print(",");
  Serial2.print(kalman_distance[2]);
  Serial2.print(",");
  Serial2.print(Output3);
  Serial2.print(",");
  Serial2.print(ppmWrite[2]);
  Serial2.print(",");
  //Serial2.println(millis() - start);
}

void printArray(int *a, int n)
{
  for (int i = 0; i < n; i++)
  {
    Serial2.print(a[i], DEC);
    Serial2.print(' ');
  }

  Serial2.println();
}

//Sorting function
// sort function (Author: Bill Gentles, Nov. 12, 2010)
void isort(int *a, int n) {
  //  *a is an array pointer function

  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

//Mode function, returning the mode or median.
int mode(int *x, int n) {
  int i = 0;
  int count = 0;
  int maxCount = 0;
  int mode = 0;
  int bimodal;
  int prevCount = 0;

  while (i < (n - 1)) {
    prevCount = count;
    count = 0;

    while (x[i] == x[i + 1]) {
      count++;
      i++;
    }

    if (count > prevCount & count > maxCount) {
      mode = x[i];
      maxCount = count;
      bimodal = 0;
    }
    if (count == 0) {
      i++;
    }
    if (count == maxCount) { //If the dataset has 2 or more modes.
      bimodal = 1;
    }
    if (mode == 0 || bimodal == 1) { //Return the median if there is no mode.
      mode = x[(n / 2)];
    }
    return mode;
  }
}

