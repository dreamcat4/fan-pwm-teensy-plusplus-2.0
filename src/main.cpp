#include <Arduino.h>

#include <TimerOne.h>
#include <TimerThree.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

// #include <PID_AutoTune_v0.h>
// #include <EnableInterrupt.h>

// my own shorthand
#define uint  uint16_t
#define ulong uint32_t

const uint oneWirePin_B0 = 20;
const uint oneWirePin_B1 = 21;
const uint oneWirePin_B2 = 22;
const uint oneWirePin_B3 = 23;

const uint fanPinInt0_D0 =  0;
const uint fanPinInt1_D1 =  1;
const uint fanPinInt2_D2 =  2;
const uint fanPinInt3_D3 =  3;
const uint fanPinInt4_E4 = 36;
const uint fanPinInt5_E5 = 37;
const uint fanPinInt6_E6 = 18;
const uint fanPinInt7_E7 = 19;

const uint fanPinT1_B5 = 25;
const uint fanPinT1_B6 = 26;
const uint fanPinT1_B7 = 27;
const uint fanPinT3_C4 = 14;
const uint fanPinT3_C5 = 15;
const uint fanPinT3_C6 = 16;

const uint pwm_freq_kz=25; // 25 kHz = 40us


// Analog Read Potentiometer Value
// for wiring, see: https://www.arduino.cc/en/tutorial/potentiometer
// Basically: (pin1) +5v --- WIPER --- (pin2) ANALOG_PIN_A2 --- WIPER --- (pin3) GND
// int potPin = 2;    // select the input pin for the potentiometer
// int pot0_val = 0;       // variable to store the value coming from the sensor
// uint pid_loop_inner_freq = 10;
uint pid_loop_outer_inner_ratio = 7;


// lowest possible value, that still produces a valid pwm signal
// expressed as % percentage of PWM duty cycle
double innerS_min = 06.00;

// highest possible value, that still produces a valid pwm signal
// expressed as % percentage of PWM duty cycle
double innerS_max = 94.00;



double tach_threshold_near_mid = 0.20;
// double tach_threshold_mid_fine = 0.05;

// double tach_threshold_near_mid = 0.160;
double tach_threshold_mid_fine = 0.040;


uint near_overshoots = 0;
uint max_near_overshoots = 3;

double innerS_max_change = 0.3;
double innerS_max_change_abs = innerS_max * innerS_max_change;

struct timer
{
};
// arduino uno atmega328p only has 3 fast timer registers
timer timer0, timer1, timer2;

// a dallas ds18b20 temperature object
struct temp
{
  float temp_min, temp_max, temp_target;
  float temp;
  uint pad;
};
temp temp0, temp1;

// pid object - a single set of pid configuration values
struct pid
{
  double setpoint;
  double kp, ki, kd;
};

struct fan
{
   uint pin_tach, pin_pwm;
   double rpm_min, rpm_max, rpm_target, pwm_max_change_abs;
   uint near_overshoots;
   double tach_min, tach_max, tach_target;
   double rpm, tach, last_tach, pwm, pwm_last;
   uint pad;
};

// example - how to declare a fan instance
// ========================================
// fan FAN_NAME = { pin_tach, pin_pwm, min_rpm, max_rpm, default_rpm, \
//                  pwm_max_change_per_cycle, num_fine_seek_operations_until_settling }

fan fan0 = { fanPinInt0_D0, fanPinT1_B6, 900, 2600, 1500, innerS_max_change_abs, max_near_overshoots };
fan fan1 = { fanPinInt1_D1, fanPinT1_B7, 200, 2000, 1100, innerS_max_change_abs, max_near_overshoots };

// fan fans[] = { fan0, fan1 };
fan fans[] = { fan0 };
uint num_fans = sizeof(fans);

// Setup Dallas DS18b20 Temperature Sensor
// BTW i2c is on pin 2 (middle pin) of the Dallas ds18b20
OneWire oneWire_B0(oneWirePin_B0);
DallasTemperature sensors(&oneWire_B0);

double setPoint, sensor0_temp, Output;
double innerS, innerS_last;
double outerS, outerS_last;

uint innerS_delay = 1700;
// uint innerS_delay = 1000;

// inner pid = ip_, outer pid = op
// aggressive=_fast, medium=_mid, conservative=_fine

// original
// pid ip_fine_orig = { 20, 10, 10 };
// pid ip_fast_orig = { 50, 50, 20 };

// for rpm
// pid ip_fast = {  2.0, 00.5, 0.5 };
// pid ip_fast = { 4, 0.2, 1.0 };
// pid ip_fast = {  8.0, 00.0, 2.0 };

// pid ip_fine = {  2.7, 00.8, 0.6 };
// pid ip_fine = { 2, 0.1, 0.5 };

// for tach
// pid ip_fast = {  70, 40, 40 };
// pid ip_fast = {  30, 20, 20 };

// pid ip_fast = {  140, 80, 80 };
pid ip_fast = {  14.0, 8., 8.0 };

// pid ip_mid  = {  10, 10, 10 };
pid ip_mid  = {  1.0, 1.0, 1.0 };

// pid ip_fine = {  27, 08.8, 6.6 };
// pid ip_fine = {  170, 120, 110 };
// pid ip_fine = {  70, 40, 40 };
// pid ip_fine = {  20, 10, 10 };

// pid ip_fine = {  03, 03, 03 };
pid ip_fine = {  0.3, 0.3, 0.3 };


// PID innerPID(&fan0.rpm, &innerS, &fan0.rpm_target, ip_fine.kp, ip_fine.ki, ip_fine.kd, DIRECT);
// PID innerPID(&fan0.last_tach, &innerS, &fan0.tach_target, ip_fine.kp, ip_fine.ki, ip_fine.kd, DIRECT);
PID innerPID(&fan0.last_tach, &fan0.pwm, &fan0.tach_target, ip_fine.kp, ip_fine.ki, ip_fine.kd, DIRECT);

pid op_fast = { 40, 2.0, 10 };
pid op_med  = { 20, 1.0, 05 };
pid op_slow = { 10, 0.5, 02 };

// pid op_slow2 = { 20, 10, 10 };
// pid op_fast2 = { 50, 50, 20 };
PID outerPID(&sensor0_temp, &outerS, &setPoint, op_slow.kp, op_slow.ki, op_slow.kd, REVERSE);


// fan tachometer

// void fan0_tick()   { fan0.tach++;   Serial.println("fan0_tick()"); }
// void fan1_tick()   { fan1.tach++;   Serial.println("fan1_tick()"); }
void fan0_tick()   { fan0.tach++; }
void fan1_tick()   { fan1.tach++; }

void clear_tachs() { fan0.tach = 0; fan1.tach = 0; }

double tach_to_rpm(uint tach, uint ms_ellapsed)
{
   return ((double)tach * 60 * 1000) / (2 * ms_ellapsed);
}

uint rpm_to_tach(double rpm, uint ms_ellapsed)
{
  // double tach_exact = (rpm * 2 * (double)ms_ellapsed) / (60 * 1000);
  double tach_exact = (rpm * 2 / 60) * ms_ellapsed / 1000;

  // round to nearest whole integer
  return (uint)round(tach_exact);
}

void fan_instance_autofill_tachs(fan &fan_instance)
{
  fan_instance.tach_target = rpm_to_tach(fan_instance.rpm_target, innerS_delay);
  fan_instance.tach_min    = rpm_to_tach(fan_instance.rpm_min,    innerS_delay);
  fan_instance.tach_max    = rpm_to_tach(fan_instance.rpm_max,    innerS_delay);
}

void fan_instance_moderate_pwm(double gap, fan &fan_instance)
{
  if( abs(fan_instance.pwm - fan_instance.pwm_last) > fan_instance.pwm_max_change_abs )
  {
    if( fan_instance.pwm > fan_instance.pwm_last)
    {
      fan_instance.pwm = fan_instance.pwm_last + fan_instance.pwm_max_change_abs;
    }
    else
    {
      fan_instance.pwm = fan_instance.pwm_last - fan_instance.pwm_max_change_abs;       
    }
  }

  if(abs(gap) > tach_threshold_mid_fine)
  {
    fan_instance.pwm_last = fan_instance.pwm;
    fan_instance.near_overshoots = 0;
  }
  else if( (gap < 0) && (fan_instance.near_overshoots < max_near_overshoots) )
  {
    fan_instance.pwm_last = fan_instance.pwm;
    fan_instance.near_overshoots++;
  }
}

unsigned long start, stop;
int loopCounter;

void setup()
{  
  Timer1.initialize(1000/pwm_freq_kz);
  Timer3.initialize(1000/pwm_freq_kz);

  // Setup Pins
  pinMode(fan0.pin_pwm, OUTPUT);
  pinMode(fan0.pin_tach, INPUT);

  pinMode(fan1.pin_pwm, OUTPUT);
  pinMode(fan1.pin_tach, INPUT);

  // setup interupt callbacks
  // arduino uno atmega 328p can only support max 2 digital pin interrupts!!
  attachInterrupt(digitalPinToInterrupt(fan0.pin_tach), fan0_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(fan1.pin_tach), fan1_tick, RISING);

  Serial.begin(9600);
  // Serial.begin(19200);
  Serial.println("Start");

  // Temperature Setup
  sensors.begin();                    //Start Library
  sensors.requestTemperatures();      // Send the command to get temperatures
  sensor0_temp = sensors.getTempCByIndex(0); //Set Input to Current Temperature

  fan_instance_autofill_tachs(fan0);
  clear_tachs();

  // double temp0Min = 25;
  // double temp0Max = 50;
  // double temp0Tgt = setPoint;

  // double temp1Min = 25;
  // double temp1Max = 50;
  // double temp1Tgt = setPoint;
  // double temp2Min = 25;
  // double temp2Max = 50;
  // double temp2Tgt = setPoint;
  // double temp3Min = 25;
  // double temp3Max = 50;
  // double temp3Tgt = setPoint;

  setPoint = 28;                      //Inintialize desired Temperature in Deg C


  // PID Setup
  innerPID.SetMode(AUTOMATIC);
  innerPID.SetOutputLimits(innerS_min, innerS_max);

  outerPID.SetMode(AUTOMATIC);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;  //adjust the PWM Frequency, note: this changes timing like delay()
 
  loopCounter=0;

  // After requesting a temperature sample from a dallas ds18b20, we have to wait for the sensor to respond back
  sensors.setWaitForConversion(false);  // make it async
  sensors.requestTemperatures();
}


// input  = temp
// output = fan rpm value
// output = relay, OTP over temperature timeout period exceeded
void outer_loop()
{
  Serial.println("");
  Serial.println("outer_loop()");

  // // get the potentiometer input value
  // pot0_val = analogRead(potPin);

  // now get the temperature
  sensor0_temp=sensors.getTempCByIndex(0);

  // Request the next temperature conversion in time for the next outer loop. non-blocking / async
  sensors.requestTemperatures();

  //Compute PID value
  double gap = abs(setPoint-sensor0_temp); //distance away from setpoint
  if(gap < 1)
  {
    //Close to setPoint, be conservative
    outerPID.SetTunings(op_slow.kp, op_slow.ki, op_slow.kd);
  }
  else
  {
     //Far from setPoint, be aggresive
     outerPID.SetTunings(op_fast.kp, op_fast.ki, op_fast.kd);
  } 
  outerPID.Compute();

  // Serial.print("  ");
  // Serial.print("pot0_val=");
  // Serial.print(pot0_val);

  Serial.print("    ");
  Serial.print("sensor0_temp=");
  Serial.print(sensor0_temp);


  Serial.print(",   ");
  Serial.print("pidOutput=");
  Serial.println(Output);

  Serial.println("");

}


bool fan1_started = false;

// input  = current fan rpm
// output = desired fan rpm
// output = relay, fan tach failed
void inner_loop()
{
  Serial.println("  inner_loop()");

  fan0.last_tach = fan0.tach;

  Serial.print(",    ");
  Serial.print("fan0.last_tach=");
  Serial.print(fan0.last_tach);

  Serial.print("     ");
  Serial.print("fan0.tach_target=");
  Serial.print(fan0.tach_target);

  fan0.rpm = (fan0.last_tach * 60 * 1000) / (2 * innerS_delay);
  fan1.rpm = (fan1.last_tach * 60 * 1000) / (2 * innerS_delay);

  Serial.print("     ");
  Serial.print("fan0.rpm=");
  Serial.print(fan0.rpm);

  Serial.print("     ");
  Serial.print("fan0.rpm_target=");
  Serial.print(fan0.rpm_target);

  // Serial.print(",   ");
  // Serial.print("fan1_rpm=");
  // Serial.print(fan1_rpm);


  // Compute PID value
  double gap = (fan0.tach_target - fan0.tach) / fan0.tach_target; //distance away from setpoint

  Serial.print("     ");
  Serial.print("gap=");
  Serial.print(gap);

  if(abs(gap) < tach_threshold_mid_fine)
  {
    // Close distance to setPoint
    innerPID.SetTunings(ip_fine.kp, ip_fine.ki, ip_fine.kd);
  }
  else if(abs(gap) < tach_threshold_near_mid)
  {
    // Medium distance to setPoint
    innerPID.SetTunings(ip_mid.kp, ip_mid.ki, ip_mid.kd);
  }
  else
  {
     //Far from setPoint, be aggresive
     innerPID.SetTunings(ip_fast.kp, ip_fast.ki, ip_fast.kd);
  } 

  innerPID.Compute();
  fan_instance_moderate_pwm(gap, fan0);

  Serial.print(",   ");
  Serial.print("fan0.pwm_last (PID)=");
  Serial.println(fan0.pwm_last);


  Timer1.pwm(fanPinT1_B5, (fan0.pwm_last / 100) * 1023);
  Timer1.pwm(fanPinT1_B6, (fan0.pwm_last / 100) * 1023);
  Timer1.pwm(fanPinT1_B7, (fan0.pwm_last / 100) * 1023);

  Timer3.pwm(fanPinT3_C4, (fan0.pwm_last / 100) * 1023);
  Timer3.pwm(fanPinT3_C5, (fan0.pwm_last / 100) * 1023);
  Timer3.pwm(fanPinT3_C6, (fan0.pwm_last / 100) * 1023);


  // if(!fan1_started)
  // {
  //   OCR2B = (((long) (400 + 1) * timer2_OCR2A_Setting) / (1024L*32)) - 1;
  //   fan1_started = true;
  // }


  // OCR2B = (((long) (fan0.pwm_last + 1) * timer2_OCR2A_Setting) / (1024L*256)) - 1;

  // loopCounter++;
  // Serial.println(loopCounter);
  Serial.println("");
}

unsigned long currentMillis  = 0;
unsigned long previousMillis = 0;

void loop()
{
  currentMillis = millis();

  if (currentMillis - previousMillis >= innerS_delay)
  {
    previousMillis = currentMillis;

    inner_loop();
    loopCounter++;

    if (loopCounter % pid_loop_outer_inner_ratio == 0)
      outer_loop();

    clear_tachs();
  }
}





