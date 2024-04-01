//IMPORTANT: When connecting SX1 motor from Grin Technologies (ebikes.ca), follow instructions
  //Connect Blue Phase wire to Channel A
  //Connect Yellow Phase wire to Channel B
  //Connect Green Phase wire to Channel C 
  //this arrangement means that Hall C aligns with Phase C, Hall B aligns with Phase B, and Hall A aligns with Phase A

//would also like to implement constant power operation, reduce phase current limit when at high speed
//Potentially also implement field weakening control, current in q axis is reduced and current in d axis is increased while at maximum speed
//need a better estimate of the rotor position, efficient foc requires accurate rotor position estimation
//should add temperature limits/sensing for both on-board temperature and motor temperature (both have probes)
//should add wheel speed sensing to the code, has digital pin connected, put on interrupt? 

#include <math.h>

int LED0 = 13;                 //connects pin 13 to onboard LED
int LED1 = 37;
int Switch1 = 31;              //connects to switch on pin 0, switch is either high or low
int Switch2 = 32;

int HallA = 1;                //connects the digital pin 1 to the A phase Hall sensor, unsure of loctation and which phase to corrrelate with
int HallB = 2;                //connects the digital pin 2 to the B phase Hall sensor, unsure of loctation and which phase to corrrelate with
int HallC = 3;                //connects the digital pin 3 to the C phase Hall sensor, unsure of loctation and which phase to corrrelate with

int PWMA = 5;                 //connects the digital pin 5 to the PWM input of phase A, limit is 1007 of 1023, cannot achieve full range
int SDPhA = 4;                  //connects the digital pin 4 to the Shutdown pin of phase A
int PWMB = 7;                 //connects the digital pin 6 to the PWM input of phase B, limit is 1007 of 1023, cannot achieve full range
int SDPhB = 6;                  //connects the digital pin 7 to the Shutdown pin of phase B
int PWMC = 9;                 //connects the digital pin 9 to the PWM input of phase C, limit is 1007 of 1023, cannot achieve full range
int SDPhC = 8;                  //connects the digital pin 8 to the Shutdown pin of phase C

int pot = 22;                 //connects potentiometer to analog pin 23
int f = 50000;                //setting switching frequency to 20 kHz, Motor controller V4 with IR2101 IC's only capable of up to 5 kHz, values above this hitch when FET is unable to keep up
                              //limit with Teensy 4.1 is up to 146484.38 Hz with CPU at 600 MHz

int DCV = 21;
int PhAV = 41;                //setting up pins for phase voltage analog read
int PhBV = 40;                //setting up pins for phase voltage analog read
int PhCV = 39;                //setting up pins for phase voltage analog read
int NeuV = 38;                //setting up pins for neutral voltage analog read

int PhAI = 25;                //setting up pins for phase A current sense
int PhBI = 26;                //setting up pins for phsae B current sense
int PhCI = 27;                //setting up pins for phase C current sense
int DCI  = 23;                //does not work at the moment

int MotorTemp = 24;           //Pin connected to motor connector, reads temperature of motor windings
int FetTemp = 14;             //This analog pin reads the on-board thermistor for the FET Temperatures

int Speedometer = 0;          //this is digital read for the 6 pole wheel speed hall sensor on the SX1 motor from Grin Technologies




/*
//Black Flipsky E-board motor has phase - phase inductance of ~60 uH, phase - phase resistance of 65 mOhms, has 14 poles, 7 pole pairs, claims 110 RPM/V
double SpeedConst = 110.0; //this is speed constant of the motor, for Flipsky 7070 E-Board Motor the Kv is 110 RPM/V
double PolePairs = 7.0;           //Machine constant, SX1 motor has 15 pole pairs, used to calculate actual motor RPM
unsigned long PhPhR = 0.065;  //phase to phase resistance of motor
int MaxPhCurr = 20;           //max phase current of motor, 20 amp (peak) phase current with phase resistance of 0.072 Ohms results in 1.44 V + phase voltage is max, PWM needs to be adjusted for speed, need KV as well or use current sensors instead
double BackEMFV = 0.0;
double KpD = 0.0;
double KiD = 0.0;
double KpQ = 0.0; 
double KiQ = 0.0; 
int Offset_Angle = 125; //Angle used to align the stator with phase A high = 0 degrees and rotor with transition between Hall state 1 and hall state 5 being zero degrees (with 5 -> 1 being the positive direction)
*/

//Black unbranded hobby motor from Amazon, likely this one: https://www.amazon.com/Efficience-Brushless-Sensored-Skateboard-Longboarding/dp/B07PNMYHFT/ref=sr_1_31?crid=1D3H8FKCKOL0V&dib=eyJ2IjoiMSJ9.aGiiL2SAsS47-btR03Nw4JufuhSFOISWnKEUWHlbZoftZRPYQx58loUWQxojwx1xaAjUxhkzrBvaYExRUuVz3jnEA1C4asLgGeTBlDTOnt67R_uJJzTJWGZ6hcnj4048qvo8jVeCHoBHzAG2Q47DtGnYXSqZSs53YHdtUBLLWmAYoDgTHd5IKrjO2rt2GWLFQP6WX-kilWHiLXKMxHb6FX9MlkyTvB2kwYluG8wR2TNglejVgMok5kNKa8zdYoEofYZfG83SXS4fEUOD0p2kyXKG-njAo4olI2OMRnGAQCo.AiuNrt1-6194gw-HIOIPPzb_afuE-TZCKSklTSOQh8g&dib_tag=se&keywords=brushless+dc+motor+sensored&qid=1711905125&sprefix=brushless+dc+motor+sensored%2Caps%2C154&sr=8-31
//claims that motor has 170 Kv, phase to phase resistance is around 50 mOhms, phase to phase inductance is around 10 mH
double SpeedConst = 170.0; //this is speed constant of the motor, for Flipsky 7070 E-Board Motor the Kv is 110 RPM/V
double PolePairs = 7.0;           //Machine constant, SX1 motor has 15 pole pairs, used to calculate actual motor RPM
unsigned long PhPhR = 0.05;  //phase to phase resistance of motor
int MaxPhCurr = 20;           //max phase current of motor, 20 amp (peak) phase current with phase resistance of 0.072 Ohms results in 1.44 V + phase voltage is max, PWM needs to be adjusted for speed, need KV as well or use current sensors instead
double BackEMFV = 0.0;
double KpD = 0.0;
double KiD = 0.0;
double KpQ = 0.0; 
double KiQ = 0.0; 
int Offset_Angle = 232; //Angle used to align the stator with phase A high = 0 degrees and rotor with transition between Hall state 1 and hall state 5 being zero degrees (with 5 -> 1 being the positive direction)


/*
//Shengyi SX1 motor with Fast wind from Grin Technologies, stated at 9.8 RPM/V (at the output, has internal gear train), with a 4.777777:1 mechanical gear ratio, means motor is 46.82 RPM/V
double SpeedConst = 46.82; //this is speed constant of the motor, for Flipsky 7070 E-Board Motor the Kv is 110 RPM/V
double PolePairs = 15;           //Machine constant, SX1 motor has 15 pole pairs, used to calculate actual motor RPM
unsigned long PhPhR = 0.072;  //phase to phase resistance of motor
int MaxPhCurr = 20;           //max phase current of motor, 20 amp (peak) phase current with phase resistance of 0.072 Ohms results in 1.44 V + phase voltage is max, PWM needs to be adjusted for speed, need KV as well or use current sensors instead
double BackEMFV = 0.0;
double KpD = 0.13; //0.65; //0.22; //0.18; //0.26;  //0.13; //0.0000000003125;   //2.8; //0.26 value is from Phaserunner V5 (ebikes.ca controller), 0.18 is calculated value from Thesis paper
double KiD = 0.00000000000007;  //0.000000000069; //0.0000000000625; //0.00000000000007; //0.000008;   //0.000000002; //297.81 value is from Phaserunner V5 (ebikes.ca controller), 480.0 is calculated value from Thesis paper
double KpQ = 0.13; //0.65; //0.22; //0.18; //0.26;  //0.13; //0.0000000003125;    //2.8; //0.26 value is from Phaserunner V5 (ebikes.ca controller), 0.18 is calculated value from Thesis paper
double KiQ = 0.00000000000007;  //0.000000000069; //0.0000000000625; //0.00000000000007; //0.000008;   //0.000000002; //297.81 value is from Phaserunner V5 (ebikes.ca controller), 480.0 is calculated value from Thesis paper
int Offset_Angle = 125; //Angle used to align the stator with phase A high = 0 degrees and rotor with transition between Hall state 1 and hall state 5 being zero degrees (with 5 -> 1 being the positive direction)
*/




float AConv = 0.00322265625;  //Volt per division for 10 bit and 3.3 V
float DCVConv  = 20.4842;        //Conversion factor between DC voltage and Teensy voltage (172 + 10 kOhm / 10 kOhm)
float PhAVConv = 18.2;        //Conversion factor between phase voltage and Teensy voltage (172 + 10 kOhm / 10 kOhm)
float PhBVConv = 18.2;        //Conversion factor between phase voltage and Teensy voltage (172 + 10 kOhm / 10 kOhm)
float PhCVConv = 18.2;        //Conversion factor between phase voltage and Teensy voltage (172 + 10 kOhm / 10 kOhm)
float NeuVConv = 1.0;         //conversion factor between neutral voltage and teensy voltage
float PhAIConv = 0.05371;         //conversion factor between phase A current and teensy voltage
float PhBIConv = 0.05371;         //conversion factor between phase B current and teensy voltage
float PhCIConv = 0.05371;         //conversion factor between phase C current and teensy voltage
float DCIConv  = 1.0;         //conversion factor between DC current and teensy voltage

bool print = true;            //boolean variable that switches print mode on and off (easier than commenting out stuff you don't want to see)
int PWM = 0;                  //making variable for the PWM value, needs to be variable in future based on current POT position, upper limit is 1007, (1008 produces ~1/2 output value) cannot do full range of 0 - 1023
int Dir = 1;                  //direction variable, direction is calculated based on previous and current hall states, intent is that forward is positive Dir
int Stator_Pos = 0;           //variable intended to provide position of Stator magnetic field, use increment and decrement with direction variable to rotate

unsigned long HallMicros = 0;     //variable to store time data for Hall change in microseconds
unsigned long LastHallMicros = 0; //variable to store time data for last Hall change in microseconds
float MicrosTime = 0.0;           //variable to store time since last hall change
                                  //micros overflow limit for a Teensy uses a 32 bit integer (unsigned long) will reset after 1 hour, 11 minutes, 34 seconds (same as arduino at around 70 minutes)
int CurrRPMEst = 0;               //variable to store RPM estimation from hall state switching
  //since black unbranded hobby motor has 7 rotor pole pairs and halls create 6 steps per electrical revolution, then 42 hall steps for a full mechanical rotation
  //1 ms per step would be 42 ms/revolution, which results in a max of 1,428 RPM if using milliseconds
  //microseconds results in up to 1,428,000 RPM max
  //1,428 RPM with 10:1 gear ratio and 20" diameter wheel would result in a maximum speed of 3.8 m/s (15 mph min avg = 6.7 m/s)
  //SX1 motor has 15 rotor pole pairs and halls create 6 steps per electrical revolution, then 90 steps for full mechanical rotation
  //1 ms per step would be 90 ms/revolution, results in a maximum of 667 RPM if using milliseconds, microseconds results in up to 667,000 RPM
  //gear ratio of SX1 is 4.78:1, resulting in 3.71 m/s maximum speed if using milliseconds (8.3 mph)

bool Sensor = false;  //boolean value for if sensor is connected (specifically halls for motor)
bool PhAHTest = false;  //Boolean test variable for Phase A high FET - Not implemented yet
bool PhBHTest = false;  //Boolean test variable for phase B high FET - Not implemented yet
bool PhCHTest = false;  //Boolean test variable for phase C high FET - Not implemented yet
bool PhALTest = false;  //Boolean test variable for phase A low FET - Not implemented yet
bool PhBLTest = false;  //Boolean test variable for phase B low FET - Not implemented yet
bool PhCLTest = false;  //Boolean test variable for phase C low FET - Not implemented yet
bool PhAGood = false; //Boolean test variable for phase A - Not implemented yet
bool PhBGood = false; //Boolean test variable for phase B - Not implemented yet
bool PhCGood = false; //Boolean test variable for phase C - Not implemented yet

int DCVolt = 0; //reports the maximum DC voltage seen, value reported is analog integer, should be 10 bit (0-1023), uses the phase voltage read
int SysVolt = 0; //system voltage variable, left as integer from analogRead function
int SysVoltH = 0; //system voltage variable, this is +10%, used as upper bound for other calculations
int SysVoltL = 0; //system voltage variable, this is -10%, used as lower bound for other calculations 
int AnalogVal = 0;  //system voltage storage location for comparisons

float PhaseA   = 0.0; //float storing the duty cycle for phase A
float PhaseB   = 0.0; //float storing the duty cycle for phase B
float PhaseC   = 0.0; //float storing the duty cycle for phase C
int   DcA      = 0;   //duty cycle integer for Phase A
int   DcB      = 0;   //duty cycle integer for Phase B
int   DcC      = 0;   //duty cycle integer for Phase C

int Halls[3] = {0, 0, 0}; //setting up hall state array, the interrupts will call function to update these
  //if hall state 001 (1) is 0 degrees, then assume state 1 is 0 - 59 electrical degrees
  //hall state 101 (5) is 60 to 119 electrical degrees
  //hall state 100 (4) is 120 to 179 electrical degrees
  //hall state 110 (6) is 180 to 239 electrical degrees
  //hall state 010 (2) is 240 to 299 electrical degrees
  //hall state 011 (3) is 300 to 359 electrical degrees
  //this means if hall C is high, then motor is between 300 - 119 electrical degrees
  //this means if hall B is high, then motor is between 180 - 359 electrical degrees
  //this means if hall A is high, then motor is between 60 - 239 electrical degrees

int Angle = 0;          //This is angle used for the stator field, integer limits from 0 to 359 degrees, all other angles are illegal
int CurrPos = 0;        //Current position of rotor estimated from last hall information
int MaxPos = 0;         //placing upper limit on the rotor position for rotation
int MidPos = 0;         //placing middle position of current hall state, will be used for cold starts
int MinPos = 0;         //last position mesured in degrees from 0 angle (line between hall state 1 and 3)
int CurrHallState = 0;  //This is the current state of the hall sensors, gives rough/potential position
int PrevHallState = 0;  //This is previous hall state, used to calculate velocity and acceleration
int tempHall = 0;       //temporary hall storage, used to determine if the time and position needs to be updated

unsigned long CurrTime = 0;       //this is current time, used to estimate current position of rotor with velocity and acceleration from before
unsigned long CurrStateTime = 0;  //This is interrupt recorded time for when it entered the current hall state
unsigned long PrevStateTime = 0;  //This is interrupt recorded time for when motor entered the previous hall state
unsigned long Curr_Time_Diff = 0; //this is time difference stored for current velocity calculation, is difference between CurrTIme and CurrStateTime 
unsigned long Time_Diff = 0;      //this is time difference stored for velocity calculation
unsigned long Prev_Time_Diff = 0; //This is time difference stored for acceleration calculation

float CurrentVelocity = 0.0;    //this is estimated current velocity measured in electrical degrees per microsecond
float CurrentStateVel = 0.0;    //this is the average velocity between the previous state and entering the current state measured in electrical degrees per microsecond 
float PrevStateVel = 0.0;       //this is the average velocity between the prev state and entering the current state
float PrevAcceleration = 0.0;   //this is the average acceleration between the previous state velocity and the current state velocity

int Potentiometer = 0;  //place to store the potentiometer value

int Advance_Angle = 77; //angle offset in degrees to provide maximum torque (Maximum Torque Per Ampere control typically at 90 degrees for BLDC)
int THI_Mag = 0;        //Third Harmonic Injection Magnitude variable, initializes to zero

int i = 0;

double PhaseAI = 0.0;
double PhaseBI = 0.0;
double PhaseCI = 0.0;

double PhaseAV = 0.0;
double PhaseBV = 0.0;
double PhaseCV = 0.0;

double PhACurr = 0.0;
double PhBCurr = 0.0;

double AlphaCurr = 0.0;
double BetaCurr = 0.0;

double DaxisCurr = 0.0;
double QaxisCurr = 0.0;
double SinAngle = 0.0;
double CosAngle = 0.0;

double DaxisCurrError = 0.0;
double QaxisCurrError = 0.0;
double RequestedCurr = 0.0;
double TotalDaxisCurrError = 0.0;
double TotalQaxisCurrError = 0.0;

double DaxisVolt = 0.0;
double QaxisVolt = 0.0;

double AlphaVolt = 0.0;
double BetaVolt = 0.0;

double PhAVOut = 0.0;
double PhBVOut = 0.0;
double PhCVOut = 0.0;

double DCVoltage = 0.0;

int CurrOffset = 0;
int VoltOffset = 0;

double SampleTimeChange = 0.0;  //this is actual time, will be 0.000xxx seconds
unsigned long CurrSampleTime = 0;
unsigned long LastSampleTime = 0;

double CurrentFilterTimeConstant = 0.001; //setting filter to 250 us, around the minimum time for a hall state change //0.025
double AlphaIIRFilterVal = 0.0;
double OneMinusAlphaFilterVal = 0.0;

double PhaseAPrevCurrFilter = 0.0;
double PhaseBPrevCurrFilter = 0.0;
double PhaseCPrevCurrFilter = 0.0;

double PhaseAFilteredCurr = 0.0;
double PhaseBFilteredCurr = 0.0;
double PhaseCFilteredCurr = 0.0;

unsigned long CurrentTimePos = 0;
unsigned long PrevTimePos = 0;
double PosTimeDiff = 0.0;

double FilterTimePos = 0.00005;  //maximum speed results in around 4 us per electrical degree, not sure if this value is correct
double IIRPosFilterAlpha = 0.0;
double IIRPosFilterOneMinusAlpha = 0.0;

double FilteredPos = 0.0;
double LastFilteredPos = 0.0;

bool OverCurrent = false;

int AnalogCurrReq = 0;
bool Activated = false;

bool LowVoltage = false;

double VoltSampleTime = 0.0;
double SampleTimeChangeVolt = 0.0;
double LastVoltSampleTime = 0.0;
double AlphaIIRFilterValVolt = 0.0;
double VoltageFilterTimeConstant = 0.0001;
double OneMinusAlphaFilterValVolt = 0.0;

double PhaseAFilteredVolt = 0.0;
double PhaseBFilteredVolt = 0.0;
double PhaseCFilteredVolt = 0.0;

double PhaseAPrevVoltFilter = 0.0;
double PhaseBPrevVoltFilter = 0.0;
double PhaseCPrevVoltFilter = 0.0;

double AlphaVoltRead = 0.0;
double BetaVoltRead = 0.0;

double SensorlessAngle = 0.0;
double BEMFVoltMag = 0.0;

long NowSpeedTime = 0;
long LastSpeedTime = 0;
long SpeedTimeDiff = 0;
double SpeedTimeDiffSecs = 0.0;
double SpeedTimeDiffMins = 0.0;
double SpeedTimeDiffHour = 0.0;
double WheelSpeedMPH = 0.0;
unsigned long TempTimeSpeed = 0;

double OutputAngle = 0.0;

double DutyCycleA = 0.0;
double DutyCycleB = 0.0;
double DutyCycleC = 0.0;

double LastSensorlessAngle = 0.0;
double AngleChange = 0.0;
double AngularSpeed = 0.0;

double AngleDriven = 0.0;
int AngleSensorless = 0; //SensorlessAngle;
int AngleSensored = 0; //AngleCorrection(PositionFilter() + Offset_Angle)
int AngleError = 0;

int j = 0;

unsigned long TimeA = 0;
unsigned long TimeB = 0;
unsigned long TimeDiffAB = 0;
double MaxPhVolt = 0.0;

int DCCurr = 0.0;




void setup() 
{
  //Making sure that all FETs are off while starting 
  MotStateOff();

  Serial.begin(115200);     //initializes serial monitor to 115200 baud
  //Serial.println("...Initializing...");
  pinMode(LED0, OUTPUT);     //initializes the on-board led to be an ouptut
  digitalWrite(LED0, HIGH);  //sets the LED digital pin to be high/on

  pinMode(HallA, INPUT_PULLUP); //puts the hall states to be input with an internal pullup
  pinMode(HallB, INPUT_PULLUP); //puts the hall states to be input with an internal pullup
  pinMode(HallC, INPUT_PULLUP); //puts the hall states to be input with an internal pullup
  pinMode(Speedometer, INPUT_PULLUP); //puts the hall states to be input with an internal pullup


  pinMode(PWMA,   OUTPUT);  //initializes motor phase shutdown and phase control connections to be outputs
  pinMode(SDPhA,  OUTPUT);  //initializes motor phase shutdown and phase control connections to be outputs
  pinMode(PWMB,   OUTPUT);  //initializes motor phase shutdown and phase control connections to be outputs
  pinMode(SDPhB,  OUTPUT);  //initializes motor phase shutdown and phase control connections to be outputs
  pinMode(PWMC,   OUTPUT);  //initializes motor phase shutdown and phase control connections to be outputs
  pinMode(SDPhC,  OUTPUT);  //initializes motor phase shutdown and phase control connections to be outputs

  //V5 controller uses pins 5, 7, 9 for phase PWM, this is FlexPWM2.1 for pin 5, FlexPWM1.3 for pin 7, and FlexPWM2.2 for pin 9
  //all of the above values are the same as for Teensy 4.0, which was used on V4 controller
  //FLEXPWM2_SM0CTRL2 |= FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_FORCE; // pin 4, initializes teensy PWM timers to be center aligned rather than left aligned
  FLEXPWM2_SM1CTRL2 |= FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_FORCE; // pin 5, initializes teensy PWM timers to be center aligned rather than left aligned
  FLEXPWM2_SM2CTRL2 |= FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_FORCE; // pin 6,9, initializes teensy PWM timers to be center aligned rather than left aligned
  FLEXPWM1_SM3CTRL2 |= FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_FORCE; // pin 8, initializes teensy PWM timers to be center aligned rather than left aligned

  analogWriteFrequency(5, f); //sets the PWM pins to be switching at a frequency of 20 kHz
  analogWriteFrequency(6, f); //sets the PWM pins to be switching at a frequency of 20 kHz
  analogWriteFrequency(9, f); //sets the PWM pins to be switching at a frequency of 20 kHz
  //above section should only use analog write for the PWM pins (PWMA = 5, PWMB = 6, PWMC = 9)

  analogWriteResolution(10);
  analogReadAveraging(3); //teensy 4.1 traditionally uses the average of 4x analog readings to calculate the output, switching to single reading for time reduction from 17 us to 5 us with averaging of 1 to 3

  MotStateOff();  //Making sure that all FETs are off while starting 
  
  delay(10);

  ReadHalls();
  //Serial.print("Hall A: ");
  //Serial.println(Halls[0]);
  //Serial.print("Hall B: ");
  //Serial.println(Halls[1]);
  //Serial.print("Hall C: ");
  //Serial.println(Halls[2]);

  //Serial.print("Initialization Hall state is: ");
  //Serial.println(CurrHallState);

  if(CurrHallState >= 1 && CurrHallState <= 6)
  {
    Sensor = true;
    CurrStateTime = micros(); //function stores current micros time as the Current State Time for position and velocity sensing
    //Serial.print("Init Hall state is: ");
    //Serial.println(CurrHallState);
  }
  else
  {
    //illegal states need to put system into sensorless mode (until sensorless mode is created, would need to be disabled)
    //if the halls are equal to 0 or 7, then illegal state, or no sensor attached
    //State 7 is with no sensor attached due to pullup
    Sensor = false;
    if(print == true)
    {
      //Serial.println("No Hall Sensor attached, please check wiring");
    }
  }

  TestDCV();  //function cycles through phases, placing each at a PWM value and verifies that all FETs are working correctly

/*
  while(TestMotorPos() == false); //function TestMotorPos will return true if all phases aligned correctly, otherwise will send LED SOS repeatedly - may make "first time startup" function and store value to EEPROM or something
  {
    LEDSOS();
  }
*/

  BackEMFV = 0.0000103923 * PolePairs * SpeedConst; //the constant is 0.000006 * SQRT(3), the SQRT(3) is to convert the DC Voltage into the q axis voltage

  //setup complete, blink LED 3x 50% duty cycle
  digitalWrite(LED0, LOW);
  delay(100);
  digitalWrite(LED0, HIGH);
  delay(100);
  digitalWrite(LED0, LOW);
  delay(100);
  digitalWrite(LED0, HIGH);
  delay(100);
  digitalWrite(LED0, LOW);
  delay(100);
  digitalWrite(LED0, HIGH);

  delay(500);

  //attachInterrupt(digitalPinToInterrupt(HallA), ReadHalls, CHANGE); //sets the halls to be an interrupt, measures any change, calls switching function every time hall changes
  //attachInterrupt(digitalPinToInterrupt(HallB), ReadHalls, CHANGE); //sets the halls to be an interrupt, measures any change, calls switching function every time hall changes
  //attachInterrupt(digitalPinToInterrupt(HallC), ReadHalls, CHANGE); //sets the halls to be an interrupt, measures any change, calls switching function every time hall changes

  //attachInterrupt(digitalPinToInterrupt(Speedometer), WheelSpeed, RISING); //sets the Speedometer hall to be an interrupt, measures any change, calls switching function every time hall changes

} //end of setup section

void loop()
{ 

/*
  ReadHalls();

  Serial.print("Hall A: ");
  Serial.print(Halls[0]);
  Serial.print(", Hall B: ");
  Serial.print(Halls[1]);
  Serial.print(", Hall C: ");
  Serial.print(Halls[2]);

  Serial.print(", Hall state is: ");
  Serial.println(CurrHallState);
  delay(10);
*/



  //Serial.print("Begin time: ");
  //Serial.print(micros());
  //Serial.println(" microseconds");

  FOC();  //as of 25 March 2024 12:06 PM, uses 155 us of time to calculate, need to improve, by changing the analog read averaging from 4 to 3, the time dropped to 47 us per FOC call


  Serial.print(KpD, 4);
  Serial.print(", ");  

  //Serial.print(OverCurrent);
  //Serial.print(", ");  
  //Serial.print(RequestedCurr);
  //Serial.println(BEMFVoltMag);
  //Serial.print(", ");
  Serial.print(DaxisCurr);
  Serial.print(", ");
  Serial.println(QaxisCurr);
  //Serial.print(", ");
  //Serial.print(DaxisVolt);
  //Serial.print(", ");
  //Serial.println(QaxisVolt);







/*
  Potentiometer = analogRead(pot);

  digitalWrite(SDPhA, HIGH);
  digitalWrite(SDPhB, HIGH);
  digitalWrite(SDPhC, HIGH);

  analogWrite(PWMA, Potentiometer);
  analogWrite(PWMB, Potentiometer); //analog write maximum should be 1023
  analogWrite(PWMC, Potentiometer);

  //Serial.println("DC Voltage Read");
  //Serial.print("Phase A voltage read bits: ");
  Serial.print(analogRead(PhAV)); //Phase A analog Voltage read
  //Serial.print(", Phase B voltage read bits: ");
  Serial.print(", ");
  Serial.print(analogRead(PhBV)); //Phase B analog Voltage read
  //Serial.print(", Phase C voltage read bits: ");
  Serial.print(", ");
  Serial.println(analogRead(PhCV)); //Phase C analog Voltage read

  //delay(1);
*/

/*
  Serial.print(KiD, 12);
  Serial.print(", ");  
*/

/*
  //Serial.print(OverCurrent);
  //Serial.print(", ");  
  Serial.print(RequestedCurr);
  //Serial.println(BEMFVoltMag);
  Serial.print(", ");
  Serial.print(DaxisCurr);
  Serial.print(", ");
  Serial.println(QaxisCurr);
  //Serial.print(", ");
  //Serial.print(DaxisVolt);
  //Serial.print(", ");
  //Serial.println(QaxisVolt);
*/

/*
  AngleDriven = AngleCorrection(57.295779 * atan2(BetaVolt, AlphaVolt));
  AngleSensorless = SensorlessAngle;
  AngleSensored = AngleCorrection(Rotor_Curr_Pos() + Offset_Angle); //PositionFilter()
  AngleError = AngleSensorless - AngleSensored;
  //get error from sensorless angle to the hall positions, align both
  //Serial.print(AngleDriven);
  //Serial.print(", ");
  //Serial.print(360);  //360 and -360 are to set limits of max and min on serial plotter
  //Serial.print(", ");
  //Serial.print(-360);
  //Serial.print(", ");
  //Serial.print(AngleSensorless);
  //Serial.print(", ");
  //Serial.print(AngleSensored);
  //Serial.print(", ");
  Serial.println(AngleError);
*/

//timer module, move time A to before whatever you want to measure and TimeB to after, will spit out time in microseconds
/*
  TimeA = micros();
  TimeB = micros();
  TimeDiffAB = TimeB - TimeA;

  Serial.print(", ");
  Serial.println(TimeDiffAB);
*/

/*
  Serial.print("Phase Duty Cycles: ");
  Serial.print(DcA);
  Serial.print(", ");
  Serial.print(DcB);
  Serial.print(", ");
  Serial.print(DcC);
//  Serial.print(", Phase Voltage Set: ");
//  Serial.print(PhAVOut);
//  Serial.print(", ");
//  Serial.print(PhBVOut);
//  Serial.print(", ");
//  Serial.print(PhCVOut);
  //Serial.print(", Requested Current: ");
  //Serial.println(RequestedCurr);
//  Serial.print(", DC Voltage: ");
//  Serial.print(DCVoltage);
//  Serial.print(", Alpha and Beta Voltages: ");
//  Serial.print(AlphaVolt);
//  Serial.print(", ");
//  Serial.print(BetaVolt);
//  Serial.print(", D and Q Axis Voltages: ");
//  Serial.print(DaxisVolt);
//  Serial.print(", ");
//  Serial.println(QaxisVolt);

  Serial.print(", Offset Angle: ");
  Serial.print(Offset_Angle);
//  Serial.print(SinAngle);
//  Serial.print(", ");
//  Serial.print(CosAngle);
  Serial.print(", Phase Currents: ");
//  Serial.print(PhaseAI);
//  Serial.print(", ");
//  Serial.print(PhaseBI);
//  Serial.print(", ");
//  Serial.print(PhaseCI);
//  Serial.print(", Filtered Phase Currents: ");
  //Serial.print(AlphaCurr);
  //Serial.print(", ");
  //Serial.print(BetaCurr);
  //Serial.print(", ");
  Serial.print(DaxisCurr);
  Serial.print(", ");
  Serial.println(QaxisCurr);
*/
  //delay(1);

//  Serial.print(PhaseAI);
//  Serial.print(", ");
//  Serial.print(PhaseBI);
//  Serial.print(", ");
//  Serial.print(PhaseCI);
//  Serial.print(", ");
//  Serial.print(PhaseAFilteredCurr);
//  Serial.print(", ");
//  Serial.print(PhaseBFilteredCurr);
//  Serial.print(", ");
//  Serial.println(PhaseCFilteredCurr);

  //Serial.print(TotalDaxisCurrError);
  //Serial.print(", ");
  //Serial.println(TotalQaxisCurrError);

  //Serial.print("Variable1 ");
  //Serial.print(DaxisCurr);
  //Serial.print(", ");
  //Serial.print("Variable2 ");
  //Serial.print(QaxisCurr);
  //Serial.print(SampleTimeChangeVolt);

  //Serial.print(", ");
  //Serial.println(analogRead(MotorTemp));

//  Serial.print(DcA);
//  Serial.print(", ");
//  Serial.print(DcB);
//  Serial.print(", ");
//  Serial.print(DcC);
//  Serial.print(", ");
//  Serial.print(DaxisVolt);
//  Serial.print(", ");
//  Serial.println(QaxisVolt);
//  Serial.print(", ");
//  Serial.print(atan2(BetaVolt, AlphaVolt));
  //Serial.print(", ");
  //Serial.println(THI_Mag);

//  Serial.print(PhaseAFilteredVolt);
//  Serial.print(", ");
//  Serial.print(PhaseBFilteredVolt);
//  Serial.print(", ");
//  Serial.print(PhaseCFilteredVolt);
//  Serial.print(", ");
//  Serial.print(SensorlessAngle);
//  Serial.print(", ");

//  Serial.print(AlphaVolt);
//  Serial.print(", ");
//  Serial.print(BetaVolt);

  //Serial.println(AngularSpeed);

//  Serial.println(BEMFVoltMag);

  //Serial.print(", ");
  //Serial.print("Variable 3: ");
  //Serial.println(OverCurrent);
//  Serial.print(", ");
//  Serial.print("value 3 ");
//  Serial.println(PhaseCI);
  //Serial.print(", ");
  //Serial.print("value 4 ");
  //Serial.println(CurrOffset);

  //Serial.print("Current Rotor Position estimation: ");
  //Serial.print(Rotor_Curr_Pos());  //Uses last known position, velocity, and acceleration to calculate the position (electrical degrees) of the rotor
  //Serial.println(" electrical degrees");

  //Serial.print("Current Hall State is: ");
  //Serial.print("Variable_1: ");
  //Serial.println(CurrHallState);

  //Serial.print("End time: ");
  //Serial.print(micros());
  //Serial.println(" microseconds");

  //Serial.print("Current Estimated motor speed: ");
  //Serial.print(CurrentVelocity);
  //Serial.println(" electrical degrees per microsecond");

  //Serial.print("Motor direction is: ");
  //Serial.println(Dir);
  //Serial.println(" with 1 being forward and -1 being backwards");

  //Serial.print("Current Estimated motor speed: ");
  //Serial.print(CurrRPMEst);
  //Serial.println(" RPM");

  //Serial.print("Average velocity between the last 2 hall states: ");
  //Serial.print(CurrentStateVel);
  //Serial.println(" degrees per microsecond");

  //Serial.print("Average acceleration between the last 2 hall states: ");
  //Serial.print(PrevAcceleration);
  //Serial.println(" degrees per microsecond squared");

  //delay(50);

/*
  i++;
  if(i == 2000)
  {
    Offset_Angle++;
    i = 0;
    if(Offset_Angle >= 360)
    {
      Offset_Angle = 0;
    }
  }
*/

  //Advance_Angle++;

  //if(Advance_Angle >= 360)
  //{
  //  Advance_Angle = 0;
  //}
  //reverse direction, angle is wrong

  //Serial.println(Advance_Angle);
  //MotorPos();

  //delay(250);

  //Serial.print(", ");
  //Serial.print("Variable_2: ");
  //Serial.print(CurrRPMEst); //Phase A analog Voltage read

  //Serial.print(" Rotor Est Angle is: ");
  //Serial.print(Rotor_Curr_Pos());
  //Serial.print(" Angle Difference: ");
  //Serial.println(Angle - Rotor_Curr_Pos());

  //Serial.print("Potentiometer and PWM value is: ");
  //Serial.print("Variable_1:");
  //Serial.print(Potentiometer);

/*
  //checking what DC voltage is 
  digitalWrite(SDPhA, HIGH);
  digitalWrite(SDPhB, HIGH);
  digitalWrite(SDPhC, HIGH);

  analogWrite(PWMA, 504 + PWM);
  analogWrite(PWMB, 504 - PWM);
  analogWrite(PWMC, 504 - PWM);
*/

  //Angle = map(analogRead(pot), 0, 1023, 0, 359);
  //Serial.print("Potentiometer Angle Read is: ");
  //Serial.println(Angle);
  //Serial.println(" Degrees");

  //Serial.print(", Phase A voltage read bits: ");
  //Serial.print(", ");
  //Serial.print(", Variable_2: ");
  //Serial.print(analogRead(DcA)); //Phase A analog Voltage read
  //Serial.print(", Phase B voltage read bits: ");
  //Serial.print(", ");
  //Serial.print(", Variable_3: ");
  //Serial.print(analogRead(DcB)); //Phase B analog Voltage read
  //Serial.print(", Phase C voltage read bits: ");
  //Serial.print(", ");
  //Serial.print(", Variable_4: ");
  //Serial.println(analogRead(DcC)); //Phase C analog Voltage read
  //Serial.print(", Neutral voltage read bits: ");
  //Serial.print(",");
  //Serial.print("Variable_5:");
  //Serial.println(analogRead(NeuV)); //Neutral analog Voltage read

  //Serial.print("Phase A current read bits: ");
  //Serial.println(analogRead(PhAI)); //Phase A analog current read
  //Serial.print("Phase C current read bits: ");
  //Serial.println(analogRead(PhCI)); //Phase C analog current read


  //Serial.print("Hall State is: ");
  //Serial.println(ReadHalls());
  //delay(10);

  //Serial.println("Phase A PWM Voltage Read");
  //Serial.print("Phase A voltage read bits: ");
  //Serial.println(analogRead(PhAV)); //Phase A analog Voltage read
  //Serial.print("Phase B voltage read bits: ");
  //Serial.println(analogRead(PhBV)); //Phase B analog Voltage read
  //Serial.print("Phase C voltage read bits: ");
  //Serial.println(analogRead(PhCV)); //Phase C analog Voltage read
  //Serial.print("Neutral voltage read bits: ");
  //Serial.println(analogRead(NeuV)); //Neutral analog Voltage read
} //end of loop section











void ReadPhaseVoltages()  //Reading the voltages of all three phases, as of 25 March 2024 12:08 PM, this function takes 50 us to calculate
{
  //May also be able to use these voltages for back-emf, Third Harmonic Injection, and high speed rotor position estimation, can also be used to determine total power in motor
  //uses same as ReadCurrent function where analog values are read, 

  PhaseAV = analogRead(PhAV);   //each analog read takes approximately 17 us, this is majority of the time for FOC, changing averaging from 4 samples to 3 reduces this to 5 us
  PhaseBV = analogRead(PhBV);
  PhaseCV = analogRead(PhCV);


  //Serial.print(PhaseAV);
  //Serial.print(", ");
  //Serial.print(PhaseBV);  //something is wrong with phase BV, I think capacitor C31 is a wrong value or not fully installed
  //Serial.print(", ");
  //Serial.println(PhaseCV);
  //Serial.print(", ");

  //averaged to remove noise/DC offset, 
  VoltOffset = (PhaseAV + PhaseBV + PhaseCV) / 3; //averaging the three phases results in an offset

  //each voltage has average subtracted to result in +/- from 0
  PhaseAV -= VoltOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)
  PhaseBV -= VoltOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)
  PhaseCV -= VoltOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)

  //then multiplied by conversion factor to get actual analog value in Volts
  PhaseAV *= PhAVConv;  //converts digital phase voltage values into accurate voltage values in Volts
  PhaseBV *= PhBVConv;  //converts digital phase voltage values into accurate voltage values in Volts
  PhaseCV *= PhCVConv;  //converts digital phase voltage values into accurate voltage values in Volts
}

void VoltageFilter()
{
  //intended to be low pass filter for the current measurements
  //using IIR filter with discrete form of Vf(k) = [Tf * Vf(k-1)/(Tf+Ts)] + [Ts * V(k) / (Tf + Ts)]
  //This is equivalent to Vf(k) = a Vf(k-1) + (1-a) V(k) with a = Tf / (Tf + Ts) 
  //in the above equations, Tf is the time constant of the low pass filter (actual time), Ts is the actual measured time between measurements, Vf is the filtered value, V is the actual (input) value
  //when a time in between samples is much less than the time constant, the current value is added as a very small portion of the filtered output, with most being the previous filtered output
  VoltSampleTime = micros();
  SampleTimeChangeVolt = VoltSampleTime - LastVoltSampleTime;
  SampleTimeChangeVolt *= 0.000001; //conversion from microseconds to seconds
  LastVoltSampleTime = VoltSampleTime;

  AlphaIIRFilterValVolt = VoltageFilterTimeConstant / (VoltageFilterTimeConstant + SampleTimeChangeVolt); //can multiply by 1/time constant to get the frequency multiplied by the time change
  OneMinusAlphaFilterValVolt = 1.0 - AlphaIIRFilterValVolt;

  PhaseAFilteredVolt = (AlphaIIRFilterValVolt * PhaseAPrevVoltFilter) + (OneMinusAlphaFilterValVolt * PhaseAV);
  PhaseBFilteredVolt = (AlphaIIRFilterValVolt * PhaseBPrevVoltFilter) + (OneMinusAlphaFilterValVolt * PhaseBV);
  PhaseCFilteredVolt = (AlphaIIRFilterValVolt * PhaseCPrevVoltFilter) + (OneMinusAlphaFilterValVolt * PhaseCV);

  PhaseAPrevVoltFilter = PhaseAFilteredVolt;
  PhaseBPrevVoltFilter = PhaseBFilteredVolt;
  PhaseCPrevVoltFilter = PhaseCFilteredVolt;
}

void ClarkeReadVolt() //clarke transform, takes in phase A and Phase B voltage measurements and transforms into alpha beta voltages
{
  AlphaVoltRead = ((2.0 * PhaseAFilteredVolt) - PhaseBFilteredVolt - PhaseCFilteredVolt) / 3.0; //full transform has alpha = ((2 * A) - B - C) / 3
  BetaVoltRead = 0.57735026918962 * (PhaseBFilteredVolt - PhaseCFilteredVolt);  //full transform has beta = (0 * A) + (B/SQRT(3)) - (C/SQRT(3))
  //true Clarke transform also has third factor which is 1/3 for all phase inputs, should be equal to 0 at all times (not equal to 0 means current leaving through wye)
}

//with filtered phase voltages and currents, want to calculate a per phase impedance, this impedance should be constant (for specific requested current)
//ideally this impedance is only a resistance
//this will not do much at low speeds since the back emf will be low, but at higher speeds the back emf will dominate and provide a sinusoidal back emf on the q axis, driving the current to be in line due to the constant impedance
//for sensorless control, know that all back emf voltage is in q axis (changing flux induces voltage) d axis will be equal to 0
//need to use the alpha and beta voltages to determine what the d and q axis voltage magnitude and angle is
//to find the angle, need to use inverse tangent of (BetaVoltRead / AlphaVoltRead), this will result in the q axis angle from the a phase, subtract 90 degrees to get the d axis rotor angle
//Since arduino atan2 function returns a result from -Pi to Pi, this is full rotation (atan is only -Pi/2 to Pi/2)
//can get amplitude of back emf from sqrt(AlphaVoltRead^2 + BetaVoltRead^2), if this value is greater than some voltage, then we can switch from halls to sensorless

void SensorlessRotorPosition()  //determines position of rotor based on back emf voltage reads
{
  SensorlessAngle = 57.295779513 * atan2(BetaVoltRead, AlphaVoltRead); //should determine position of q axis from phase A in radians
  SensorlessAngle = AngleCorrection(SensorlessAngle); //subtract 90 degrees again if this does not work //reduces angle by 90 degrees so that q axis is turned into +d axis, actual position of rotor
}

void VoltageVectorReadMagnitude() //converts alpha and beta voltages into a single voltage vector for magnitude
{
  BEMFVoltMag = sqrt(sq(AlphaVoltRead) + sq(BetaVoltRead));
}

void ReadCurrent()  //Reading the current of all three phases
{
  PhaseAI = analogRead(PhAI);
  PhaseBI = analogRead(PhBI);
  PhaseCI = analogRead(PhCI);
  DCCurr = analogRead(DCI);

/*
  Serial.print(DCCurr);
  Serial.print(", ");
  Serial.print(PhaseAI);
  Serial.print(", ");
  Serial.print(PhaseBI);
  Serial.print(", ");
  Serial.println(PhaseCI);
*/

  //PhaseAV = analogRead(PhAV);
  //PhaseBV = analogRead(PhBV);
  //PhaseCV = analogRead(PhCV);

  if(PhaseAI == 1023)  //unsure how to do this if common mode noise is issue, can get boolean for if current is over measurable values
  {
    //phase current over sense limit of ADC
    OverCurrent = true;
    TotalDaxisCurrError = 0;
    TotalQaxisCurrError = 0;
  }
  else if(PhaseAI == 0)
  {
    //phase current over sense limit of ADC
    OverCurrent = true;
    TotalDaxisCurrError = 0;
    TotalQaxisCurrError = 0;
  }
  else if(PhaseBI == 1023)
  {
    //phase current over sense limit of ADC
    OverCurrent = true;
    TotalDaxisCurrError = 0;
    TotalQaxisCurrError = 0;
  }
  else if(PhaseBI == 0)
  {
    //phase current over sense limit of ADC
    OverCurrent = true;
    TotalDaxisCurrError = 0;
    TotalQaxisCurrError = 0;
  }
  else if(PhaseCI == 1023)
  {
    //phase current over sense limit of ADC
    OverCurrent = true;
    TotalDaxisCurrError = 0;
    TotalQaxisCurrError = 0;
  }
  else if(PhaseCI == 0)
  {
    //phase current over sense limit of ADC
    OverCurrent = true;
    TotalDaxisCurrError = 0;
    TotalQaxisCurrError = 0;
  }
  else
  {
    OverCurrent = false;
  }

  CurrOffset = (PhaseAI + PhaseBI + PhaseCI) / 3; //averaging the three phases results in an offset

  PhaseAI -= CurrOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)
  PhaseBI -= CurrOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)
  PhaseCI -= CurrOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)

/*
  Serial.print(PhaseAI);
  Serial.print(", ");
  Serial.print(PhaseBI);
  Serial.print(", ");
  Serial.println(PhaseCI);
*/

  PhaseAI *= PhAIConv;  //converts digital phase current values into accurate current values in Amps
  PhaseBI *= PhBIConv;  //converts digital phase current values into accurate current values in Amps
  PhaseCI *= PhCIConv;  //converts digital phase current values into accurate current values in Amps

/*
  //averaged to remove noise/DC offset, 
  VoltOffset = (PhaseAV + PhaseBV + PhaseCV) / 3; //averaging the three phases results in an offset

  //each voltage has average subtracted to result in +/- from 0
  PhaseAV -= VoltOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)
  PhaseBV -= VoltOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)
  PhaseCV -= VoltOffset;  //subtracting this offset from each phase results in removal of all common mode noise (DC offset and common mode noise)

  //then multiplied by conversion factor to get actual analog value in Volts
  PhaseAV *= PhAVConv;  //converts digital phase voltage values into accurate voltage values in Volts
  PhaseBV *= PhBVConv;  //converts digital phase voltage values into accurate voltage values in Volts
  PhaseCV *= PhCVConv;  //converts digital phase voltage values into accurate voltage values in Volts
*/

  //this results in Phase currents with all common mode noise removed
}

void CurrentFilter()
{
  //intended to be low pass filter for the current measurements
  //using IIR filter with discrete form of Vf(k) = [Tf * Vf(k-1)/(Tf+Ts)] + [Ts * V(k) / (Tf + Ts)]
  //This is equivalent to Vf(k) = a Vf(k-1) + (1-a) V(k) with a = Tf / (Tf + Ts) 
  //in the above equations, Tf is the time constant of the low pass filter (actual time), Ts is the actual measured time between measurements, Vf is the filtered value, V is the actual (input) value
  //when a time in between samples is much less than the time constant, the current value is added as a very small portion of the filtered output, with most being the previous filtered output
  CurrSampleTime = micros();
  SampleTimeChange = CurrSampleTime - LastSampleTime;
  //Serial.print(SampleTimeChange); //testing to see how long a foc loop takes, 158 us is current, need to reduce (~6.8 kHz) w/ clock at 600 MHz, can do overclock
  SampleTimeChange *= 0.000001; //conversion from microseconds to seconds
  LastSampleTime = CurrSampleTime;

  AlphaIIRFilterVal = CurrentFilterTimeConstant / (CurrentFilterTimeConstant + SampleTimeChange);
  OneMinusAlphaFilterVal = 1.0 - AlphaIIRFilterVal;

  PhaseAFilteredCurr = (AlphaIIRFilterVal * PhaseAPrevCurrFilter) + (OneMinusAlphaFilterVal * PhaseAI);
  PhaseBFilteredCurr = (AlphaIIRFilterVal * PhaseBPrevCurrFilter) + (OneMinusAlphaFilterVal * PhaseBI);
  PhaseCFilteredCurr = (AlphaIIRFilterVal * PhaseCPrevCurrFilter) + (OneMinusAlphaFilterVal * PhaseCI);

  PhaseAPrevCurrFilter = PhaseAFilteredCurr;
  PhaseBPrevCurrFilter = PhaseBFilteredCurr;
  PhaseCPrevCurrFilter = PhaseCFilteredCurr;

/*
  PhaseAFilteredVolt = (AlphaIIRFilterVal * PhaseAPrevVoltFilter) + (OneMinusAlphaFilterVal * PhaseAV);
  PhaseBFilteredVolt = (AlphaIIRFilterVal * PhaseBPrevVoltFilter) + (OneMinusAlphaFilterVal * PhaseBV);
  PhaseCFilteredVolt = (AlphaIIRFilterVal * PhaseCPrevVoltFilter) + (OneMinusAlphaFilterVal * PhaseCV);

  PhaseAPrevVoltFilter = PhaseAFilteredVolt;
  PhaseBPrevVoltFilter = PhaseBFilteredVolt;
  PhaseCPrevVoltFilter = PhaseCFilteredVolt;
*/
}

/*
void CurrentFilterTwo()
{
  //intended to be low pass filter for the current measurements
  //using IIR filter with discrete form of Vf(k) = [Tf * Vf(k-1)/(Tf+Ts)] + [Ts * V(k) / (Tf + Ts)]
  //This is equivalent to Vf(k) = a Vf(k-1) + (1-a) V(k) with a = Tf / (Tf + Ts) 
  //in the above equations, Tf is the time constant of the low pass filter (actual time), Ts is the actual measured time between measurements, Vf is the filtered value, V is the actual (input) value
  //when a time in between samples is much less than the time constant, the current value is added as a very small portion of the filtered output, with most being the previous filtered output
  CurrSampleTime = micros();
  SampleTimeChange = CurrSampleTime - LastSampleTime;
  //Serial.print(SampleTimeChange); //testing to see how long a foc loop takes, 158 us is current, need to reduce (~6.8 kHz) w/ clock at 600 MHz, can do overclock
  SampleTimeChange *= 0.000001; //conversion from microseconds to seconds
  LastSampleTime = CurrSampleTime;

  AlphaIIRFilterVal = CurrentFilterTimeConstant / (CurrentFilterTimeConstant + SampleTimeChange);
  OneMinusAlphaFilterVal = 1.0 - AlphaIIRFilterVal;

  PhaseAFilteredCurr = (AlphaIIRFilterVal * PhaseAPrevCurrFilter) + (OneMinusAlphaFilterVal * PhaseAI);
  PhaseBFilteredCurr = (AlphaIIRFilterVal * PhaseBPrevCurrFilter) + (OneMinusAlphaFilterVal * PhaseBI);
  PhaseCFilteredCurr = (AlphaIIRFilterVal * PhaseCPrevCurrFilter) + (OneMinusAlphaFilterVal * PhaseCI);

  PhaseAPrevCurrFilter = PhaseAFilteredCurr;
  PhaseBPrevCurrFilter = PhaseBFilteredCurr;
  PhaseCPrevCurrFilter = PhaseCFilteredCurr;
}
*/

void Clarke() //clarke transform, takes in phase A and Phase B current measurements and transforms into alpha beta currents
{
/*
  //removing current filter for faster response  
  AlphaCurr = ((2.0 * PhaseAI) - PhaseBI - PhaseCI) / 3.0;  //full transform has alpha = ((2 * A) - B - C) / 3
  BetaCurr = 0.57735026918962 * (PhaseBI - PhaseCI); //full transform has beta = (0 * A) + (B/SQRT(3)) - (C/SQRT(3))
  //true Clarke transform also has third factor which is 1/3 for all phase inputs, should be equal to 0 at all times (not equal to 0 means current leaving through wye) 
*/

  AlphaCurr = ((2.0 * PhaseAFilteredCurr) - PhaseBFilteredCurr - PhaseCFilteredCurr) / 3.0;  //full transform has alpha = ((2 * A) - B - C) / 3
  BetaCurr = 0.57735026918962 * (PhaseBFilteredCurr - PhaseCFilteredCurr); //full transform has beta = (0 * A) + (B/SQRT(3)) - (C/SQRT(3))
  //true Clarke transform also has third factor which is 1/3 for all phase inputs, should be equal to 0 at all times (not equal to 0 means current leaving through wye) 

}

double Radians(int Degrees) //turns integer degrees into double precision radians
{
  double radiantemp = Degrees * 0.01745329252;
  return radiantemp;
}

void Park() //park transform, takes in alpha beta currents and current rotor position to calculate d and q axis currents, q axis aligns with alpha axis
{
  //converting equations to have the q axis aligned with the alpha axis - this means the back emf produces correct angle
  //DaxisCurr = (SinAngle * AlphaCurr) - (CosAngle * BetaCurr);
  //QaxisCurr = (CosAngle * AlphaCurr) + (SinAngle * BetaCurr);  
  //Inverse Park (dq to alpha beta) with q axis aligned to alpha axis is [sin cos; -cos sin]
  //forward Park (alpha beta to dq) with q axis aligned to alpha axis is [sin -cos; cos sin]


  //below is if d axis is aligned with alpha axis
  DaxisCurr = (CosAngle * AlphaCurr) + (SinAngle * BetaCurr);
  QaxisCurr = (-1.0 * SinAngle * AlphaCurr) + (CosAngle * BetaCurr);
  //D axis is Cos * alpha + Sin * Beta
  //Q axis is -Sin * Alpha + Cos * Beta
}

void ErrorCorrection()  //takes in d and q axis currents and returns the error value for the PI filters
{ //D axis current should be 0, q axis should be requested amount, swapped to estimate rotor offset angle
  DaxisCurrError = RequestedCurr - DaxisCurr;
  QaxisCurrError = 0.0 - QaxisCurr; //swap the 0.0 and the requested current for actual motor control
}

void DCurrtoVolt()  //function is PI filter for d axis
{
  TotalDaxisCurrError += DaxisCurrError;
  DaxisVolt = (KpD * DaxisCurrError) + (KiD * TotalDaxisCurrError * micros());  //don't forget 0.000001 later
  //DaxisVolt = 0.0;
  //control signal = P term * error + I term * total error * total time
  if(DaxisVolt >= MaxPhVolt)  //what should this limit be? cannot exceed DC bus voltage, limit should be DC bus / SQRT(3)
  {
    DaxisVolt = MaxPhVolt;
  }
  //can include if/else statements to limit the maximum values
}

void QCurrtoVolt()  //function is PI filter for q axis
{
  TotalQaxisCurrError += QaxisCurrError;
  QaxisVolt = (KpQ * QaxisCurrError) + (KiQ * TotalQaxisCurrError * micros());
  
  //need to add the voltage term for speed and Kv, integral term cannot handle alone
  //CurrentVelocity is the estimated velocity (magnitude and direction) of the speed calculated from the halls, measured in electrical degrees per microsecond
  //SpeedConst is the speed constant of motor in use, typically measured in RPM/V (DC), this is MECHANICAL RPM, not electrical, so also need pole pairs to convert between
  //what is conversion between these for minimal calculation? 
  //{(ele-deg/us) / [(mech-RPM/V) * (Pole Pairs)]} = {(ele-deg/us) / (ele-RPM/V)} = (V * ele-deg) / (ele-RPM * us) = (V * ele-deg * minutes) / (ele-Rev * us)
  //[(V * ele-deg * minutes) / (ele-Rev * us)] * [(1 ele-rev / 360 ele-deg) * (1,000,000 us / 1 second) * (60 seconds / 1 minute)] = Volts
  //[(V * ele-deg * minutes) / (ele-Rev * us)] * [166,666.7] = Volts
  //Back EMF Voltage = (166666.7 * CurrentVelocity) / (PolePairs * SpeedConst) = CurrentVelocity / [0.000006 * PolePairs * SpeedConst] - this is single constant for machine
  //resulting voltage is unfortunately the DC voltage, divide by SQRT(3) to get peak phase voltage/q axis voltage instead
  QaxisVolt += CurrentVelocity / BackEMFV;

  //QaxisVolt = RequestedCurr;
  //control signal = P term * error + I term * total error * total time
  if(QaxisVolt >= MaxPhVolt)  //what should this limit be? cannot exceed DC bus voltage
  {
    QaxisVolt = MaxPhVolt;
  }
  //can include if/else statements to limit the maximum values
}

void IPark()  //inverse park transform, takes d and q command and current position of rotor to calculate alpha and beta command values
{
  //converting equations to have the q axis aligned with the alpha axis
  //AlphaVolt = (SinAngle * DaxisVolt) + (CosAngle * QaxisVolt);
  //BetaVolt = (-1.0 * CosAngle * DaxisVolt) + (SinAngle * QaxisVolt);
  //Inverse Park (dq to alpha beta) with q axis aligned to alpha axis is [sin cos; -cos sin]
  //forward Park (alpha beta to dq) with q axis aligned to alpha axis is [sin -cos; cos sin]


  AlphaVolt = (CosAngle * DaxisVolt) - (SinAngle * QaxisVolt);
  BetaVolt = (SinAngle * DaxisVolt) + (CosAngle * QaxisVolt);
  //above equations assume that the d-axis is aligned with the alpha axis, switch to have the q axis aligned with the alpha axis
  //inverse park transform is inverse of the 2x2 park matrix
  //original matrix has a = cos, b = sin, c = -sin, d = cos
  //inverted matrix is 1/ad-bc * [d -b; -c a] = 1/(cos*cos) - (sin*-sin) * [cos -sin; sin cos] = 1/(cos^2) + (sin^2) * [cos -sin; sin cos] = [cos -sin; sin cos]
}

void IClarke() //inverse Clarke transform, gets alpha and beta command values, transforms into PWM/duty cycles for phases
{
  PhAVOut = AlphaVolt;
  PhBVOut = (-0.5 * AlphaVolt) + (0.86602540378443 * BetaVolt); //0.866 number is SQRT(3)/2
  PhCVOut = (-0.5 * AlphaVolt) - (0.86602540378443 * BetaVolt);
  //these resulting values are the actual voltage outputs, need to divide by DC voltage to determine duty cycle
  //can also add Third Harmonic Injection for 15.5% increased speed/power, need to limit above duty cycle to between 0 and 100%
}

void THI() //third harmonic injection is a method to increase speed performance of a BLDC motor 
{ //by injecting the third harmonic on the neutral voltage (same for all phases) it allows the motor to have ~15% increased speed for the same DC input voltage

  //below calculates angle of output, used for Third Harmonic Injection for 15.5% speed increase
  //atan2 returns position of q axis from phase A in radians
  //OutputAngle = AngleCorrection(57.295779513 * atan2(BetaVolt, AlphaVolt));  //Do angle correction to put -180 to 180 degrees into 0 - 360 degrees 

  //requires 1/6x magnitude of the sine output (can just be of maximum value of 504)
  THI_Mag = cos(3.0 * atan2(BetaVolt, AlphaVolt)) / 6.0;  //this magnitude is from -1 to 1
  //85 is 512 / 6, rounded up, which produces the proper magnitude for maximum Third harmonic injection at maximum
}


void Switching()
{
  DCVoltage = analogRead(DCV) * AConv * DCVConv;
  MaxPhVolt = DCVoltage * 0.5773503; 
  //THI();

  //The phase voltages are actual voltages, dividing by the DC voltage gives the wave to make that with limits of -1 to 1
  //THI_Mag also has limit of -1/6 to 1/6

  DutyCycleA = (PhAVOut / DCVoltage); // - THI_Mag;  //597 is used as this is 512 (half of PWM precision at 1024) plus 85 (1/6 of 512 as this produces maximum voltage difference) 
  DutyCycleB = (PhBVOut / DCVoltage); // - THI_Mag;
  DutyCycleC = (PhCVOut / DCVoltage); // - THI_Mag;

  DcA = 512 * DutyCycleA; //591 is scaling limit of THI with range of 0 - 1023, also turns double into integer instead
  DcB = 512 * DutyCycleB;
  DcC = 512 * DutyCycleC;

  DcA += 511; //output is 0 to 1023, 511 +/- 512 covers entire range between 0 - 1023 (can unfortunately go all the way to -1, this is ok)
  DcB += 511;
  DcC += 511;

  if(DcA < 0) //setting Duty Cycle limits, should stay between 0 and 1023 due to 10 bit system
  {
    DcA = 0;
  }
  if(DcB < 0)
  {
    DcB = 0;
  }
  if(DcC < 0)
  {
    DcC = 0;
  }
  if(DcA > 1023)
  {
    DcA = 1023;
  }
  if(DcB > 1023)
  {
    DcB = 1023;
  }
  if(DcC > 1023)
  {
    DcC = 1023;
  }
}





void FOC()
{
  ReadHalls();
  RotorVelEst();  //calculates the average velocity and acceleration between the last two hall interrupt events, also calls direction calculation, moved to ReadHalls()
  Rotor_Curr_Vel(); //calculates current velocity of the rotor based on last hall velocity and acceleration
  Rotor_Vel_RPM();  //Converts Rotor_Curr_Vel to mechanical RPM integer rather than electrical degrees per microsecond  

  ReadPhaseVoltages();  //testing sensorless control
  
  ReadCurrent();
  CurrentFilter();

  Clarke();
  VoltageFilter();
  ClarkeReadVolt();
  SensorlessRotorPosition();
  VoltageVectorReadMagnitude();

  //current method is to choose wether to use sensored or sensorless based on the voltage magnitude of the back emf
  //Is there a better way to do sensor/sensorless selection? could be averaged between the two, but this would lead to large error at 0 speed (since the BEMF always outputs a small voltage)
  //

  SinAngle = sin(Radians(AngleCorrection(PositionFilter() + Offset_Angle + 90)));
  CosAngle = cos(Radians(AngleCorrection(PositionFilter() + Offset_Angle + 90)));

/*
  if(BEMFVoltMag <= 500.0)
  {
    SinAngle = sin(Radians(AngleCorrection(PositionFilter() + Offset_Angle + 90)));
    CosAngle = cos(Radians(AngleCorrection(PositionFilter() + Offset_Angle + 90)));
  }
  else if(BEMFVoltMag > 500.0)
  {
    SinAngle = sin(Radians(SensorlessAngle + 90));
    CosAngle = cos(Radians(SensorlessAngle + 90));
  }
*/

  Park();

  Potentiometer = analogRead(pot) - 24;

  KpD = Potentiometer * 0.0001;
  KpQ = Potentiometer * 0.0001;

  //KpD = 0.18; //0.26;  //0.13; //0.0000000003125;   //2.8; //0.26 value is from Phaserunner V5 (ebikes.ca controller), 0.18 is calculated value from Thesis paper
  //KiD = 0.0000000000625; //0.00000000000007; //0.000008;   //0.000000002; //297.81 value is from Phaserunner V5 (ebikes.ca controller), 480.0 is calculated value from Thesis paper
  //KpQ = 0.18; //0.26;  //0.13; //0.0000000003125;    //2.8; //0.26 value is from Phaserunner V5 (ebikes.ca controller), 0.18 is calculated value from Thesis paper
  //KiQ = 0.0000000000625; //0.00000000000007; //0.000008;   //0.000000002; //297.81 value is from Phaserunner V5 (ebikes.ca controller), 480.0 is calculated value from Thesis paper

  //Potentiometer = analogRead(pot) - 24;

  if(Potentiometer <= 0)
  {
    AnalogCurrReq = 0.0;
    Activated = false;
    //j = 0;
  }
  else if(Potentiometer > 0)
  {
    AnalogCurrReq = 10.0; //Potentiometer;
    Activated = true;
    //j++;
  }

/*
  //make a step function for testing and tuning the PI Current controllers
  //Serial plotter shows last 50 points, want the width of the pulse of the current controller to be 25 points 
  //should this only pulse once and show the current response 
  if(j < 10)
  {
    RequestedCurr = 0.0;
  }
  else if(j >= 10 && j < 35)  //if i is 1 or higher, then the loop is run
  {
    RequestedCurr = 10.0;  //sets the requested current to value in Amps
  }
  else if(j >= 35 && j < 50)
  {
    RequestedCurr = 0.0;
  }
  while(j >= 50)
  {
    RequestedCurr = 0.0;
    MotStateOff();
    Potentiometer = analogRead(pot) - 24;
    if(Potentiometer <= 0)
    {
      j = 0;
    }
    delay(50);
  }
*/

  //RequestedCurr = 0.02 * AnalogCurrReq;     //switch back to 0.02 later for ~20 A max current
  //if(OverCurrent == true)
  //{
  //  RequestedCurr = RequestedCurr / 3.0;
  //}

  ErrorCorrection();
  DCurrtoVolt();
  QCurrtoVolt();
  IPark();
  IClarke();
  Switching();

  if(DCVoltage <= 20.00)
  {
    LowVoltage = true;
  }
  else if(DCVoltage > 20.00)
  {
    LowVoltage = false;
  }

  if(Activated == true && LowVoltage == false && OverCurrent == false) 
  {
    digitalWrite(SDPhA, HIGH);
    digitalWrite(SDPhB, HIGH);
    digitalWrite(SDPhC, HIGH);

    analogWrite(PWMA, DcA);
    analogWrite(PWMB, DcB);
    analogWrite(PWMC, DcC);
  }

  else //if(Activated == false || OverCurrent == true || LowVoltage == true)
  {
    //pulls enable pins of each phase LOW, Turns off all FETs, blocking current except for back emf, should decelerate motor quickly
    digitalWrite(SDPhA, LOW);
    digitalWrite(SDPhB, LOW);
    digitalWrite(SDPhC, LOW);

    //sets PWM/Duty cycle values for each phase
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    analogWrite(PWMC, 0);

    TotalDaxisCurrError = 0;
    TotalQaxisCurrError = 0;

    //Serial.println("Low Voltage, Overcurrent, or Deactivated");

    //delay(10);
  }

  WheelSpeed();
}




/*
  //make a step function for testing and tuning the PI Current controllers
  //Serial plotter shows last 50 points, want the width of the pulse of the current controller to be 25 points 
  //should this only pulse once and show the current response 

  FOC();

  Serial.print(RequestedCurr);
  Serial.print(", ");
  Serial.print(DaxisCurr);
  Serial.print(", ");
  Serial.print(QaxisCurr);
  Serial.print(", ");
  Serial.print(DaxisVolt);
  Serial.print(", ");
  Serial.println(QaxisVolt);
*/













void WheelSpeed() //function called every time wheel speed sensor goes high, determine speed of wheel
{ //wheel has 6 magnets, called 6 times per wheel rotation, 20" diameter wheel, travels around 0.266 m/magnet, to be unable to use millis, would be traveling at 266 m/s or 595 mph
  TempTimeSpeed = micros();
  SpeedTimeDiff = TempTimeSpeed - LastSpeedTime;

  AngleChange = SensorlessAngle - LastSensorlessAngle;

  AngularSpeed = 138.372 * AngleChange / SpeedTimeDiff; //this is electrical degrees per microsecond

  //converting electrical degrees per microsecond to RPM 
  //edeg/us * 1000000 us/s * 60 s/min * 60 min/hour * 1 erev/360 edeg * 1 mrev/15 erev * 1 wrev/4.78 mrev * 1 mile/1008.4 wrev 
  //138.372 mph / edeg/us

  LastSensorlessAngle = SensorlessAngle;
  LastSpeedTime = TempTimeSpeed;
/*
  if(SpeedTimeDiff > 14875)  //if the time between magnets is more than 14875 microseconds, then traveling above 40 mph, likely magnet error
  {
    LastSpeedTime = TempTimeSpeed;
  
    SpeedTimeDiffSecs = SpeedTimeDiff * 0.000001;  //converting from microseconds to seconds
    SpeedTimeDiffHour = SpeedTimeDiffSecs / 3600.0; //converted to time difference in hours

    //have time, wheel radius, number of magnets per revolution
    //6x magnets and 20" diameter wheel makes 10.472" per magnet (0.266 m/magnet, 0.0001653 miles/magnet)
    //with distance in miles/magnet, need to divide by time change in hours

    WheelSpeedMPH = 0.0001653 / SpeedTimeDiffHour;
    Serial.print(WheelSpeedMPH);
  }
*/
}








void ReadHalls()  //function called every cycle, reads all Hall states and updates position, interrupts removed due to noise
{
  Halls[0] = digitalRead(HallA);  //reads what Hall A has switched to and stores it in array, should this call function to read all hall states?
  Halls[1] = digitalRead(HallB);  //reads what Hall B has switched to and stores it in array
  Halls[2] = digitalRead(HallC);  //reads what Hall C has switched to and stores it in array
  tempHall = HallState();         //temporary storage for current hall state

  //Serial.print("Halls are: ");
  //Serial.println(tempHall);

  if(tempHall != CurrHallState && tempHall != 7)
  {
    //function updates current hall state, records time
    PrevStateTime = CurrStateTime;  //stores the last state time into the previous 
    CurrStateTime = micros(); //retrieves new data for current state time
    PrevHallState = CurrHallState;  //stores "current" hall state in previous
    CurrHallState = tempHall;
    RotorVelEst();
  }
} //end of ReadHalls

int HallState() //function is called at each hall interrupt, calculates hall state from array
{
  //function should take array and put halls in state (0-7)
  int temp = Halls[0];  //sets the C hall equal to temp (can be 0 or 1)
  temp += Halls[1]*2;   //adds the B hall to temp (0 or 1 * 2 results in up to +2)
  temp += Halls[2]*4;   //adds the A hall to temp (0 or 1 * 4 results in up to +4)
  return temp;          //returns int with binary ABC 
} //end of HallState

//Motor position
//Have 3 halls to sense rotor position, then calculate vector for what the phase currents should be at and etc
//During normal operation, interrupts on halls will record time that a hall changes (in micros)
//This gives you the current state and you should know the previous state (receding data into register/storage)
//known that change from one state to the next is 60 electrical degrees (ish), can then divide 60 degrees by the change in time (in micros)
//will get degrees per microsecond for the average velocity of the last hall state/section
//if previous velocity data is maintained with change in time between them (from hall interrupt from 2 states ago and from past state) can then calculate acceleration estimation in electrical degrees per us^2
//This means that 3 hall times need to be maintained and rotated through, 2x velocities (assuming 60 degree separation), and single acceleration
//With initial position, last velocity estimate, and last acceleration estimate the current position can be calculated
//for 90 degree phase angle, simply add 90 electrical degrees and calculate the stator currents for that state/position
//an additional measure may be to record the hall states for each change (record current hall state after every change) can help with direction determination 
//Since velocity information is also available, may be able to determine/calculate the back EMF, need to know/estimate kV
//for actual position information, the angle of each hall state needs to be known, which depends on direction if that is the low or high side of the range
//"velocity" calculation will actually be speed, the real velocity calculation will require a +/- differentiation from current and previous state, this means acceleration will always be acceleration of the rotor (if rotor is speeding up or slowing down, not absolute direction, right?)
//how to best split up calculations into function?
  //each phase has separate interrupt, calls separate function for each hall
    //each of these functions records the current time (micros), reads what the single hall changed to
    //they call a function to convert the binary hall state array into an integer value (0 - 8) This is HallState() function
  //Need a function to calculate current velocity, divides 60 electrical degrees by the current state time - previous state time
  //need a function to determine if rotor is spinning in positive or negative direction, use current and last position (hall state) with the below state/positon assumptions
  //Need a function to calculate the acceleration, uses current and last velocity, also uses time difference between last position and previous previous position, this is same time difference as used for last velocity
    //only need to store 2 positions(unsigned integer/hall state), 2 times(unsigned integer/micros), 2 time differences(unsigned integer/micros), 2 velocities(signed float/degrees/microsecond), and 1 acceleration (signed float/degrees/us^2) 

//60 degrees in 1 microsecond, what motor RPM does this result in
//Motor has 7 pole pairs, 
//60 electrical degrees in 1 microsecond would be 360 degrees in 6 microseconds, 
//one mechanical revolution in 42 us, this would be 23,809 Revolutions per second
//this results in 1,428,571 RPM
//at the maximum speed of ~5000 RPM, this results in 83.3 revolutions per second
//7 pole pairs results in 583 electrical revolutions per second
//with 6 divisions per electrical rotation, this is 3500 sections per second, or a hall state every 285.7 us

//If velocity is an integer with a step size of 1 electrical degree per microsecond increments
//This is a minimum speed of 360 electrical degrees in 360 us
//with 7 pole pairs, 1 mechanical rotation would be completed in 2.52 ms
//converting results in 396.8 revolutions per second or 23,809 RPM
//acceleration and velocity values likely need to be floats due to how large a value is 1 degree/us

void RotorVelEst()  //function is here to calculate the velocity estimate for the average of the last two hall states
{
  PrevStateVel = CurrentStateVel; //stores the "current" velocity into the previous velocity space
  Prev_Time_Diff = Time_Diff;  //stores the "current" time difference into the previous time difference

  Time_Diff = CurrStateTime - PrevStateTime;  //calculates time difference from current and previous times
  if(Time_Diff == 0)  //prevents system from dividing by 0 and stalling
  {
    Time_Diff = 1;
  }
  CurrentStateVel = 60.0 / Time_Diff; //calculates average velocity in degrees per microsecond across the last hall state, assumes all halls are equally at 60 degree separation
//  if(Time_Diff >= 700000) //during low speeds, the velocity estimation becomes inaccurate, need any time greater than 1 RPM to be zeroed, 1 RPM is 700,000 us
//  {
//    CurrentStateVel = 0;
//  }
  MotorDir(); //function determines what the motor direction is by checking if the hall order is positive or negative
  CurrentStateVel *= Dir;  //this multiplies the current state speed by the direction (calculated in Motor_Dir function), turns CurrentStateVel into a vector rather than a scalar
  
  PrevAcceleration = CurrentStateVel - PrevStateVel;  //begins calculation for previous acceleration by storing the change in velocity
  PrevAcceleration = PrevAcceleration / Prev_Time_Diff; //completes calculation for previous acceleration by dividing change in velocity by amount of time for that change
//  if(Time_Diff >= 700000)
//  {
//    PrevAcceleration = 0;
//  }
}





void Rotor_Curr_Vel() //function calculates current estimated velocity of the rotor from current time
{ //using kinematic equation v = v0 + at to where v and v0 are measured in degrees per microsecond, a is degrees per microsecond squared, and t is microseconds
  Curr_Time_Diff = CurrTime - CurrStateTime;
  CurrentVelocity = 0;
  CurrentVelocity = PrevAcceleration * Curr_Time_Diff;
  CurrentVelocity += CurrentStateVel; //returns result as degrees per microsecond
//  if(Curr_Time_Diff >= 700000)
//  {
//    CurrentVelocity = 0;
//  }
  //make function a float and return the variable?
  //should function convert to RPM or rad/s for simpler back emf calculation
}

void Rotor_Vel_RPM()  //function converts Rotor_Curr_Vel() function from degrees per microsecond to RPM
{
  CurrRPMEst = 0;
  CurrRPMEst = CurrentVelocity * 166667; //Current Velocity is in electrical degrees per microsecond, Converting to electrical revolution per minute takes (1 rev / 360 degrees) * (1,000,000 us / second) * (60 seconds/minute) = 166,666.7 rev/degree * us/minute
  CurrRPMEst = CurrRPMEst / PolePairs; //need to convert to mechanical RPM from electrical, If the number of pole pairs are known then the electrical RPM will be the number of pole pairs more than the mechanical RPM
}











double PositionFilter()
{
  //Setting up IIR digital filter for position, done on value before correction to be between 0 and 359 degrees
  CurrentTimePos = micros();
  PosTimeDiff = (CurrentTimePos - PrevTimePos) * 0.000001;
  PrevTimePos = CurrentTimePos;

  IIRPosFilterAlpha = FilterTimePos / (FilterTimePos + PosTimeDiff);
  IIRPosFilterOneMinusAlpha = 1.0 - IIRPosFilterAlpha;

  FilteredPos = (IIRPosFilterAlpha * LastFilteredPos) + (IIRPosFilterOneMinusAlpha * Rotor_Curr_Pos());
  LastFilteredPos = FilteredPos;
  return FilteredPos;
}






int Rotor_Curr_Pos()  //Uses last known position, velocity, and acceleration to calculate the position (electrical degrees) of the rotor
{
  CurrTime = micros();  //records the current time in microseconds
  Curr_Time_Diff = CurrTime - CurrStateTime;  //Calculates time difference from now to whenever the last hall state change was
  float tempAcc = sq(Curr_Time_Diff);  //square of the current, used to calculate the change in position due to acceleration (following 1/2 a t^2)
  tempAcc = tempAcc * PrevAcceleration * 0.5; //finalizing change in position due to acceleration equation

  float tempVel = CurrentStateVel * Curr_Time_Diff; //calculating the change in position due to velocity

//  if(Curr_Time_Diff >= 700000)
//  {
//    tempAcc = 0;
//    tempVel = 0;
//  }

  if(Dir == 1)
  {
    CurrPos = MinPos + tempVel + tempAcc;  //calculating new absolute angle by adding the original angle, the change due to velocity, and the change due to acceleration 
  }
  else if(Dir == -1)
  {
    CurrPos = MaxPos + tempVel + tempAcc; //should this be - the temp vel and acc values instead of +?
  }

  //Serial.print("Last Rotor Position is: ");
  //Serial.print(MinPos);
  //Serial.println(" Degrees");

  if(Dir == 1 && CurrPos >= MaxPos)
  {
    //need some method to check if the CurrPos is past (negative or positive) the MaxPos, then have it set the angle to the Max Pos, since the hall hasn't been hit yet
    //make function to check that CurrPos is in between MaxPos and MinPos, if it's not then set it to Max Pos
    CurrPos = MaxPos;
  }
  else if(Dir == -1 && CurrPos <= MinPos)
  {
    CurrPos = MinPos;
  }

  return CurrPos;
}

//below is positive rotation, fix, is wrong direction, can calibrate for angle
//if hall state 001 (1) is 0 degrees, then assume state 1 is 0 - 59 electrical degrees, 1 - 3 transition around stator angle 20 (13?)
//hall state 101 (5) is 60 to 119 electrical degrees, 5 - 1 transition around stator angle 336 (353?)
//hall state 100 (4) is 120 to 179 electrical degrees, 4 - 5 transition around stator angle 253
//hall state 110 (6) is 180 to 239 electrical degrees, 6 - 4 transition around stator angle 211 (203?)
//hall state 010 (2) is 240 to 299 electrical degrees, 2 - 6 transition around stator angle 172 
//hall state 011 (3) is 300 to 359 electrical degrees, 3 - 2 transition around stator angle 83 (79)
//fixing for actual positive direction (reversing)
//hall state 001 (1) is 20 degrees to 79 degrees
//hall state 011 (3) is 80 degrees to 139 degrees
//hall state 010 (2) is 140 degrees to 199 degrees
//hall state 110 (6) is 200 degrees to 259 degrees
//hall state 100 (4) is 260 degrees to 319 degrees
//hall state 101 (5) is 320 degrees to 19 degrees
void MotorDir() //hall state 6 transitions to 4 at 100 degrees, 4 to 5 at 160 degrees, 5 to 1 at 220 degrees, 1 to 3 at 280 degrees, 3 to 2 at 337 degrees, 2 to 6 at 38 degrees
{
  //function needs to take in current hall state and previous hall state and determine if direction is positive or negative
  //include error checking if a state or multiple were skipped

  //Hall states don't allow for incremental counting, can't use > or < logic to determine direction
  //is case or if/else better for this? Initial should be case, following determination should be 2x if with single else for error checking
  switch(CurrHallState)
  {
    case 1: //This means hall state is state 1, positive direction is from state 3, negative direction is from state 5
      MinPos = 0;  //this is angle of rotor at this time, since direction is known and angle range is also known
      MidPos = 30;  //this is estimated angle of rotor for the middle of the hall state
      MaxPos = 59;  //this is maximum angle of the rotor when traveling in this direction
      //Serial.println("Hall State Case 1");
      if(PrevHallState == 1)  //If the state has not changed and the MotorDir() function is called, then 
      {
        //Serial.println("Hall state has not changed yet");
      }
      else if(PrevHallState == 5)
      {
        Dir = 1;
      }
      else if(PrevHallState == 3)
      {
        Dir = -1;
      }
      else  //this is error case, means that steps were skipped, could have system double check the hall state
      {
        //if(print == true)
        //{
        //  Serial.println("Hall states were skipped, check wiring"); //may also be that hall bounces or rotor goes back and forth
        //}
      }
    break;

    case 3: //This means hall state is state 3, positive direction is from state 2, negative direction is from state 1
      MinPos = 60; //this is angle of rotor at this time, since direction is known and angle range is also known
      MidPos = 90;
      MaxPos = 119;
      //Serial.println("Hall State Case 3");
      if(PrevHallState == 3)  //
      {
        //Serial.println("Hall state has not changed yet");
      }
      else if(PrevHallState == 1)
      {
        Dir = 1;
      }
      else if(PrevHallState == 2)
      {
        Dir = -1;
      }
      else  //this is error case, means that steps were skipped, could have system double check the hall state
      {
        //if(print == true)
        //{
        //  Serial.println("Hall states were skipped, check wiring");
        //}
      }
    break;

    case 2: //This means hall state is state 2, positive direction is from state 6, negative direction is from state 3
      MinPos = 120; //this is angle of rotor at this time, since direction is known and angle range is also known
      MidPos = 150;
      MaxPos = 179;
      //Serial.println("Hall State Case 2");
      if(PrevHallState == 2)  //
      {
        //Serial.println("Hall state has not changed yet");
      }
      else if(PrevHallState == 3)
      {
        Dir = 1;
      }
      else if(PrevHallState == 6)
      {
        Dir = -1;
      }
      else  //this is error case, means that steps were skipped, could have system double check the hall state
      {
        //if(print == true)
        //{
        //  Serial.println("Hall states were skipped, check wiring");
        //}
      }
    break;

    case 6: //This means hall state is state 6, positive direction is from state 4, negative direction is from state 2
      MinPos = 180; //this is angle of rotor at this time, since direction is known and angle range is also known
      MidPos = 210;
      MaxPos = 239;
      //Serial.println("Hall State Case 6");
      if(PrevHallState == 6)  //
      {
        //Serial.println("Hall state has not changed yet");
      }
      else if(PrevHallState == 2)
      {
        Dir = 1;
      }
      else if(PrevHallState == 4)
      {
        Dir = -1;
      }
      else  //this is error case, means that steps were skipped, could have system double check the hall state
      {
        //if(print == true)
        //{
        //  Serial.println("Hall states were skipped, check wiring");
        //}
      }
    break;

    case 4: //This means hall state is state 4, positive direction is from state 5, negative direction is from state 6
      MinPos = 240; //this is angle of rotor at this time, since direction is known and angle range is also known
      MidPos = 270;
      MaxPos = 299;
      //Serial.println("Hall State Case 4");
      if(PrevHallState == 4)  //
      {
        //Serial.println("Hall state has not changed yet");
      }
      else if(PrevHallState == 6)
      {
        Dir = 1;
      }
      else if(PrevHallState == 5)
      {
        Dir = -1;
      }
      else  //this is error case, means that steps were skipped, could have system double check the hall state
      {
        //if(print == true)
        //{
        //  Serial.println("Hall states were skipped, check wiring");
        //}
      }
    break;

    case 5: //This means hall state is state 5, positive direction is from state 1, negative direction is from state 4
      MinPos = 300; //this is angle of rotor at this time, since direction is known and angle range is also known
      MidPos = 330;
      MaxPos = 359;
      //Serial.println("Hall State Case 5");
      if(PrevHallState == 5)  //
      {
        //Serial.println("Hall state has not changed yet");
      }
      else if(PrevHallState == 4)
      {
        Dir = 1;
      }
      else if(PrevHallState == 1)
      {
        Dir = -1;
      }
      else  //this is error case, means that steps were skipped, could have system double check the hall state
      {
        //if(print == true)
        //{
        //  Serial.println("Hall states were skipped, check wiring");
        //}
      }
    break;

    default:  //this is if CurrHallState = 0 or 7, both illegal states
      Dir = 0;
      //if(print == true)
      //{
      //  Serial.println("Error, Illegal Hall State");
      //}
    break;
  }
}






int AngleCorrection(int Any_Angle)  //function takes in integer angles, corrects to be within 0 - 359 degrees, then returns corrected angle
{
  while(Any_Angle >= 360)
  {
    Any_Angle -= 360;
  }
  while(Any_Angle < 0)
  {
    Any_Angle += 360;
  }
  return Any_Angle;
}



















/*
double CurrentMag   = 0;
int VoltageMag      = 0;
//float AlphaI      = 0;
//float BetaI       = 0;
//float DIMag       = 0;
//float QIMag       = 0;

void IMagCalc() //Calculates Clarke and Park transform to result in d-q reference frame for current magnitude 
{
  PhaseAI = analogRead(PhAI) - 509; //offset to "calibrate" current sensors to a 0 value
  PhaseBI = analogRead(PhBI) - 509;
  PhaseCI = analogRead(PhCI) - 509;

  //AlphaI = PhaseAI * 2/3 - PhaseBI / 3 - PhaseCI / 3; //clarke transform to put abc into alpha beta frame
  //BetaI = 0.5774 * PhaseBI - 0.5774 * PhaseCI;  //clarke transform to put abc into alpha and beta frame

  //need to replace theta below with what the current rotor angle is, park/clarke transform will make sure field is correct
  //DIMag = cos(theta) * AlphaI + sin(theta) * BetaI; //park transform to put alpha beta frame into dq reference frame
  //QIMag = -sin(theta) * AlphaI + cos(theta) * BetaI;  //park transform to put alpha beta frame into dq reference frame

  CurrentMag = sq(PhaseAI) + sq(PhaseBI) + sq(PhaseCI);
  CurrentMag = sqrt(CurrentMag);  //calculates current magnitude by = SQRT(a^2 + b^2 + c^2)
}



//sholuld this be removed and just use the sin(), cos(), and tan() functions as well as asin(), acos(), and atan() for inverse functions, they use radians, so must convert
float SinAngle[360] = { 0.0000, 0.0175, 0.0349, 0.0523, 0.0698, 0.0872, 0.1045, 0.1219, 0.1392, 0.1564, 0.1736, 0.1908, 0.2079, 0.2250, 0.2419, 0.2588, 0.2756, 0.2924, 0.3090, 0.3256, 0.3420, 0.3584, 0.3746, 0.3907, 0.4067, 0.4226, 0.4384, 0.4540, 0.4695, 0.4848, 
                        0.5000, 0.5150, 0.5299, 0.5446, 0.5592, 0.5736, 0.5878, 0.6018, 0.6157, 0.6293, 0.6428, 0.6561, 0.6691, 0.6820, 0.6947, 0.7071, 0.7193, 0.7314, 0.7431, 0.7547, 0.7660, 0.7771, 0.7880, 0.7986, 0.8090, 0.8192, 0.8290, 0.8387, 0.8480, 0.8572, 
                        0.8660, 0.8746, 0.8829, 0.8910, 0.8988, 0.9063, 0.9135, 0.9205, 0.9272, 0.9336, 0.9397, 0.9455, 0.9511, 0.9563, 0.9613, 0.9659, 0.9703, 0.9744, 0.9781, 0.9816, 0.9848, 0.9877, 0.9903, 0.9925, 0.9945, 0.9962, 0.9976, 0.9986, 0.9994, 0.9998, 
                        1.0000, 0.9998, 0.9994, 0.9986, 0.9976, 0.9962, 0.9945, 0.9925, 0.9903, 0.9877, 0.9848, 0.9816, 0.9781, 0.9744, 0.9703, 0.9659, 0.9613, 0.9563, 0.9511, 0.9455, 0.9397, 0.9336, 0.9272, 0.9205, 0.9135, 0.9063, 0.8988, 0.8910, 0.8829, 0.8746, 
                        0.8660, 0.8572, 0.8480, 0.8387, 0.8290, 0.8192, 0.8090, 0.7986, 0.7880, 0.7771, 0.7660, 0.7547, 0.7431, 0.7314, 0.7193, 0.7071, 0.6947, 0.6820, 0.6691, 0.6561, 0.6428, 0.6293, 0.6157, 0.6018, 0.5878, 0.5736, 0.5592, 0.5446, 0.5299, 0.5150, 
                        0.5000, 0.4848, 0.4695, 0.4540, 0.4384, 0.4226, 0.4067, 0.3907, 0.3746, 0.3584, 0.3420, 0.3256, 0.3090, 0.2924, 0.2756, 0.2588, 0.2419, 0.2250, 0.2079, 0.1908, 0.1736, 0.1564, 0.1392, 0.1219, 0.1045, 0.0872, 0.0698, 0.0523, 0.0349, 0.0175, 
                        0.0000, -0.0175, -0.0349, -0.0523, -0.0698, -0.0872, -0.1045, -0.1219, -0.1392, -0.1564, -0.1736, -0.1908, -0.2079, -0.2250, -0.2419, -0.2588, -0.2756, -0.2924, -0.3090, -0.3256, -0.3420, -0.3584, -0.3746, -0.3907, -0.4067, -0.4226, -0.4384, -0.4540, -0.4695, -0.4848, 
                       -0.5000, -0.5150, -0.5299, -0.5446, -0.5592, -0.5736, -0.5878, -0.6018, -0.6157, -0.6293, -0.6428, -0.6561, -0.6691, -0.6820, -0.6947, -0.7071, -0.7193, -0.7314, -0.7431, -0.7547, -0.7660, -0.7771, -0.7880, -0.7986, -0.8090, -0.8192, -0.8290, -0.8387, -0.8480, -0.8572, 
                       -0.8660, -0.8746, -0.8829, -0.8910, -0.8988, -0.9063, -0.9135, -0.9205, -0.9272, -0.9336, -0.9397, -0.9455, -0.9511, -0.9563, -0.9613, -0.9659, -0.9703, -0.9744, -0.9781, -0.9816, -0.9848, -0.9877, -0.9903, -0.9925, -0.9945, -0.9962, -0.9976, -0.9986, -0.9994, -0.9998, 
                       -1.0000, -0.9998, -0.9994, -0.9986, -0.9976, -0.9962, -0.9945, -0.9925, -0.9903, -0.9877, -0.9848, -0.9816, -0.9781, -0.9744, -0.9703, -0.9659, -0.9613, -0.9563, -0.9511, -0.9455, -0.9397, -0.9336, -0.9272, -0.9205, -0.9135, -0.9063, -0.8988, -0.8910, -0.8829, -0.8746, 
                       -0.8660, -0.8572, -0.8480, -0.8387, -0.8290, -0.8192, -0.8090, -0.7986, -0.7880, -0.7771, -0.7660, -0.7547, -0.7431, -0.7314, -0.7193, -0.7071, -0.6947, -0.6820, -0.6691, -0.6561, -0.6428, -0.6293, -0.6157, -0.6018, -0.5878, -0.5736, -0.5592, -0.5446, -0.5299, -0.5150, 
                       -0.5000, -0.4848, -0.4695, -0.4540, -0.4384, -0.4226, -0.4067, -0.3907, -0.3746, -0.3584, -0.3420, -0.3256, -0.3090, -0.2924, -0.2756, -0.2588, -0.2419, -0.2250, -0.2079, -0.1908, -0.1736, -0.1564, -0.1392, -0.1219, -0.1045, -0.0872, -0.0698, -0.0523, -0.0349, -0.0175};
//cosine is just SinAngle[Angle + 90], need to be aware of values greater than 359 degrees

float Cos_Angle(int Stator_Angle) //function takes in angle and finds the cosine of it from sine lookup table, Cos(angle) = Sin(90 + angle)
{
  Stator_Angle += 90; //
  return SinAngle[AngleCorrection(Stator_Angle)];
}

void MotorPos()//takes in current rotor angle and applies stator field to align to that angle with magnitude, want for a positive angle/progression to be forward on the vehicle
{
  IMagCalc();
  Angle = AngleCorrection(Rotor_Curr_Pos() + Advance_Angle + Offset_Angle); //assumes positive direction of travel, change to - 90 if wanting negative rotation, may tie rotation direction to the direction of the switch on PCB
  
  Potentiometer = map(analogRead(pot), 0, 1024, -25, 886);  //this gives the potentiometer a value from -25 to 627, want anything below zero to set duty cycle to 0
  //using current control, what is maximum magnitude? Ideal 3 phase is up to 627 current magnitude, maximum of 512 on all phases is 886

  //Serial.print("Stator Field Angle: ");
  //Serial.print(Angle);
  //Serial.print(", Advance Angle: ");
  //Serial.print(Advance_Angle);
  //Serial.print(", Phase A Current: ");
  Serial.print("Variable_1:");
  Serial.print(PhaseAI);
  //Serial.print(", Phase B Current: ");
  Serial.print(", ");
  Serial.print("Variable_2:");
  Serial.print(PhaseBI);
  //Serial.print(", Phase C Current: ");
  Serial.print(", ");
  Serial.print("Variable_3:");
  Serial.print(PhaseCI);

  //Serial.print(", Current Magnitude: ");
  Serial.print(", ");
  Serial.print("Variable_4:");
  Serial.print(CurrentMag);

  //Serial.print(", Voltage Magnitude: ");
  Serial.print(", ");
  Serial.print("Variable_5:");
  Serial.println(VoltageMag);

  if(CurrentMag < Potentiometer)
  {
    VoltageMag++;
  }
  else if(CurrentMag > Potentiometer)
  {
    VoltageMag--;
  }

  if(PhaseAI > 365 || PhaseAI < -365 || PhaseBI > 365 || PhaseBI < -365 || PhaseCI > 365 || PhaseCI < -365)  //limiting motor phase to around +/- 20 A
  {
    VoltageMag--;
  }
  
  if(VoltageMag < 0)
  {
    VoltageMag = 0;
  }
  if(VoltageMag > 886)
  {
    VoltageMag = 886;
  }

  //with below code, Ampl_Out is the expected voltage vector magnitude, goes from 0 to 1.5 (can be negative for regen or reverse direction, not used now)
  //the below calls to SinAngle[Angle + 90] is asking for the Cosine
  //PhaseA = Ampl_Out * CosAngle(Angle) / 1.5; //calculates the Phase A PWM value from potentiometer value and the angle of the rotor (+90 degrees as above)
  //PhaseC = (Ampl_Out * SinAngle[Angle] / -1.7321) - (Ampl_Out * CosAngle(Angle) / 3);  //Phase C calculated from (Sine/-SQRT(3)) - Cos/3
  //PhaseB = - PhaseA - PhaseC; //phase B calculated from subtraction of phase A and phase C
  
  //converting to just calculate the float for the % on time
  //below is technically Clarke Transformation
  PhaseA = Cos_Angle(Angle); //calculates the Phase A PWM value from potentiometer value and the angle of the rotor (+90 degrees as above)
  PhaseC = (SinAngle[Angle] / -1.1547) - (Cos_Angle(Angle) / 2);  //Phase C calculated from (Sine/-SQRT(3)) - Cos/3
  PhaseB = - PhaseA - PhaseC; //phase B calculated from subtraction of phase A and phase C
  
  //Serial.print("Phase float values: ");
  //Serial.print(PhaseA);
  //Serial.print(", ");
  //Serial.print(PhaseB);
  //Serial.print(", ");
  //Serial.println(PhaseC);

  THI();  //calls third harmonic injection function to allow for up to 15% improved speed with same input voltage  
  
  DcA = (PhaseA * VoltageMag) - THI_Mag + 512; //duty cycle integer for Phase A, should have upper limit of 1007, and lower limit of 0
  DcB = (PhaseB * VoltageMag) - THI_Mag + 512; //duty cycle integer for Phase B, should have upper limit of 1007, and lower limit of 0
  DcC = (PhaseC * VoltageMag) - THI_Mag + 512; //duty cycle integer for Phase C, should have upper limit of 1007, and lower limit of 0

  //if(i == 1)  //intent is to print below values every i number of cycles, helps to reduce load on serial monitor
  //{
    //Serial.print("Variable_1:");
    //Serial.print(Angle);
    //Serial.print(", ");
    //Serial.print("Variable_2:");
    //Serial.print(DcA); //Phase A analog Voltage read
    //Serial.print(", ");
    //Serial.print("Variable_3:");
    //Serial.print(DcB); //Phase B analog Voltage read
    //Serial.print(", ");
    //Serial.print("Variable_4:");
    //Serial.print(DcC); //Phase C analog Voltage read
    //Serial.print(", ");
    //Serial.print("Variable_5: ");
    //Serial.print(CurrHallState);
    //Serial.print(", ");
    //Serial.print("Variable_1: ");
    //Serial.print(CurrentMag);
    //Serial.print(", ");
    //Serial.print("Variable_2: ");
    //Serial.print(AlphaI);
    //Serial.print(", ");
    //Serial.print("Variable_3: ");
    //Serial.println(BetaI);

    //i = 0;
  //}
  //i++;

  if(Potentiometer > 0) //if the potneitometer is at a value greater than 0, then it will work with duty cycle
  {
    //pulls enable pins of each phase high, allows switching to take place
    digitalWrite(SDPhA, HIGH);
    digitalWrite(SDPhB, HIGH);
    digitalWrite(SDPhC, HIGH);

    //sets PWM/Duty cycle values for each phase
    analogWrite(PWMA, DcA);
    analogWrite(PWMB, DcB);
    analogWrite(PWMC, DcC);
  }
  else  //if the potentiometer is less than or equal to zero, then the PWM values are zeroed and gate drivers are disabled
  {
    //pulls enable pins of each phase LOW, Turns off all FETs, blocking current except for back emf, should decelerate motor quickly
    digitalWrite(SDPhA, LOW);
    digitalWrite(SDPhB, LOW);
    digitalWrite(SDPhC, LOW);

    //sets PWM/Duty cycle values for each phase
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    analogWrite(PWMC, 0);
  }
  //if the potentiometer is >0, then will let the duty cycle values pass onto the PWM of the phases

  //Can this phase % duty cycle be multiplied by the pwm/potentiometer signal (limited to stay within 0 - 1007) and be added to a PWM of 504 (half between 0 and 1007) 
  //use potentiometer value with a range of 0 - 504, use Sin with a maximum amplitude of -1 to 1
  //Phase A, B, and C PWM can go from 0 - 1007 (limitations with maximum PWM of IR2104 gate drivers)
  //Sine will rely on 504 +/- the PWM, which can reach up to a magnitude of 504
  //the angle of the rotor + 90 degrees (positive direction of rotation) will set the stator angle for MTPA (Maximum Torque Per Ampere)
  //Magnitude can be driven by the Potentiometer value (for now) eventually want current control and voltage control/speed compensation
  //What is math needed?
  //Need to turn vector angle (rotor angle +90 degrees for positive direction, -90 degrees for negative direction) into % of each phase
    //what does this take?
    //angle is known and magnitude is assumed to be 1
    //Phase A is assumed to be at 0 degrees, Phase B at 120 degrees, Phase C at 240 degrees
    //Magnitude @ angle = x @ 0 degrees + y @ 90 degrees = Phase A @ 0 degrees + Phase B @ 120 degrees + Phase C @ 240 degrees
    //Phase A is conveniently 100% in the x direction, B and C combine to provide the rest of the X direction
    //Only phases B and C provide y axis field (don't need to calculate a for this)
    //Also know that A + B + C = 0 (magnitudes) since motor is Wye connected
    //What is maximum Magnitude for Phase A, B, and C having limits of -1 to 1
      //a three phase sine wave at angle zero = cos(0) + cos(120) + cos(240) = 1 + (-0.5) + (-0.5) = 0 -> showing that all current input on one leg goes out the other two
      //need vector addition to determine 
      //first x dimension calculations
      //Magnitude cos(0) = Phase A cos(0) + Phase B cos(120) + Phase C cos(240)
      //Magnitude * 1 = Phase A * 1 + Phase B * (-0.5) + Phase C * (-0.5)
      //Magnitude = Phase A - Phase B / 2 - Phase C / 2
      //next Y dimension calculation
      //Magnitude * Sin(0) = Phase A * Sin(0) + Phase B * sin(120) + Phase C sin(240)
      //Magnitude * 0 = Phase A * 0 + Phase B * 0.866 + Phase C * (-0.866)
      //0 = Phase B * 0.866 - Phase C * 0.866
      //Phase B = Phase C
      //plugging back into x dimension equation
      //Magnitude = Phase A - Phase B / 2 - Phase B / 2
      //Magnitude = Phase A - Phase B
      //Wye connection means that Phase A + Phase B + Phase C = 0
      //if Phase A can be up to 1, then it means that Phase B and C can be up to -0.5 (when they are equal to one another)
      //Magnitude = 1 - (-0.5) = 1.5
    //this means that phase A can be 2/3 of the total magnitude in the x direction (Cos)
    //Can use remaining 0.5 split between phase B and C, use vertical to calculate how much each gets

  //analog write value for each one will be half when the requested current = 0
  //while system is idle, want for all phases to be pulled low
  //when system has a requested current, need to make vector of current using adjacent states
  //amplitude of vector current is vector sum 
  //  vector control has Amplitude @ angle = Amplitude_A @ A + Amplitude_B @ B + Amplitude_C @ C
  //  assume angle A = 0 degrees, B = 120 degrees, C = 240 degrees
  //  also known that Amplitude_A + Amplitude_B + Amplitude_C = 0
  //    the vector addition actually results in Amplitude @ angle = Amplitude_A sin(A) + Amplitude_A cos(A) + Amplitude_B sin(B) + Amplitude_B cos(B) + Amplitude_C sin(C) + Amplitude_C cos(C)
  //    Amplitude_x = Amplitude_A cos(A) + Amplitude_B cos(B) + Amplitude_C cos(C)
  //    Amplitude_y = Amplitude_A sin(A) + Amplitude_B sin(B) + Amplitude_C sin(C)
  //    Result is Amplitude @ Angle = Amplitude_x cos(Angle) + Amplitude_y sin(Angle)
  //
  //then need to do Amplitude * cos(Angle) = Amplitude_A cos(0) + Amplitude_B cos(120) + Amplitude_C cos(240)
  //also            Amplitude * sin(Angle) = Amplitude_A sin(0) + Amplitude_B sin(120) + Amplitude_C sin(240)
  //Need to include                      0 = Amplitude_A        + Amplitude_B          + Amplitude_C
  //
  //Amplitude * cos(Angle) = Amplitude_A * 1.0000 + Amplitude_B * -0.5000 + Amplitude_C * -0.5000
  //Amplitude * sin(Angle) = 0.0000               + Amplitude_B * 0.8660  + Amplitude_C * -0.8660
  //                     0 = Amplitude_A          + Amplitude_B           + Amplitude_C
  //
  //Amplitude_B = (((Amplitude_C * -0.8660) - (Amplitude * sin(Angle))) / 0.8660)
  //Amplitude * cos(Angle) = Amplitude_A * 1.0000 + (((Amplitude_C * -0.8660) - (Amplitude * sin(Angle))) / 0.8660) * -0.5000 + Amplitude_C * -0.5000
  //
  //Addition/subtraction method
  //Amplitude * cos(Angle) = Amplitude_A * 1.0000 + Amplitude_B * -0.5000 + Amplitude_C * -0.5000
  // - (                 0 = Amplitude_A          + Amplitude_B           + Amplitude_C)
  //----------------------------------------------------------------------------------------------
  //Amplitude * cos(Angle) = 0 + Amplitude_B * -0.5000 - Amplitude_B + Amplitude_C * -0.5000 - Amplitude_C
  //Amplitude * cos(Angle) = -1.5 * Amplitude_B - 1.5 Amplitude_C
  //
  //             Amplitude * cos(Angle) = -1.5 * Amplitude_B - 1.5 Amplitude_C
  //+1.5/0.8660 (Amplitude * sin(Angle) = Amplitude_B * 0.8660  + Amplitude_C * -0.8660)
  //-------------------------------------------------------------------------------------
  //Amplitude * cos(Angle) + (1.5 * Amplitude * sin(Angle) / 0.8660) = -1.5 * Amplitude_B + 1.5 * Amplitude_B - 1.5 Amplitude_C - 1.5 Amplitude_C
  //Amplitude * cos(Angle) + (1.5 * Amplitude * sin(Angle) / 0.8660) = -3.0 Amplitude_C
  //
  //Amplitude_C = -(Amplitude * cos(Angle) / 3.0) - (Amplitude * sin(Angle) / 0.5774)                                          testing with Amplitude = 630 V / 975 V = 0.6462, angle is 38.3 degrees, results in -0.8626 for Amplitude_C, -0.4002 for Amplitude B, and equation 1 returns 0.5071 for Amplitude_A while equation 3 returns -1.2629 (illegal value)
  //
  //plug back into equation 2 (Amplitude * sin(Angle) = 0.0000 + Amplitude_B * 0.8660  + Amplitude_C * -0.8660)
  //  Amplitude_B = (Amplitude * sin(Angle)/0.8660) + Amplitude_C                                        
  //
  //plug back into either equation 1 (Amplitude * cos(Angle) = Amplitude_A * 1.0000 + Amplitude_B * -0.5000 + Amplitude_C * -0.5000) or equation 3 (0 = Amplitude_A + Amplitude_B + Amplitude_C)
  //  Amplitude_A = Amplitude * cos(Angle) + 0.5 * Amplitude_B + 0.5 * Amplitude_C
  //  Amplitude_A = - Amplitude_B - Amplitude_C
}
*/





void TestDCV()  //function tests voltage of FETs when switching, makes sure they all still work, Would like to implement additional current testing to make sure all FETs are health at low resistance, requires passing current through motor
{
  MotStateOff();

  delay(2);

  if(analogRead(PhAV) <= 5)
  {
    //this means that the phase A high side FET can turn off as it should
    PhAHTest = true;
  }

  if(analogRead(PhBV) <= 5)
  {
    //this means that the phase B high side FET can turn off as it should
    PhBHTest = true;
  }

  if(analogRead(PhCV) <= 5)
  {
    //this means that the phase C high side FET can turn off as it should
    PhCHTest = true;
  }

  delay(2);

  //check DC voltage
  digitalWrite(SDPhA, HIGH);
  digitalWrite(SDPhB, LOW);
  digitalWrite(SDPhC, LOW);

  analogWrite(PWMA, 255);
  analogWrite(PWMB, 0);
  analogWrite(PWMC, 0);

  delay(2); //delay for voltage to rise if needed (due to capacitance on phase voltage sensors, this may be insufficient)

  SysVolt = analogRead(PhAV);
  SysVoltH = SysVolt * 1.1;
  SysVoltL = SysVolt * 0.9;

  if(SysVolt <= 94)
  {
    //means that voltage is below ~5.5 V (5 V + 10%)
    Serial.println("Input voltage too low or supply not connected");
  }
  else if(SysVolt <= 225)
  {
    //means that voltage is below ~13.2 V (12 V + 10%)
    Serial.println("Input voltage low, please raise input voltage");
  }
  else
  {
    //system voltage is above ~13.2 V, should be fine to run system as gate drivers have full required supply
    Serial.println("Input voltage is sufficient");
  }

  Serial.println("Phase A PWM Voltage Read");
  Serial.print("Phase A voltage read bits: ");
  Serial.println(analogRead(PhAV)); //Phase A analog Voltage read
  Serial.print("Phase B voltage read bits: ");
  Serial.println(analogRead(PhBV)); //Phase B analog Voltage read
  Serial.print("Phase C voltage read bits: ");
  Serial.println(analogRead(PhCV)); //Phase C analog Voltage read
//  Serial.print("Neutral voltage read bits: ");
//  Serial.println(analogRead(NeuV)); //Neutral analog Voltage read

  delay(2);

  MotStateOff();

  delay(2);

  digitalWrite(SDPhA, LOW);
  digitalWrite(SDPhB, HIGH);
  digitalWrite(SDPhC, LOW);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 255); //analog write maximum should be 1023
  analogWrite(PWMC, 0);

  delay(2);

  AnalogVal = analogRead(PhBV);
  if(AnalogVal >= SysVoltH || AnalogVal <= SysVoltL)
  {
    //this means it is not within +/- 10% of previous voltage measurement
    Serial.println("Phase B out of voltage range, Phases A or B may be inoperable");
    //should maybe call measurement if bad voltage is read, can then set flag and keep program in while loop of endless SOS and error messages
  }

  Serial.println("Phase B PWM Voltage Read");
  Serial.print("Phase A voltage read bits: ");
  Serial.println(analogRead(PhAV)); //Phase A analog Voltage read
  Serial.print("Phase B voltage read bits: ");
  Serial.println(analogRead(PhBV)); //Phase B analog Voltage read
  Serial.print("Phase C voltage read bits: ");
  Serial.println(analogRead(PhCV)); //Phase C analog Voltage read
//  Serial.print("Neutral voltage read bits: ");
//  Serial.println(analogRead(NeuV)); //Neutral analog Voltage read
 
  delay(2);

  MotStateOff();

  delay(2);

  digitalWrite(SDPhA, LOW);
  digitalWrite(SDPhB, LOW);
  digitalWrite(SDPhC, HIGH);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMC, 255);

  delay(2);

  Serial.println("Phase C PWM Voltage Read");
  Serial.print("Phase A voltage read bits: ");
  Serial.println(analogRead(PhAV)); //Phase A analog Voltage read
  Serial.print("Phase B voltage read bits: ");
  Serial.println(analogRead(PhBV)); //Phase B analog Voltage read
  Serial.print("Phase C voltage read bits: ");
  Serial.println(analogRead(PhCV)); //Phase C analog Voltage read
//  Serial.print("Neutral voltage read bits: ");
//  Serial.println(analogRead(NeuV)); //Neutral analog Voltage read

  delay(2);

  MotStateOff();

  delay(2);

  digitalWrite(SDPhA, HIGH);
  digitalWrite(SDPhB, HIGH);
  digitalWrite(SDPhC, HIGH);

  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);
  analogWrite(PWMC, 255);

  delay(2);

  Serial.println("DC Voltage Read");
  Serial.print("Phase A voltage read bits: ");
  Serial.println(analogRead(PhAV)); //Phase A analog Voltage read
  Serial.print("Phase B voltage read bits: ");
  Serial.println(analogRead(PhBV)); //Phase B analog Voltage read
  Serial.print("Phase C voltage read bits: ");
  Serial.println(analogRead(PhCV)); //Phase C analog Voltage read
//  Serial.print("Neutral voltage read bits: ");
//  Serial.println(analogRead(NeuV)); //Neutral analog Voltage read

  delay(2);

  MotStateOff();

  delay(2);
} //end of TestDCV function

void LEDSOS()
{
  int MorseTime = 60; //number of millisecond per unit of Morse time (unsure what to call it)
  //function blinks LED connected to pin 13 is pattern of SOS (dot dot dot dash dash dash dot dot dot)
  //each single unit of time is 60 ms, dot = 1 unit, dash = 3 units, intra-character space = 1 unit, inter-character space = 3 units, word space = 7 units
  digitalWrite(LED0, LOW);
  delay(100);

  digitalWrite(LED0, HIGH);
  delay(MorseTime);  //dot
  digitalWrite(LED0, LOW);
  delay(MorseTime);  //intra-character space
  digitalWrite(LED0, HIGH);
  delay(MorseTime);  //dot
  digitalWrite(LED0, LOW);
  delay(MorseTime);  //intra-character space
  digitalWrite(LED0, HIGH);
  delay(MorseTime);  //dot
  digitalWrite(LED0, LOW);

  delay(MorseTime * 3);  //inter-character space

  digitalWrite(LED0, HIGH);
  delay(MorseTime * 3);  //dash
  digitalWrite(LED0, LOW);
  delay(MorseTime);  //intra-character space
  digitalWrite(LED0, HIGH);
  delay(MorseTime * 3);  //dash
  digitalWrite(LED0, LOW);
  delay(MorseTime);  //intra-character space
  digitalWrite(LED0, HIGH);
  delay(MorseTime * 3);  //dash
  digitalWrite(LED0, LOW);

  delay(MorseTime * 3);  //inter-character space

  digitalWrite(LED0, HIGH);
  delay(MorseTime);  //dot
  digitalWrite(LED0, LOW);
  delay(MorseTime);  //intra-character space
  digitalWrite(LED0, HIGH);
  delay(MorseTime);  //dot
  digitalWrite(LED0, LOW);
  delay(MorseTime);  //intra-character space
  digitalWrite(LED0, HIGH);
  delay(MorseTime);  //dot
  digitalWrite(LED0, LOW);

  delay(MorseTime * 7);  //inter-character space
} //end of LEDSOS

void MotStateOff()  //pulls all shutdown pins low to disable gate drivers, sets all phase duty cycles to 0
{
  digitalWrite(SDPhA, LOW);
  digitalWrite(SDPhB, LOW);
  digitalWrite(SDPhC, LOW);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMC, 0);
}





//Below is works in progress

/*
bool TestMotorPos() //function is intended to make sure motor is aligned correctly, will pulse phases in current hall state and make sure it doesn't move, then attempt to move to adjacent position
{
  //Since the initial hall state is known and the Hall states have been correlated with the rotor position, can use MidPos as initial angle estimate
  //First apply some low magnitude field to the direct axis (at the rotor position angle)
  //check and make sure that rotor has not switched to a new hall state
  //can then increment angle to rotate into next hall state (due to clutch in SX1 motor, can decrement to make sure motor itself does not move)
  //Record the angle when the hall state changes (and which hall state it changes to or at least the direction)
  //continue rotation and recording of angles when hall state switches
    //if all of the above match the expected rotor positions/offset, then system is working as intended
    //the angle offset can have some error due to alignment of magnetic field and actual torque production, should not be significant (?)




  //rewriting to assist in initial setup of motor 
  //Hall states are known, have initial angle estimates
  //apply field (magnitude?) to the middle angle of the current hall state
  //if the hall changes state, then halls may be incorrect
  //continue incrementing angle, record angle when halls change (and which transitions, may be reverse rotation)
  //do one complete (electrical) rotation 
  //can do negative direction to check hall transition angles
  //should the phase votlages be monitored for smoothness/hitching? how do you check for that? want to check phase currents, but need working current sensors

  ReadHalls();  //reads current hall states
  MotorDir();   //uses current hall states to set 
  Angle = MidPos;       //use MidPos from MotorDir function to have least initial movement if correct, need switching at this angle at some low magnitude
  //command for switching at this angle, may generalize MotorPos command to take in angle and PWM value
  //delay for some time, 10 ms? 100 ms? what is enough?
  ReadHalls();
  //check to see if hall state now is the same as it was when checking before
    //if the state is the same then begin incrementing angle, if it's a different state then begin recalibration
  
  //increment angle until hall state switches (and stays the same for x times in a row)
    //can the interrupts be used to tell if a state switches? Are they worth it or do I want to just hold the PWM and manually check the value again
    //when it gets to a new hall state, record which hall and the order/direction
    //may be best to have 6x hall variables, when first starting it stores the first one, then increments angle and saves second, third, so on until all 6 are filled
    //Can set order for positive gray coding, could call MotorDir function and make sure Dir = 1
    //store the ~6 angles that are recorded during transitions
*/


/*
  switch(CurrHallState)
  {
    case 1:
      //this means that hall C is high while Halls A and B are low, follow for phases 
      MotState1(PWM);
      delay(10);
      if(CurrHallState != ReadHalls())
      {
        return false;
      }
      //this means hall C is high while Halls A and B are low, want to turn on phases for adjacent position, state 5 
      MotState5(PWM);
      delay(10);
      if(ReadHalls() != 5)
      {
        return false;
      }
      //this means halls A and C are high while hall B is low, want to turn on phases for adjacent position, state 4
      MotState4(PWM);
      delay(10);
      if(ReadHalls() != 4)
      {
        return false;
      }
      //this means hall A is high while halls B and C are low, want to turn on phases for adjacent position, state 6
      MotState6(PWM);
      delay(10);
      if(ReadHalls() != 6)
      {
        return false;
      }
      //this means halls A and B are high while hall C is low, want to turn on phases for adjacent position, state 2
      MotState2(PWM);
      delay(10);
      if(ReadHalls() != 2)
      {
        return false;
      }
      //this means hall B is high while Halls A and C are low, want to turn on phases for adjacent position, state 3
      MotState3(PWM);
      delay(10);
      if(ReadHalls() != 3)
      {
        return false;
      }
    break;

    case 2: 
      //this means hall B is high while Halls A and C are low, follow for phases
      MotState2(PWM);
      delay(10);
      if(CurrHallState != ReadHalls())
      {
        return false;
      }
      //this means hall B is high while Halls A and C are low, want to turn on phases for adjacent position, state 3
      MotState3(PWM);
      delay(10);
      if(ReadHalls() != 3)
      {
        return false;
      }
      //this means halls B and C are high while Hall A is low, want to turn on phases for adjacent position, state 1
      MotState1(PWM);
      delay(10);
      if(ReadHalls() != 1)
      {
        return false;
      }
      //this means hall C is high while Halls A and B are low, want to turn on phases for adjacent position, state 5 
      MotState5(PWM);
      delay(10);
      if(ReadHalls() != 5)
      {
        return false;
      }
      //this means halls A and C are high while hall B is low, want to turn on phases for adjacent position, state 4
      MotState4(PWM);
      delay(10);
      if(ReadHalls() != 4)
      {
        return false;
      }
      //this means hall A is high while halls B and C are low, want to turn on phases for adjacent position, state 6
      MotState6(PWM);
      delay(10);
      if(ReadHalls() != 6)
      {
        return false;
      }
    break;

    case 3:
      //this means halls B and C are high while Hall A is low, follow for phases 
      MotState3(PWM);
      delay(10);
      if(CurrHallState != ReadHalls())
      {
        return false;
      }
      //this means halls B and C are high while Hall A is low, want to turn on phases for adjacent position, state 1
      MotState1(PWM);
      delay(10);
      if(ReadHalls() != 1)
      {
        return false;
      }
      //this means hall C is high while Halls A and B are low, want to turn on phases for adjacent position, state 5 
      MotState5(PWM);
      delay(10);
      if(ReadHalls() != 5)
      {
        return false;
      }
      //this means halls A and C are high while hall B is low, want to turn on phases for adjacent position, state 4
      MotState4(PWM);
      delay(10);
      if(ReadHalls() != 4)
      {
        return false;
      }
      //this means hall A is high while halls B and C are low, want to turn on phases for adjacent position, state 6
      MotState6(PWM);
      delay(10);
      if(ReadHalls() != 6)
      {
        return false;
      }
      //this means halls A and B are high while hall C is low, want to turn on phases for adjacent position, state 2
      MotState2(PWM);
      delay(10);
      if(ReadHalls() != 2)
      {
        return false;
      }
    break;

    case 4:
      //this means hall A is high while halls B and C are low, follow for phases
      MotState4(PWM);
      delay(10);
      if(CurrHallState != ReadHalls())
      {
        return false;
      }
      //this means hall A is high while halls B and C are low, want to turn on phases for adjacent position, state 6
      MotState6(PWM);
      delay(10);
      if(ReadHalls() != 6)
      {
        return false;
      }
      //this means halls A and B are high while hall C is low, want to turn on phases for adjacent position, state 2
      MotState2(PWM);
      delay(10);
      if(ReadHalls() != 2)
      {
        return false;
      }
      //this means hall B is high while Halls A and C are low, want to turn on phases for adjacent position, state 3
      MotState3(PWM);
      delay(10);
      if(ReadHalls() != 3)
      {
        return false;
      }
      //this means halls B and C are high while Hall A is low, want to turn on phases for adjacent position, state 1
      MotState1(PWM);
      delay(10);
      if(ReadHalls() != 1)
      {
        return false;
      }
      //this means hall C is high while Halls A and B are low, want to turn on phases for adjacent position, state 5 
      MotState5(PWM);
      delay(10);
      if(ReadHalls() != 5)
      {
        return false;
      }
    break;

    case 5:
      //this means halls A and C are high while hall B is low, follow for phases
      MotState5(PWM);
      delay(10);
      if(CurrHallState != ReadHalls())
      {
        return false;
      }
      //this means halls A and C are high while hall B is low, want to turn on phases for adjacent position, state 4
      MotState4(PWM);
      delay(10);
      if(ReadHalls() != 4)
      {
        return false;
      }
      //this means hall A is high while halls B and C are low, want to turn on phases for adjacent position, state 6
      MotState6(PWM);
      delay(10);
      if(ReadHalls() != 6)
      {
        return false;
      }
      //this means halls A and B are high while hall C is low, want to turn on phases for adjacent position, state 2
      MotState2(PWM);
      delay(10);
      if(ReadHalls() != 2)
      {
        return false;
      }
      //this means hall B is high while Halls A and C are low, want to turn on phases for adjacent position, state 3
      MotState3(PWM);
      delay(10);
      if(ReadHalls() != 3)
      {
        return false;
      }
      //this means halls B and C are high while Hall A is low, want to turn on phases for adjacent position, state 1
      MotState1(PWM);
      delay(10);
      if(ReadHalls() != 1)
      {
        return false;
      }
    break;

    case 6:
      //this means halls A and B are high while hall C is low, follow for phases
      MotState6(PWM);
      delay(10);
      if(CurrHallState != ReadHalls())
      {
        return false;
      }
      //this means halls A and B are high while hall C is low, want to turn on phases for adjacent position, state 2
      MotState2(PWM);
      delay(10);
      if(ReadHalls() != 2)
      {
        return false;
      }
      //this means hall B is high while Halls A and C are low, want to turn on phases for adjacent position, state 3
      MotState3(PWM);
      delay(10);
      if(ReadHalls() != 3)
      {
        return false;
      }
      //this means halls B and C are high while Hall A is low, want to turn on phases for adjacent position, state 1
      MotState1(PWM);
      delay(10);
      if(ReadHalls() != 1)
      {
        return false;
      }
      //this means hall C is high while Halls A and B are low, want to turn on phases for adjacent position, state 5 
      MotState5(PWM);
      delay(10);
      if(ReadHalls() != 5)
      {
        return false;
      }
      //this means halls A and C are high while hall B is low, want to turn on phases for adjacent position, state 4
      MotState4(PWM);
      delay(10);
      if(ReadHalls() != 4)
      {
        return false;
      }
    break;

    default:
      //motor halls are in illegal state, message to plug in Halls and to blink LED, do SOS?
      MotStateOff();
      return false;
    break;
  }
  return true;


*/
//} //end of TestMotorPos









void MakeSomeNoise()  //how do I get motor to make noise? 
{
  //don't want to change switching frequency, so must pulse magnitude of stator field (at direct angle?) at low frequencies

}
