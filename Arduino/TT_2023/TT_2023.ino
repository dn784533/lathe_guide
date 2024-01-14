//===========================================================================================
// This program is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, either 
// version 3 of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with this program. 
// If not, see <https://www.gnu.org/licenses/>.
//===========================================================================================
// David Nelson - November 2023
// Controller for lathe platter
// 
//
// Runs on Arduino Uno. Drives stepper motor using TMC2209 driver.
// Also required: rotary encoder with push button action, I2C OLED display unit, optional
// photointerruptor circuit for tacho feedback.


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//------------------- DEFINES for OLED SCREEN -------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//------------------- DEFINES for TURNTABLE DRIVER -------------------
// PWM status codes
#define PWM_STOP 0
#define PWM_RUN 1
#define PWM_CHANGING_SPEED 2

#define PWM_ACCEL_DIVISOR 50000000.00   // Alter this value to change rate at which motor accel/decelerates
#define NUM_PLATTER_SPEEDS 4            // Number of manually selectable platter speeds
#define ROTARY_POSITIONS_PER_SPEED 5    // The number of positions the speed rotary encoder must be moved to change TT RPM
#define CPU_FREQ_MHZ 16.0               // nominal CPU speed in MHz
#define MOTOR_PULLEY_DIAM_MM 60.0       // diameter of motor pulley in mm
#define PLATTER_DIAM_MM 304.2           // diameter of platter in mm
#define FULL_STEPS_REV 200              // number of full steps the stepper motor makes in one revolution
#define uSTEPS 8.0                      // number of microsteps the driver is set to make for each full step
#define CLOCK_ERROR_FACTOR 1.000105     // Adjustment for tacho calcs if Arduino crystal runs slow

#define KP 0.4                          // PID values. Found by experiment!
#define KI 0.1

// A constant value, into which the platter speeds are divided in order to establish, for each speed, the number of 
// CPU clock cycles between (micro)steps of the motor.
#define TICK_DIVISOR ((PLATTER_DIAM_MM * FULL_STEPS_REV * uSTEPS) / (CPU_FREQ_MHZ * MOTOR_PULLEY_DIAM_MM))

// Speed adjust pins
#define PIN_SPEED_SELECT_DT  15
#define PIN_SPEED_SELECT_PB  16
#define PIN_SPEED_SELECT_CLK 17

#define ROTARY_ENCODER_PORT PINC
// Offsets for direct port manipulation
#define PIN_SPEED_SELECT_DT_OFFSET 1
#define PIN_SPEED_SELECT_PB_OFFSET 2
#define PIN_SPEED_SELECT_CLK_OFFSET 3

// Stepper control pins
#define PIN_STEPPER_EN    8
#define PIN_STEPPER_PULSE 9

// Photointerrupter input for tacho
#define PIN_TACHO 2

//------------------- DEFINES for LINEAR RAIL DRIVER -------------------



#define TICKS_IN_REPORT_CYCLE 10.000           // Timer 2 ticks are 0.01632 seconds each
#define H_ACCEL_DIVISOR 104857600             // For acceleration/deceleration of H stepper 
#define H_ACCEL_DIVIDEND 400                  // For smooth acceleration/deceleration
#define MIN_H_TICK 32
#define MAX_H_TICK 8160
#define H_STEPPER_WIND_LPCM 5
#define STARTUP_H_TICK_HALF_STEP 2000
#define H_STEPPER_FULL_STEPS_PER_REV 200 //for information only - not used!
#define H_STEPPER_MULTIPLIER 750000   // 60 * 16000000 * 100 / 10 / 400 / 32, i.e.
        // seconds in minute * clock speed in Hz * multiplier for speed (specified via serial as 100 * actual, eg 4500)
        // / stepper revs per linear cm / number of stepper half-steps per rev / number of clock cycles per 'tick'

#define STAT_STOP 0
#define STAT_FORWARD_AT_LPCM 1
#define STAT_BACK_AT_LPCM 2
#define STAT_FORWARD_WIND 3
#define STAT_BACK_WIND 4

#define SER_STAT_IDLE 0
#define SER_STAT_RECD_LT 1
#define SER_STAT_RECD_C 2
#define SER_STAT_RECD_R 3
#define SER_STAT_RECD_T 4

// Arduino pin connections. 
// Pin for thermistor circuit
#define PIN_TEMP_SENSOR 14


// Limit switch pins. A limit switch (photointerruptor, e.g HY870P) is required
// at either end of the cutter head carriage travel. The signal from each is 
// cleaned up by passing it through one gate of a 74HC14 Schmitt inverter.
#define PIN_H_STEPPER_LIMIT_HI 11
#define PIN_H_STEPPER_LIMIT_LO 10

// Horizontal cutter head travel. Stepper driver (TMC2208) pins.
#define PIN_H_STEPPER_EN 7
#define PIN_H_STEPPER_MS1 6
#define PIN_H_STEPPER_MS2 5
#define PIN_H_STEPPER_DIR 3
#define PIN_H_STEPPER_PULSE 4


//------------------- VARIABLES for TURNTABLE DRIVER -------------------
// Variables for rotary encoder / centre switch
boolean prevCLKState = false; // Rotary Encoder clock
boolean currCLKState = false;
boolean currDTState = false;  // Rotary Encoder data
boolean currPBState = false;  // Rotary Encoder central push-button
boolean prevPBState = false;
boolean tachoOn = false;      // Use tacho input to regulate speed
byte currREPositions = 0;     // Keep track of number of positions Rotary Encoder has moved in the current TT RPM speed
byte currTickIndex = 0;       // Keep track of which tick value (i.e. which TT RPM speed) is selected 
long error = 0;               // Coefficients for PI control
long cumError = 0;

// Array for displaying speeds.
const char rpm0[] PROGMEM = " 0.00";
const char rpm16[] PROGMEM = "16.66";
const char rpm22[] PROGMEM = "22.50";
const char rpm33[] PROGMEM = "33.33";
const char rpm45[] PROGMEM = "45.00";
const char * const rpm[] PROGMEM = {rpm0, rpm16, rpm22, rpm33, rpm45};

// Array for calculating speeds (platter periods in microseconds). These are used to calculate the length of PWM 'tick'.
// The first of these values cannot be set manually, but is used as a threshold when decelerating, to turn the motor off.
// The last value also cannot be set manually, but is included as a cap value to prevent motor runaway if something goes
// wrong with the tacho sensing (e.g. broken drive belt, tacho activator flange falls off etc.)
// The middle values represent the actual selectable platter speeds in us/revolution.
// Array for 
const unsigned long usPerRev[NUM_PLATTER_SPEEDS + 2] = 
{
  30000000, 3600000, 2666667, 1800000, 1333333, 1000000
};
// Array for the actual tick values, This array is populated during setup().
unsigned long tick[NUM_PLATTER_SPEEDS + 2];

// Variables for PWM admin and tacho operation
byte currPWMState = PWM_STOP;
unsigned long currTick;
float currTickFloat;    // for acceleration routine
unsigned long desiredTick;
float desiredTickFloat; // for tacho calculations
unsigned long tempTick;
volatile boolean checkPWMSpeedFlag = false;
volatile boolean tachoAdjustFlag = false;
volatile unsigned long currMicros;
unsigned long prevMicros;
unsigned long setPeriodUs;
unsigned long tachoPeriodUs;
unsigned long tachoRPM;
boolean showOLEDTTFlag = false;

//------------------- VARIABLES for LINEAR RAIL DRIVER -------------------
// Constants and variables for Steinhart-Hart equation for thermistor circuit
const float T_A = 2.097030150e-3;
const float T_B = 0.6731423305e-4;
const float T_C = 6.077684946e-7;
const float Rfix = 22000.0; // value of fixed resistor in voltage divider
float logRt;

unsigned long currHStepperTick;
unsigned long desiredHStepperTick;
unsigned long tempHStepperTick;
unsigned int desiredLPcm = 72; // default
unsigned long TTRPMx100 = 3333; // default
unsigned int HStepRShift = 4; // default
volatile long HStepper16thStepsTaken = 0;
long HStepperFullStepsTaken = 0;

unsigned int accelerationRate;
unsigned int accelerationCounter;

boolean checkHStepperSpeedFlag = false;
boolean checkHLimitSwitchesFlag = false;
boolean checkHStepsTakenFlag = false;
boolean HStepperReturningFlag = false;
//boolean updateLCDFlag = true;

unsigned int serialValue;
volatile boolean toggleHPulse = false;

// Keeps track of what the system is doing
volatile byte currHStepperState = STAT_STOP;
byte currSerialState = SER_STAT_IDLE;

// Variables for serial report
volatile unsigned int ticksThisReportCycle = 0;


//======================== SETUP ROUTINE ============================


//-------------------------------------------------------------------
void setup()
{
  //delay(1000); // work-round to avoid spurious stepper movement on power-up
  // Setup for Turntable ===========================================
  currPWMState = PWM_STOP;

  // Start serial port 
  Serial.begin(115200);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0,0);
  display.println(F("IDLE.  Tick: 0"));
  display.println(F("Set: 0         0.00"));
  display.println(F("Act: 0         0.00"));
  display.println(F("Cum.Err: 0"));
  display.display();
  
  // Set Stepper enable and pulse pins to be outputs
  pinMode(PIN_STEPPER_EN, OUTPUT);
  pinMode(PIN_STEPPER_PULSE, OUTPUT);
  // Set Rotary Encoder data, clock and push-button pins to be inputs (with pull-up)
  pinMode(PIN_SPEED_SELECT_DT, INPUT_PULLUP);
  pinMode(PIN_SPEED_SELECT_CLK, INPUT_PULLUP);
  pinMode(PIN_SPEED_SELECT_PB, INPUT_PULLUP);

  // Tacho input
  pinMode(PIN_TACHO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACHO), doTacho, FALLING);

  // Set pulse off (LOW) & enable off (HIGH)
  digitalWrite(PIN_STEPPER_PULSE, LOW);
  digitalWrite(PIN_STEPPER_EN, HIGH);
  // Fill look-up table of tick values for the TT speeds
  for (byte i = 0;i < NUM_PLATTER_SPEEDS + 2; i++) 
  {
    tick[i] = usPerRev[i] / TICK_DIVISOR;
    Serial.println(rpm[i]);
    Serial.println(tick[i]);
  }
  // Initialise tick value 
  currTick = tick[0];
  currTickFloat = tick[0];
  
  // Setup for Linear Rail ==========================================
  // Thermistor pin (analogue)
  pinMode(PIN_TEMP_SENSOR, INPUT);
  
  // Stepper limit switches
  pinMode(PIN_H_STEPPER_LIMIT_LO, INPUT);
  pinMode(PIN_H_STEPPER_LIMIT_HI, INPUT);


  // H Stepper motor outputs
  pinMode(PIN_H_STEPPER_EN, OUTPUT);
  pinMode(PIN_H_STEPPER_MS1, OUTPUT);
  pinMode(PIN_H_STEPPER_MS2, OUTPUT);
  pinMode(PIN_H_STEPPER_DIR, OUTPUT);
  pinMode(PIN_H_STEPPER_PULSE, OUTPUT);
  // set pulses, dirs off (LOW) & enable off (HIGH), and step size to full (00)
  digitalWrite(PIN_H_STEPPER_DIR, LOW);
  digitalWrite(PIN_H_STEPPER_PULSE, LOW);
  digitalWrite(PIN_H_STEPPER_EN, HIGH);
  digitalWrite(PIN_H_STEPPER_MS1, LOW);
  digitalWrite(PIN_H_STEPPER_MS2, LOW);

  initialiseTimers();
  resetHStepperTicks();
  doLPcmCalc();
}


//======================== ROUTINES for TURNTABLE DRIVER ============

//======================== TIMER ROUTINES ===========================


//-------------------------------------------------------------------
// Called when user moves rotary encoder (speed control selector)
void initialiseTimer1() 
{
  cli();                              // Stop any interrupts
  TCCR1A  = _BV(COM1A1);              // Clear pin OC1A on compare match
  TCCR1A |= _BV(WGM11);               // Lower two bits of Wave Gen Mode set to 0x10
  TCCR1B  = _BV(WGM13) | _BV(WGM12);  // Upper two bits of Wave Gen Mode set to 0x11 - so WGM is Fast PWM mode 14 (0x1110)
  TCCR1B |= _BV(CS10);                // Set CS10 bit for clock divider of 1 (ie no scaling)
  ICR1 = 65535;                       // Default TOP
  OCR1A = 32767;                      // Default 50% duty cycle
  TIMSK1 |= _BV(OCIE1A);              // Set timer compare interrupt  
  DDRB |= (1<<PB1);                   // Set PORTB pin 1 to output
  sei();                              // Allow global interrupts
}

//-------------------------------------------------------------------
// Called when accelerating or decelerating.
void setTimer1(unsigned long tick)
{
  unsigned int regTop;
  cli();                                    // Stop any interrupts
  if (tick > 65535)                         // Value too big for clock divider of 1
  {
    regTop = (int)(tick >>= 6);
    TCCR1B |= _BV(CS11);
    TCCR1B |= _BV(CS10);                    // Set CS10 bit for clock divider of 64 (0x011)
  } else {                                  // Value will fit clock divider of  1
    regTop = (int)(tick);                   // convert tick to int
    TCCR1B &= ~(_BV(CS11));
    TCCR1B |= _BV(CS10);                    // Set CS10 bits for clock divider of 1 (0x001)
   }
  ICR1 = regTop;                           // Set compare match register to tick duration
  OCR1A = regTop >> 1;                      // 50% duty cycle
  sei();      
}

//=================== INTERRUPT ROUTINES ============================


//-------------------------------------------------------------------
// Interrupt routine run when timer 1 compare interrupt fires. 
// When PWM is changing speed, this sets a flag to recalculate the tick rate.
ISR(TIMER1_COMPA_vect)
{
  // If PWM is accelerating or decelerating
  if (currPWMState == PWM_CHANGING_SPEED)
  {
    // Set flag for speed check in main loop
    checkPWMSpeedFlag=true;
  }
}

//-------------------------------------------------------------------
// Interrupt routine run when a falling edge is detected from the tacho: 
// The turntable flange has tripped the photo-interruptor.
void doTacho()
{
  if (tachoOn)
  {
    // Just get the current time, and set flag for further processing in loop
    currMicros = micros();
    tachoAdjustFlag = true;
  }
}


//================ TURNTABLE STEPPER ROUTINES =======================


//-------------------------------------------------------------------
// Compares current PWM speed with desired, and accelerates/decelerates smoothly.

void checkPWMSpeed() 
{
  // Reset flag so that this routine is not called more often than necessary
  checkPWMSpeedFlag=false;
 
  // Speed is as desired - so just jump out.
  // If tacho is being used, a very slight change of the measured speed may result in this
  // method being called when currTick == desiredTick, the change in speed not being sufficient
  // to result in a change to desiredTick. So set the status to PWM_RUN.
  if (currTick == desiredTick) 
  {
    setPWMState(PWM_RUN);
    return;
  }
  else 
  {
    tempTick = currTick;
    // ACCELERATING - need to reduce length of tick  
    if (currTick > desiredTick) 
    {
      // Reduce currTick gradually
      currTickFloat *= (1 - (currTickFloat / PWM_ACCEL_DIVISOR));
      currTick = (long) currTickFloat;
      //if (currTick == tempTick) currTick--; // in case above divisor rounds to 1
  
      // Avoid 'hunting' if above causes target to be overshot
      if (currTick < desiredTick)
      {
        currTickFloat = desiredTick;
        currTick = desiredTick;
      }
      // If we have accelerated to the desired speed, set new status
      if (currTick == desiredTick)
      {
          setPWMState(PWM_RUN);
      }
      // Call timer set method for new currTick
      setTimer1(currTick);
      
    } else
    // DECELERATING - need to increase length of tick
    {
      // Increase currTick gradually
      currTickFloat /= (1 - (currTickFloat / PWM_ACCEL_DIVISOR));
      currTick = (long) currTickFloat;
      //if (currTick == tempTick) currTick++;   // in case above divisor rounds to 1

      // Avoid 'hunting' if above causes target to be overshot
      if (currTick > desiredTick)
      {
        currTickFloat = desiredTick;
        currTick = desiredTick;
      }
      // If we have decelerated to the desired speed, set new status
      if (currTick == desiredTick)
      {
          // If TT is moving slowly enough to stop, stop.
          if (currTick >= tick[0])
          {
            disableStepper();
          } else 
          {
            // Otherwise, set status to constant-speed running
            setPWMState(PWM_RUN);
          }    
      }
      // Call timer set method for new currTick
      setTimer1(currTick);
    } 
  }
}

//-------------------------------------------------------------------
// Set PWM state and flags
void setPWMState(byte newState)
{
  currPWMState = newState;
  showOLEDTTFlag = true;
  serialReport();
}

//-------------------------------------------------------------------
void disableStepper()
{
//  TTStepperRun = 0;
  pinMode(PIN_STEPPER_PULSE, OUTPUT);             // Stop PWM mode on stepper pin
  digitalWrite(PIN_STEPPER_PULSE, LOW);           // Turn off stepper pulse 
  digitalWrite(PIN_STEPPER_EN, HIGH);             // Set enable HIGH (ie. disabled)
  setPWMState(PWM_STOP);                          // Set PWM status
  //Serial.println("Disable Stepper");
  serialReport();
}

//================ USER INPUT ROUTINES ==============================


//-------------------------------------------------------------------
// Polls the rotary encoders to see if user has selected a new platter speed.
void checkHasSpeedBeenChangedManually()
{
  // Check for rotary encoder movement for fine adjustment of speed. Done using direct
  // port manipulation for efficiency.
  currDTState  = (ROTARY_ENCODER_PORT & _BV(PIN_SPEED_SELECT_DT_OFFSET))  ? true : false;
  currCLKState = (ROTARY_ENCODER_PORT & _BV(PIN_SPEED_SELECT_CLK_OFFSET)) ? true : false;
  currPBState  = (ROTARY_ENCODER_PORT & _BV(PIN_SPEED_SELECT_PB_OFFSET))  ? true : false;
  if (currCLKState != prevCLKState && currCLKState == true) 
  {
      //Serial.println("Changing speed");
            //Serial.println(currDTState);
            //Serial.println(currCLKState);

    if (currDTState != currCLKState)
    {
      //Serial.println("In 1");
      if (currTickIndex == NUM_PLATTER_SPEEDS)                         // If at max speed already, just stay there
      {
      //Serial.println("At full speed");
        if (currREPositions < ROTARY_POSITIONS_PER_SPEED)
        {
          currREPositions++;
        }
      }
      else                                            // Increasing speed, but only if not already at max speed
      {
        if (currREPositions == ROTARY_POSITIONS_PER_SPEED)  // If Rotary Encoder position within speed setting is at its max
        {
      //Serial.println("Increasing speed");
          currTickIndex++;                            // Move up to next speed setting
          desiredTick = tick[currTickIndex];          // Set desired number of clock cycles per step
          setPeriodUs = usPerRev[currTickIndex];      // Calculate desired rotational period in us, e.g. 1800000 for 33.33 rpm
          currREPositions = 0;                        // Allow several Rotary Encoder positions for this speed
          tachoTurnOff();                             // Disable tacho processing 
          digitalWrite(PIN_STEPPER_EN, LOW);          // Set enable LOW (ie. enabled)
          setPWMState(PWM_CHANGING_SPEED); 
          initialiseTimer1();       
          setTimer1(currTick);                        // Alter tick length in timer registers 
        }
        else
        {
      Serial.println(F("Incrementing position within speed"));
          currREPositions++;                          // Allow several Rotary Encoder positions for this speed
      Serial.println(currREPositions);
        }  
      }    
    }
    else 
    {
      if (currTickIndex == 0)                         // If at min speed already, just stay there
      {
        if (currREPositions > 0)
        {
          currREPositions--;
        }
      }
      else                                            // Reducing speed, but only if already moving
      {
        if (currREPositions == 0)                     // If Rotary Encoder position within speed setting is 0
        {
          currTickIndex--;                            // Move down to next speed setting
          desiredTick = tick[currTickIndex];          // Set desired number of clock cycles per step
          setPeriodUs = usPerRev[currTickIndex];      // Calculate desired rotational period in us, e.g. 1800000 for 33.33 rpm
          currREPositions = ROTARY_POSITIONS_PER_SPEED; // Allow several Rotary Encoder positions for this speed
          tachoTurnOff();                             // Disable tacho processing 
          setPWMState(PWM_CHANGING_SPEED);        
          setTimer1(currTick);                        // Alter tick length in timer registers 
        }
        else
        {
          currREPositions--;                          // Allow several Rotary Encoder positions for this speed
        }
      }
    }
  serialReport();
  }
  prevCLKState = currCLKState;
}

//-------------------------------------------------------------------
// Check whether user has turned tacho control on or off.
void checkWhetherTachoSelected()
{
  if (currPWMState == PWM_RUN && currPBState != prevPBState)                     // button is being pushed or released
  {
    if (!currPBState)                                 // button is being pushed
    {
      if (!tachoOn)        // toggle tacho control only if not accel/decelerating
      {
        //Serial.print("Tacho ON");
        tachoTurnOn();
      }
      else
      {
        //Serial.print("Tacho OFF");
        tachoTurnOff();
      }
    }
    prevPBState = currPBState;                        // store status of button
  }  
}

//======================= TACHO ROUTINES ============================

//-------------------------------------------------------------------
// Set flag to indicate that tacho input will be processed.
void tachoTurnOn()
{
  tachoOn = true;
  cumError = 0;
  desiredTickFloat = desiredTick; // Initialise from long value for tacho calcs
  showOLEDTTFlag = true;
}

//-------------------------------------------------------------------
// Indicate that tacho input will not be processed.
void tachoTurnOff()
{
  prevMicros = 0;   // zeroise to prevent false readings if tacho subsequently reactivated
  tachoPeriodUs = 0;
  tachoOn = false;  // 
  desiredTick = tick[currTickIndex];          // Set desired number of clock cycles per step
  setPWMState(PWM_CHANGING_SPEED); 
  showOLEDTTFlag = true;
}

//-------------------------------------------------------------------
// Adjust speed if necessary, based on tacho reading
void tachoAdjust()
{
  // Only run this bit if both currMicros and prevMicros have been set. 
  if (prevMicros > 0) 
  {
    tachoPeriodUs = (long)(CLOCK_ERROR_FACTOR * (currMicros - prevMicros)); // Always use subtraction to avoid timer wraparound problem
    float tachoRPMFloat = (float)(600000000.0 / tachoPeriodUs * 10.0); // For LCD only
    tachoRPM = (long)(tachoRPMFloat + 0.5);   //Round to nearest integer
//    Serial.print(" tachoPeriodUs=");  
//    Serial.print(tachoPeriodUs); 
//    Serial.print(" tachoRPMFloat=");
//    Serial.print(tachoRPMFloat);
//    Serial.print(" tachoRPM=");
//    Serial.print(tachoRPM);

//    // Recalculate the desired tick length from the measured turntable period
//    desiredTickFloat = (float)currTick * setPeriodUs / tachoPeriodUs;
//    desiredTick = (long)desiredTickFloat;

    error = setPeriodUs - tachoPeriodUs;
    cumError += error;
    float output = ((KP * error) + (KI * cumError));
    desiredTickFloat = (1.0 + (output / setPeriodUs)) * (1.0 * desiredTickFloat);  // Update desired value for calcs
    desiredTick = (long)desiredTickFloat; //(long)(desiredTickFloat + 0.5); // Round to nearest integer
//    Serial.print(" error=");
//    Serial.print(error);
//    Serial.print(" cumError=");
//    Serial.print(cumError);
//    Serial.print(" output=");
//    Serial.print(output);
//    Serial.print(" desiredTickFloat=");
//    Serial.println(desiredTickFloat);
    // Prevent the motor from accelerating beyond the upper cap value
    if (desiredTick < tick[NUM_PLATTER_SPEEDS + 1])
    {
      desiredTick = tick[NUM_PLATTER_SPEEDS + 1];
    }
    setPWMState(PWM_CHANGING_SPEED);
  }
  prevMicros = currMicros;
  tachoAdjustFlag = false;
  showOLEDTTFlag = true;
}


//-------------------------------------------------------------------
void showOLEDTT()
{

  // Display messages depending on current state and settings.
  display.setCursor(0, 0);
  display.fillRect(0,0,128,8,SSD1306_BLACK);
  display.fillRect(30,8,98,16,SSD1306_BLACK);
  display.fillRect(54,24,74,8,SSD1306_BLACK);
  
  switch (currPWMState)
  {
    case PWM_STOP:
      display.print(F("IDLE. "));
      display.setCursor(72,0);
      display.print(F("0"));
      display.setCursor(30,8);
      display.print(F("0         0.00"));
      display.setCursor(30,16);
      display.print(F("0         0.00"));
      display.setCursor(54,24);
      display.print(F("0"));
      break;
    case PWM_CHANGING_SPEED:
      if (currTick > desiredTick)
      {
        display.print(F("ACCEL."));
      }
      else
            {
        display.print(F("DECEL."));
      }
      display.setCursor(72,0);
      display.print(currTick);
      display.setCursor(30,8);
      display.print(setPeriodUs);
      display.setCursor(84,8);
      display.print((__FlashStringHelper *)pgm_read_word(&rpm[currTickIndex]));
     break;
    case PWM_RUN:
      display.print(F("RUN.  "));
      display.setCursor(72,0);
      display.print(currTick);
      display.setCursor(30,8);
      display.print(setPeriodUs);
      display.setCursor(84,8);
      display.print((__FlashStringHelper *)pgm_read_word(&rpm[currTickIndex]));
      break;
  }
  display.setCursor(30,16);
  if (tachoOn)
  {
    display.print(tachoPeriodUs);
    display.setCursor(84,16);
    display.print(tachoRPM,2);
    display.setCursor(54,24);
    display.print(cumError);
  }
  else 
  {
    display.print(F("0         0.00")); 
    display.setCursor(54,24);
    display.print(F("0"));
  }
  display.display();
  showOLEDTTFlag = false;

}

//======================== ROUTINES for LINEAR RAIL DRIVER ============

// =============== TIMER SETUP ROUTINE ===================
// Prepare timers.
// Timer 0 ticks once for each microstep (1/2, 1/4, 1/8 or 1/16)
// pulse to drive the H stepper motor.
// Timer 2 ticks as slowly as possible (0.01632 s) and
// this tick is used as the basis for serial reporting.

void initialiseTimers()
{
  cli();                              // Stop any interrupts

  TCCR0A = 0;                         // Clear TCCR0A and TCCR0B registers
  TCCR0B = 0;
  TCNT0  = 0;                         // Clear counter
  TCCR0A |= (1 << WGM01);             // Clear timer on compare match (CTC)
  TCCR0B |= (1 << CS01);              // Set CS01 bit for clock divider of 8
  OCR0A = 255;                        // Default
  TIMSK0 |= (1 << OCIE0A);            // Set timer compare interrupt

  TCCR2A = 0;                         // Clear TCCR2A and TCCR2B registers
  TCCR2B = 0;
  TCNT2  = 0;                         // Clear counter
  TCCR2A |= (1 << WGM21);             // Clear timer on compare match (CTC)
  TCCR2B |= (1 << CS22);              // Set CS22:20 bits for clock divider of 1024
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);
  OCR2A = 255;                        // Default - longest possible delay
  TIMSK2 |= (1 << OCIE2A);            // Set timer compare interrupt
  sei();                              // Allow global interrupts
}

// Called at initialisation and also when accelerating or decelerating the H stepper motor.
void setTimer0()
{
  cli();                                      // Stop any interrupts
  if (currHStepperTick > 8160)                // Each H Stepper 'tick' is defined as a multiple of 32 clock cycles.
  {                                           // Longest delay possible with 8-bit counter is 261120 ( = 1024 * 255)
    currHStepperTick = 8160;                  // - thus longest 'tick' possible is 8160 (= 261120 / 32)
  }
  if (currHStepperTick >= 2016)
  {
    TCCR0B |= (1 << CS02);
    TCCR0B &= ~(1 << CS01);
    TCCR0B |= (1 << CS00);                    // Set CS02:00 bits for clock divider of 1024 ( = 32 * 32)
    OCR0A = (int)(currHStepperTick >> 5);     // Set compare match register to tick specified 1/32 of currHStepperTick
  } else if (currHStepperTick >= 504) {
    TCCR0B |= (1 << CS02);
    TCCR0B &= ~(1 << CS01);
    TCCR0B &= ~(1 << CS00);                   // Set CS02:00 bits for clock divider of 256 ( = 8 * 32)
    OCR0A = (int)(currHStepperTick >> 3);     // Set compare match register to tick specified 1/8 of currHStepperTick
  } else if (currHStepperTick >= 64) {
    TCCR0B &= ~(1 << CS02);
    TCCR0B |= (1 << CS01);
    TCCR0B |= (1 << CS00);                    // Set CS02:00 bits for clock divider of 64 ( = 2 * 32)
    OCR0A = (int)(currHStepperTick >> 1);     // Set compare match register to tick specified 1/2 of currHStepperTick
  } else {
    TCCR0B &= ~(1 << CS02);
    TCCR0B |= (1 << CS01);
    TCCR0B &= ~(1 << CS00);                   // Set CS02:00 bits for clock divider of 8 ( = 1/4 * 32)
    OCR0A = (int)(currHStepperTick << 2);     // Set compare match register to tick specified 4 * currHStepperTick

  }
  sei();                                      // Allow global interrupts
}

// =============== THERMISTOR CALCULATIONS ==============
float calcTemp(int v0)
{
  float TC;
  logRt = log(Rfix * (1023.0 / (float)v0 - 1));
  TC = (1.0 / (T_A + T_B * logRt + T_C * logRt * logRt * logRt)) - 273.15;
  return TC;
}
// =============== INTERRUPT ROUTINES ===================

// Interrupt routine run when timer 0 compare interrupt fires.
ISR(TIMER0_COMPA_vect)
{
  // Action if stepper is running
  if (currHStepperState != STAT_STOP)
  {
    if (toggleHPulse)
    {
      digitalWrite(PIN_H_STEPPER_PULSE, LOW);
      toggleHPulse = false;
    } else {
      digitalWrite(PIN_H_STEPPER_PULSE, HIGH);
      toggleHPulse = true;
      // Set flag to check limit switches
      checkHLimitSwitchesFlag = true;
      // Adjust number of steps taken
      // If moving forward (towards centre)
      if (currHStepperState == STAT_FORWARD_AT_LPCM || currHStepperState == STAT_FORWARD_WIND) 
      {
        HStepper16thStepsTaken += 1 << (4 - HStepRShift);
        // If 'Homing', and home reached
        if (HStepperReturningFlag && HStepper16thStepsTaken >= 0)
        {}
      }
      // If moving back (away from centre)
      else if (currHStepperState == STAT_BACK_AT_LPCM || currHStepperState == STAT_BACK_WIND) 
       {
        HStepper16thStepsTaken -= 1 << (4 - HStepRShift);
        // If 'Homing', and home reached
        if (HStepperReturningFlag && HStepper16thStepsTaken <= 0)
        {}  
      }
        //checkHStepsTakenFlag = true;
      // Set flag to update LCD
      //updateLCDFlag = true;
      // If the H stepper is accelerating or decelerating
      if (currHStepperTick != desiredHStepperTick)
      {
        // Set flag for speed check in main loop
        checkHStepperSpeedFlag = true;
      }
    }
  }
}


//-------------------------------------------------------------------
// Interrupt routine run when timer 2 compare interrupt fires.
ISR(TIMER2_COMPA_vect)
{
  // Count time to next serial port message
  ticksThisReportCycle++;
}

// =============== LIMIT SWITCH ROUTINES ===================
// Photointerrupters return HIGH if CLEAR, LOW if BLOCKED.
// This is inverted via 74HC14, so input to Arduino is LOW if CLEAR, HIGH if BLOCKED.
// These routines return FALSE if CLEAR, TRUE if BLOCKED.
boolean limitHLo()
{
  return (digitalRead(PIN_H_STEPPER_LIMIT_LO) == LOW) ? false : true;
}
//-------------------------------------------------------------------
boolean limitHHi()
{
  return (digitalRead(PIN_H_STEPPER_LIMIT_HI) == LOW) ? false : true;
}

// =============== HORIZONTAL STEPPER ROUTINES ===================

// Sets angle through which horizontal stepper moves on each pulse. The faster the speed, the fewer
// ticks in a half-step, and the larger the angle required.
void setHStepperMS()
{
  if (HStepRShift == 4)
  {
    // sixteenth step
    // adjust MS1, MS2 HH
    digitalWrite(PIN_H_STEPPER_MS1, HIGH);
    digitalWrite(PIN_H_STEPPER_MS2, HIGH);
  }
  else if (HStepRShift == 3)
  {
    // eighth step
    // adjust MS1, MS2 LL
    digitalWrite(PIN_H_STEPPER_MS1, LOW);
    digitalWrite(PIN_H_STEPPER_MS2, LOW);
  }
  else if (HStepRShift == 2)
  {
    // quarter step
    // adjust MS1, MS2 LH
    digitalWrite(PIN_H_STEPPER_MS1, LOW);
    digitalWrite(PIN_H_STEPPER_MS2, HIGH);
  }
  else
  {
    // half step
    // adjust MS1, MS2 HL
    digitalWrite(PIN_H_STEPPER_MS1, HIGH);
    digitalWrite(PIN_H_STEPPER_MS2, LOW);
  }
}


//-------------------------------------------------------------------
// Set H stepper state and flags
void setHStepperState(byte newState)
{
  currHStepperState = newState;
  //updateLCDFlag = true;
}

//-------------------------------------------------------------------
// Compares current H stepper speed with desired, and accelerates/decelerates smoothly.
// The H stepper varies the length of the tick on timer 0, the pulses
// to the H stepper being made once per tick.
// The tick is lengthened or shortened gradually to avoid jerky acceleration.
void checkHStepperSpeed()
{
  // Reset flag so that this routine is not called more often than necessary
  checkHStepperSpeedFlag = false;

  // Speed is as desired - so just jump out.
  if (currHStepperTick == desiredHStepperTick) return;
  else {
    tempHStepperTick = currHStepperTick;
    // ACCELERATING - need to reduce length of tick per motor step
    if (currHStepperTick > desiredHStepperTick)
    {
      if (accelerationCounter==accelerationRate) 
      {
        accelerationCounter = 0;
        // Reduce currHStepperTick gradually to avoid motor troubles
        currHStepperTick --;
        //if (currHStepperTick == tempHStepperTick) currHStepperTick--; // in case above divisor rounds to 1
  
        // See if a 'gear change' is required
        if (currHStepperTick < MIN_H_TICK)
        {
          if (HStepRShift > 1)
          {
            currHStepperTick <<= 1;
            desiredHStepperTick <<= 1;
            HStepRShift --;
            setHStepperMS();
          } else {
            // Desired tick is too fast: set to allowed minimum tick-duration
            currHStepperTick = MIN_H_TICK;
            desiredHStepperTick = MIN_H_TICK;
          }
        }
        // Avoid 'hunting' if above causes target to be overshot
        if (currHStepperTick < desiredHStepperTick)
        {
          currHStepperTick = desiredHStepperTick;
        }
        // Calculate new acceleration rate to ensure smooth acceleration
        accelerationRate=H_ACCEL_DIVIDEND / currHStepperTick;
      } else accelerationCounter++; 
      // Call timer set method for new currHStepperTick
      setTimer0();

    } else
      // DECELERATING - need to increase length of tick per motor step
    {
      if (accelerationCounter==accelerationRate) 
      {
        accelerationCounter = 0;
        // Increase currHStepperTick gradually to avoid motor troubles
        currHStepperTick ++;
        //if (currHStepperTick == tempHStepperTick) currHStepperTick++;   // in case above divisor rounds to 1
  
        // See if a 'gear change' is required
        if (currHStepperTick > 2 * MIN_H_TICK)
        {
          if (HStepRShift < 4)
          {
            currHStepperTick >>= 1;
            desiredHStepperTick >>= 1;
            HStepRShift ++;
            setHStepperMS();
          }
        }
        if (currHStepperTick > MAX_H_TICK)
        {
          // Desired tick is too slow: set to allowed maximum tick-duration
          currHStepperTick = MAX_H_TICK;
          desiredHStepperTick = MAX_H_TICK;
        }
        // Avoid 'hunting' if above causes target to be overshot
        if (currHStepperTick > desiredHStepperTick)
        {
          currHStepperTick = desiredHStepperTick;
        }
        // Calculate new acceleration rate to ensure smooth acceleration
        accelerationRate=H_ACCEL_DIVIDEND / currHStepperTick;
      } else accelerationCounter++;
      // Call timer set method for new currHStepperTick
      setTimer0();
    }
  }
}

//-------------------------------------------------------------------
void checkHLimitSwitches()
{
  // Check limit switches before allowing horizontal stepper to proceed.
  switch (currHStepperState)
  {
    case STAT_FORWARD_AT_LPCM:
    case STAT_FORWARD_WIND:
      if (limitHHi())
      {
        // stop stepper
        disableHStepper();
      }
      break;
    case STAT_BACK_AT_LPCM:
    case STAT_BACK_WIND:
      if (limitHLo())
      {
        // stop stepper
        disableHStepper();
      }
      break;
  }
  // No need to call again.
  checkHLimitSwitchesFlag = false;
}

//-------------------------------------------------------------------
//void checkHStepsTaken()
//{
//  if (currHStepperState == STAT_FORWARD_AT_LPCM || currHStepperState == STAT_FORWARD_WIND) 
//    HStepper16thStepsTaken += (16 / HStepRShift);
//  else if (currHStepperState == STAT_BACK_AT_LPCM || currHStepperState == STAT_BACK_WIND) 
//    HStepper16thStepsTaken -= (16 / HStepRShift);
//  checkHStepsTakenFlag = false;
//}


//-------------------------------------------------------------------
// After a full stop of the H stepper, reset to default speeds.
void resetHStepperTicks()
{
  HStepperReturningFlag = false;
  HStepRShift = 4;
  currHStepperTick = STARTUP_H_TICK_HALF_STEP >> 3;
  setHStepperMS();
}

//-------------------------------------------------------------------
// Reset H stepper counter to zero.
void zeroCounter()
{
  HStepper16thStepsTaken = 0;
}

//-------------------------------------------------------------------
// Set or clear PIN_STEPPER_DIR for direction of stepper.
// Clear PIN_STEPPER_EN to enable stepper.
void setDirForward()
{
  if (!limitHHi())
  {
    digitalWrite(PIN_H_STEPPER_EN, LOW);                               // Set enable LOW (ie. enabled)
    digitalWrite(PIN_H_STEPPER_DIR, LOW);                              // Reset direction
  }
}

//-------------------------------------------------------------------
void setDirBack()
{
  if (!limitHLo())
  {
    digitalWrite(PIN_H_STEPPER_EN, LOW);                               // Set enable LOW (ie. enabled)
    digitalWrite(PIN_H_STEPPER_DIR, HIGH);                              // Reset direction
  }
}

//-------------------------------------------------------------------
void disableHStepper()
{
  digitalWrite(PIN_H_STEPPER_PULSE, LOW);                             // Turn off stepper pulse
  digitalWrite(PIN_H_STEPPER_EN, HIGH);                               // Set enable HIGH (ie. disabled)
  digitalWrite(PIN_H_STEPPER_DIR, LOW);                               // Reset direction
  setHStepperState(STAT_STOP);                                        // Set H stepper status
  serialReport();                                                     // Immediately signal via serial port
}

//-------------------------------------------------------------------
void doLPcmCalc()
{
  if (currHStepperState == STAT_FORWARD_AT_LPCM || currHStepperState == STAT_BACK_AT_LPCM) 
    // Calculate tick duration based on desired TT speed
    desiredHStepperTick = (H_STEPPER_MULTIPLIER * desiredLPcm / TTRPMx100) >> HStepRShift;
  else if (currHStepperState == STAT_FORWARD_WIND || currHStepperState == STAT_BACK_WIND)
    desiredHStepperTick = (H_STEPPER_MULTIPLIER * H_STEPPER_WIND_LPCM / 7800) >> HStepRShift;
}

// =============== SERIAL DATA ROUTINES ===================
void processSerial()
{
  byte sb = Serial.read();
  // If the current character is a digit, add it to value so far received.
  if (isdigit(sb))
  {
    serialValue *= 10;
    serialValue += sb - '0';
  }

  // Current character is not a digit: signals change of state
  else
  {
    // Do what we have to with the number just completed
    processPreviousNumber();
    // And set the new state depending on the new character
    switch (sb)
    {
      case 'E': // Emergency stop!
        // Immediately puts H stepper into freewheel, allowing power to be
        // disconnected without danger of back EMF damaging the drivers.
        disableHStepper();
        resetHStepperTicks();
        currSerialState = SER_STAT_IDLE;
        break;
      case 'C':
        currSerialState = SER_STAT_RECD_C;
        break;
      case 'R':
        currSerialState = SER_STAT_RECD_R;
        break;
      case 'T':
        currSerialState = SER_STAT_RECD_T;
        break;
      default:
        currSerialState = SER_STAT_IDLE;
        break;
    }
  }
}


void processPreviousNumber()
{
  switch (currSerialState)
  {
    // A command is coming in
    case SER_STAT_RECD_C:
      switch (serialValue)
      {
        // Stop stepper motor
        case 0:
          resetHStepperTicks();
          disableHStepper();
          break;
        // Start motor moving towards centre at rate specified by LPcm
        case 1:
          resetHStepperTicks();
          setHStepperState(STAT_FORWARD_AT_LPCM);  
          doLPcmCalc();
          setDirForward();
          setTimer0();
          break;
        // Start motor moving away from centre at rate specified by LPcm
        case 2:
          resetHStepperTicks();
          setHStepperState(STAT_BACK_AT_LPCM);  
          doLPcmCalc();
          setDirBack();
          setTimer0();
          break;
        // Start motor winding towards centre at full speed
        case 3:
          resetHStepperTicks();
          setHStepperState(STAT_FORWARD_WIND);  
          doLPcmCalc();
          setDirForward();
          setTimer0();
          break;
        // Start motor winding away from centre at full speed
        case 4:
          resetHStepperTicks();
          setHStepperState(STAT_BACK_WIND);  
          doLPcmCalc();
          setDirBack();
          setTimer0();
          break;
        // return to 'Home' : set direction depending
        // on step counter value, then proceed as for winding
        case 5:
          resetHStepperTicks();
          if (HStepperFullStepsTaken <= 0)
          {
            setHStepperState(STAT_FORWARD_WIND);  
            doLPcmCalc();
            setDirForward();
          } else {
            setHStepperState(STAT_BACK_WIND);  
            doLPcmCalc();
            setDirBack();
          }
          HStepperReturningFlag = true;
          setTimer0();
          break;
        // Zeroise step counter
        case 6:
          resetHStepperTicks();
          disableHStepper();
          zeroCounter();
          break;
      }
      break;
    // A TT RPM value is coming in
    case SER_STAT_RECD_R:
      TTRPMx100 = serialValue;                                     // Value in RPM*100 sent from PC
      doLPcmCalc();
      setTimer0();
      break;

    // An LPcm value is coming in
    case SER_STAT_RECD_T:
      desiredLPcm = serialValue;
      doLPcmCalc();
      setTimer0();
      break;
   }
  serialValue = 0;
}



//====================== OUTPUT ROUTINES ============================

//-------------------------------------------------------------------
void serialReport()
{
    Serial.print("<");
    Serial.print(HStepperFullStepsTaken);
    Serial.print("L");
    Serial.print(limitHLo());
    Serial.print("H");
    Serial.print(limitHHi());
    Serial.print(">");
//  Serial.print(CPU_DIVIDEND); Serial.print(" ");
//  Serial.print(MOTOR_PULLEY_DIAM_MM);Serial.print(" ");
//  Serial.print(PLATTER_DIAM_MM); Serial.print(" "); 
//  Serial.print(FULL_STEPS_REV); Serial.print(" "); 
//  Serial.print(uSTEPS); Serial.print(" "); 
//   Serial.print(CPU_DIVIDEND * MOTOR_PULLEY_DIAM_MM); Serial.print(" ");
//   Serial.print(PLATTER_DIAM_MM * FULL_STEPS_REV*uSTEPS); Serial.print(" "); 
  
//  Serial.print(currPWMState);
//  Serial.print(" ");
//  Serial.print(desiredTick);
//  Serial.print(" ");
//  Serial.print(currTick);
//  Serial.print(" ");
//
//  Serial.print(tachoPeriodUs); 
//    Serial.print(" ");  
  ticksThisReportCycle = 0;
}

void showOLEDLR()
{
  // Display messages depending on current state and settings.
  display.setCursor(0, 36);
  display.fillRect(0,36,128,24,SSD1306_BLACK);
  switch (currHStepperState)
  {
    case STAT_STOP:
      display.print(F("IDLE"));
      break;
    case STAT_FORWARD_AT_LPCM:
      display.print(F("FWD"));
      break;
    case STAT_BACK_AT_LPCM:
      display.print(F("BACK"));
    case STAT_FORWARD_WIND:
      display.print(F(">>"));
    case STAT_BACK_WIND:
      display.print(F("<<"));
      break;
  }
  // Show number of H Stepper steps taken
  display.setCursor(30, 36);
  display.print(HStepperFullStepsTaken);

  // Display home indicator
  display.setCursor(0, 44);
  if (HStepperFullStepsTaken == 0) {
    display.print(F("HOME ")); 
  }
  // Display temperature
  display.setCursor(0, 52);
  display.print(calcTemp(analogRead(PIN_TEMP_SENSOR)),1);
  display.write(223);  
  
  display.display();
}
//====================== M A I N   L O O P ==========================

//-------------------------------------------------------------------
void loop() 
{
  checkHasSpeedBeenChangedManually();
  checkWhetherTachoSelected();
  if (checkPWMSpeedFlag) checkPWMSpeed();
  if (tachoAdjustFlag) tachoAdjust();
  if (showOLEDTTFlag) showOLEDTT();

//
//
//    // Calculate number of FULL steps taken.
//  HStepperFullStepsTaken = HStepper16thStepsTaken >> 4;
//  // If returning to zero position, check whether zero position
//  // found, guarding against overshoot.
//  if (
//    HStepperReturningFlag &&
//    (((currHStepperState == STAT_BACK_WIND) && (HStepperFullStepsTaken <= 0))
//     || ((currHStepperState == STAT_FORWARD_WIND) && (HStepperFullStepsTaken >= 0)))
//  )
//  {
//    // Stop motor and cancel "returning" flag.
//    HStepperReturningFlag = false;
//    resetHStepperTicks();
//    disableHStepper();
//  }
//
//
//  while (Serial.available()) processSerial();
//  if (checkHStepperSpeedFlag) checkHStepperSpeed();
//  //if (checkHStepsTakenFlag) checkHStepsTaken();
//  if (checkHLimitSwitchesFlag) checkHLimitSwitches();
//
//
//  // Serial report back to PC
//  if (ticksThisReportCycle >= TICKS_IN_REPORT_CYCLE) 
//  {
//    showOLEDLR();
//    serialReport();
//  }
}
//===================== E N D   L I S T I N G =======================
