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
// David Nelson - February 2021
// Controller for disc recorder
// 
//
// Runs on Arduino Uno. Drives horizontal linear rail motor using TMC2208 driver.
// Also required: two photointerrupter circuits, 74HC14 inverter, 74HC595 LCD driver, HD44780 LCD unit,
// optional thermistor circuit.
// 
// The Arduino receives signals via USB from the PC running 'Vinyl Burn' software, moves
// the horizontal stepper as requested, and sends status reports back to the PC for
// display in the Vinyl Burn window.

// This library is required. it is available from https://www.arduino.cc/reference/en/libraries/enableinterrupt/
//#include <EnableInterrupt.h>

#include <SPI.h>
#include <LiquidCrystal.h>


#define TICKS_IN_REPORT_CYCLE 2.000           // Timer 2 ticks are 0.01632 seconds each
#define H_ACCEL_DIVISOR 104857600             // For acceleration/deceleration of H stepper 
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

// LCD pins. Requires 74HC595 IC.
#define PIN_LCD_CLOCK 13
#define PIN_LCD_DATA 11
#define PIN_LCD_LATCH 10

// Limit switch pins. A limit switch (photointerruptor, e.g HY870P) is required
// at either end of the cutter head carriage travel. The signal from each is 
// cleaned up by passing it through one gate of a 74HC14 Schmitt inverter.
#define PIN_H_STEPPER_LIMIT_HI 9
#define PIN_H_STEPPER_LIMIT_LO 8

// Horizontal cutter head travel. Stepper driver (TMC2208) pins.
#define PIN_H_STEPPER_EN 7
#define PIN_H_STEPPER_MS1 6
#define PIN_H_STEPPER_MS2 5
#define PIN_H_STEPPER_DIR 3
#define PIN_H_STEPPER_PULSE 4

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

int currTickAdj = 0;

unsigned int accelerationRate;
unsigned int accelerationDividend=400;
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

// LCD
LiquidCrystal lcd(PIN_LCD_LATCH);

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

// =============== MAIN SETUP ROUTINE ===================
// Main initialisation sequence
void setup()
{
  // Thermistor pin (analogue)
  pinMode(PIN_TEMP_SENSOR, INPUT);
  
  // Stepper limit switches
  pinMode(PIN_H_STEPPER_LIMIT_LO, INPUT);
  pinMode(PIN_H_STEPPER_LIMIT_HI, INPUT);

  // LCD outputs
  pinMode(PIN_LCD_LATCH, OUTPUT);
  pinMode(PIN_LCD_CLOCK, OUTPUT);
  pinMode(PIN_LCD_DATA, OUTPUT);

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

  lcd.begin(16, 2);
  initialiseTimers();
  resetHStepperTicks();
  doLPcmCalc();
  // Start serial port
  Serial.begin(115200);

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
        accelerationRate=accelerationDividend / currHStepperTick;
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
        accelerationRate=accelerationDividend / currHStepperTick;
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



void serialReport()
{
  
    Serial.print("<");
    Serial.print(HStepperFullStepsTaken);
    Serial.print("L");
    Serial.print(limitHLo());
    Serial.print("H");
    Serial.print(limitHHi());
    Serial.print(">");
  
  /*
    Serial.print("HS Status: ");
    Serial.print(currHStepperState);
    Serial.print(" RPM100: ");
    Serial.print(desiredTTRPMx100);
    Serial.print(" desLPcm: ");
    Serial.print(desiredLPcm);
    Serial.print(" HStRt: ");
    Serial.print(HStepRShift);
    Serial.print(" currHStTick: ");
    Serial.print(currHStepperTick);
    Serial.print(" desHStTick: ");
    Serial.println(desiredHStepperTick);
  
  Serial.print("H: ");
  Serial.print(currHStepperState);
  Serial.print("  Des/Curr: ");
  Serial.print(desiredHStepperTick);
  Serial.print("/");
  Serial.print(currHStepperTick);
  Serial.print(" RShift: ");
  Serial.print(HStepRShift);
  Serial.print(" 16ths: ");
  Serial.print(HStepper16thStepsTaken);
  Serial.print(" Full: ");
  Serial.print(HStepperFullStepsTaken);
  Serial.print(" L:");
  Serial.print(limitHLo());
  Serial.print(" H:");
  Serial.println(limitHHi());
  */
  ticksThisReportCycle = 0;
}
// =============== DISPLAY ROUTINES ===================

void showLCD()
{
  // Display messages depending on current state and settings.
  lcd.setCursor(0, 0);
  switch (currHStepperState)
  {
    case STAT_STOP:
      lcd.print("IDLE ");
      break;
    case STAT_FORWARD_AT_LPCM:
      lcd.print("FWD  ");
      break;
    case STAT_BACK_AT_LPCM:
      lcd.print("BACK ");
    case STAT_FORWARD_WIND:
      lcd.print(">>   ");
    case STAT_BACK_WIND:
      lcd.print("<<   ");
      break;
  }
  // Show number of H Stepper steps taken
  lcd.setCursor(5, 0);
  lcd.print(HStepperFullStepsTaken);
  lcd.print("     ");

  // Display home indicator
  lcd.setCursor(0, 1);
  if (HStepperFullStepsTaken == 0) {
    lcd.print("HOME "); 
  } else {
    lcd.print("     ");
  }
  // Display temperature
  lcd.print(calcTemp(analogRead(PIN_TEMP_SENSOR)));
  lcd.print((char)223);
  //updateLCDFlag = false;

}
// =============== MAIN ITERATION ===================

void loop()
{
  // Calculate number of FULL steps taken.
  HStepperFullStepsTaken = HStepper16thStepsTaken >> 4;
  // If returning to zero position, check whether zero position
  // found, guarding against overshoot.
  if (
    HStepperReturningFlag &&
    (((currHStepperState == STAT_BACK_WIND) && (HStepperFullStepsTaken <= 0))
     || ((currHStepperState == STAT_FORWARD_WIND) && (HStepperFullStepsTaken >= 0)))
  )
  {
    // Stop motor and cancel "returning" flag.
    HStepperReturningFlag = false;
    resetHStepperTicks();
    disableHStepper();
  }


  while (Serial.available()) processSerial();
  if (checkHStepperSpeedFlag) checkHStepperSpeed();
  //if (checkHStepsTakenFlag) checkHStepsTaken();
  if (checkHLimitSwitchesFlag) checkHLimitSwitches();


  // Serial report back to PC
  if (ticksThisReportCycle >= TICKS_IN_REPORT_CYCLE) 
  {
    showLCD();
    serialReport();
  }
  //if (updateLCDFlag) showLCD();
}
