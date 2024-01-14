#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

//------------------- DEFINES for OLED SCREEN -------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define i2c_Address 0x3C 
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define OLED_LINE_HEIGHT 12

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
#define PIN_SPEED_SELECT_DT  16
#define PIN_SPEED_SELECT_PB  15
#define PIN_SPEED_SELECT_CLK 17

#define ROTARY_ENCODER_PORT PINC
// Offsets for direct port manipulation
#define PIN_SPEED_SELECT_DT_OFFSET 2
#define PIN_SPEED_SELECT_PB_OFFSET 1
#define PIN_SPEED_SELECT_CLK_OFFSET 3

// Stepper control pins
#define PIN_STEPPER_EN    8
#define PIN_STEPPER_PULSE 9

// Photointerrupter input for tacho
#define PIN_TACHO 2


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
const char rpm0[] PROGMEM = " 0.0000";
const char rpm16[] PROGMEM = "16.6666";
const char rpm22[] PROGMEM = "22.5000";
const char rpm33[] PROGMEM = "33.3333";
const char rpm45[] PROGMEM = "45.0000";
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
//unsigned long tachoRPM;
float tachoRPMFloat;
boolean showOLEDTTFlag = true;


//======================== SETUP ROUTINE ============================


//-------------------------------------------------------------------
void setup()
{
  // Setup for Turntable ===========================================
  currPWMState = PWM_STOP;

  // Start serial port 
  Serial.begin(115200);
 
  
  delay(250); // wait for the OLED to power up
  display.begin(i2c_Address, true); // Address 0x3C default
  // Show initial display buffer contents on the screen --
  // the library initialises this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
 
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE); 
  display.setCursor(0,0);
  display.print(F("IDLE."));
  display.setCursor(0, OLED_LINE_HEIGHT);
  display.print(F("Tick:"));
  display.setCursor(0, 2 * OLED_LINE_HEIGHT);
  display.print(F("Set:"));
  display.setCursor(0, 3 * OLED_LINE_HEIGHT);
  display.print(F("Act:"));
  display.setCursor(0, 4 * OLED_LINE_HEIGHT);
  display.print(F("Cum.Err:"));
 
  
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
    tachoRPMFloat = (float)(600000000.0 / tachoPeriodUs / 10.0); // For LCD only
    //tachoRPM = (long)(tachoRPMFloat + 0.5);   //Round to nearest integer
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
  display.fillRect(0,0,128, OLED_LINE_HEIGHT,SH110X_BLACK);
  display.fillRect(30, OLED_LINE_HEIGHT,98,3 * OLED_LINE_HEIGHT,SH110X_BLACK);
  display.fillRect(54,4 * OLED_LINE_HEIGHT,74, OLED_LINE_HEIGHT,SH110X_BLACK);
  
  switch (currPWMState)
  {
    case PWM_STOP:
      display.print(F("IDLE. "));
      display.setCursor(84, OLED_LINE_HEIGHT);
      display.print(0);
      display.setCursor(30, 2 * OLED_LINE_HEIGHT);
      display.print(F("0         0.0000"));
      display.setCursor(30, 3 * OLED_LINE_HEIGHT);
      display.print(F("0         0.0000"));
      display.setCursor(54, 4 * OLED_LINE_HEIGHT);
      display.print(0);
      break;
    case PWM_CHANGING_SPEED:
      if (currTick > desiredTick)
      {
        display.print(F("ACCELERATING."));
      }
      else
            {
        display.print(F("DECELERATING."));
      }
      display.setCursor(84, OLED_LINE_HEIGHT);
      display.print(currTick);
      display.setCursor(30, 2 * OLED_LINE_HEIGHT);
      display.print(setPeriodUs);
      display.setCursor(84, 2 * OLED_LINE_HEIGHT);
      display.print((__FlashStringHelper *)pgm_read_word(&rpm[currTickIndex]));
     break;
    case PWM_RUN:
      display.print(F("RUNNING."));
      display.setCursor(84, OLED_LINE_HEIGHT);
      display.print(currTick);
      display.setCursor(30, 2 * OLED_LINE_HEIGHT);
      display.print(setPeriodUs);
      display.setCursor(84, 2 * OLED_LINE_HEIGHT);
      display.print((__FlashStringHelper *)pgm_read_word(&rpm[currTickIndex]));
      break;
  }
  display.setCursor(30,3 * OLED_LINE_HEIGHT);
  if (tachoOn)
  {
    display.print(tachoPeriodUs);
    display.setCursor(84, 3 * OLED_LINE_HEIGHT);
    display.print(tachoRPMFloat,4);
    display.setCursor(54, 4 * OLED_LINE_HEIGHT);
    display.print(cumError);
  }
  else 
  {
    display.print(F("0         0.0000")); 
    display.setCursor(54, 4 * OLED_LINE_HEIGHT);
    display.print(0);
 }
  display.write(32);
  display.write(229);
  display.write(115);
  display.display();
  showOLEDTTFlag = false;

}


//====================== OUTPUT ROUTINES ============================

//-------------------------------------------------------------------
void serialReport()
{

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

}
//===================== E N D   L I S T I N G =======================
