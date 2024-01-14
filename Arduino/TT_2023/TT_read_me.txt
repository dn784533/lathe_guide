Latest version of the Arduino sketch that runs my turntable. TMC2209 driver pin connections are:
8 STEPPER ENABLE
9 STEPPER STEP
(the DIR on your TMC2209 will be tied either to 5V or GND depending on which way you want the motor to run)

In order to select the speed I use a rotary encoder (like this one: https://www.switchelectronics.co.uk/products/rotary-encoder-module?variant=45334952640821&currency=GBP). It has 5 pins: 5V and GND plus three data pins, which connect to the Arduino pins as follows:
A1 (15) SW
A2 (16) DT
A3 (17) CLK

To display what's happening I used an I2C OLED unit, and its pins connect to the Arduino as follows:
A4 (18) SDA
A5 (19) SCK
Both are pulled up to 5V with 10K resistors.

Finally, for maximum speed stability I included a tacho routine. This uses a photo-interruptor mounted adjacent to the turntable platter, and a small metal flange attached to the platter is carefully positioned so that it passes through the photo-interrupter on each revolution. The output from the photo-interruptor is connected to the Arduino on pin 2. This tacho is entirely optional. You turn the tacho capability on and off by pressing the control knob on the rotary encoder once the platter is up to the desired speed.

The step-pulses for the motor driver are generated using the Arduino's inbuilt PWM capability and output on pin 9. Once the Arduino timer's registers are programmed for the desired interval and set running, the pulses are generated automatically.

What follows below is particularly hideous, and by all means feel free to skip this bit - but for what it's worth, this is how I do it:

I calculate the time between PWM pulses - a 'tick' - based on the number of microseconds per revolution of the platter, divided by a constant. This constant is calculated as ((PLATTER_DIAM_MM * FULL_STEPS_REV * uSTEPS) / (CPU_FREQ_MHZ * MOTOR_PULLEY_DIAM_MM)). For my system, these values are:
PLATTER_DIAM_MM 304.2
FULL_STEPS_REV 200
uSTEPS 8 (as driver is set to 1/8 microstepping)
CPU_FREQ_MHZ 16
MOTOR_PULLEY_DIAM_MM 60
and thus the constant works out as ((304.2 * 200 * 8 ) / (16 * 60)) = 507.

Thus the 'tick' value for 33.3 rpm is 1800000 / 507 = 3550, and that for 45 rpm is 1333333 / 507 = 2629. The 'tick' value is used by the timer to determine the length of time between PWM pulses, and thus the stepping speed of the motor.

I have two timer routines. Ths first is called when the platter speed is initialised or altered by the user, and it just resets everything prior to setting an actual 'tick' value:

void initialiseTimer1()
{
cli(); // Stop any interrupts
TCCR1A = _BV(COM1A1); // Clear pin OC1A on compare match
TCCR1A |= _BV(WGM11); // Lower two bits of Wave Gen Mode set to 0x10
TCCR1B = _BV(WGM13) | _BV(WGM12); // Upper two bits of Wave Gen Mode set to 0x11 - so WGM is Fast PWM mode 14 (0x1110)
TCCR1B |= _BV(CS10); // Set CS10 bit for clock divider of 1 (ie no scaling)
ICR1 = 65535; // Default TOP
OCR1A = 32767; // Default 50% duty cycle
TIMSK1 |= _BV(OCIE1A); // Set timer compare interrupt
DDRB |= (1<<PB1); // Set PORTB pin 1 to output
sei(); // Allow global interrupts
}

The other routine is called to set the 'tick' value:

void setTimer1(unsigned long tick)
{
unsigned int regTop;
cli(); // Stop any interrupts
if (tick > 65535) // Value too big for clock divider of 1
{
regTop = (int)(tick >>= 6);
TCCR1B |= _BV(CS11);
TCCR1B |= _BV(CS10); // Set CS10 bit for clock divider of 64 (0x011)
} else { // Value will fit clock divider of 1
regTop = (int)(tick); // convert tick to int
TCCR1B &= ~(_BV(CS11));
TCCR1B |= _BV(CS10); // Set CS10 bits for clock divider of 1 (0x001)
}
ICR1 = regTop; // Set compare match register to tick duration
OCR1A = regTop >> 1; // 50% duty cycle
sei();
}

As the platter is heavy, to avoid upsetting the motor I gradually ramp up and down to the selected speed, and call this second routine repeatedly, changing the 'tick' value a little each time, until the desired speed is reached.