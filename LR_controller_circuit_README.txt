The file "LR_controller_circuit.png" is my hand-drawn diagram for the linear rail controller circuit of my lathe. It is designed for stripboard, with the strips running left-to-right. The view shown is of the top of the board.

As I drew the diagram for my own benefit rather than anyone else's, and the drawing conventions are highly unconventional, some explanations may help.

The red X's are where the strip tracks on the back need to be broken.

The TMC2208 stepper driver is orientated so that the motor output pins are to the right. Mine is connected to the board via a 16-pin 'protector' unit.

The 74HC14 inverter is orientated so that pins 8-14 are to the right.

The black oblong boxes, all one point wide, are headers. You can use pins or sockets, whichever you prefer. Going down the LHS of the diagram:
- TMC2208 power input is marked '24V' but I found that 9 - 12V does fine for my motor. Connect the header to the motor driver pins directly, using stout wires rather than relying on the strips.
- The thermistor circuit is optional. If you use this part, connect a 20K thermistor to the two header pins.
- The 5V supply input is used to power the logic chips.
- The next eight pins connect to the Arduino. The Arduino pins are numbered in green and mostly* correspond to the pins used in my Arduino sketch (* Since drawing this diagram I seem to have swapped pins 3 and 4 in the sketch: pin 3 is direction, pin 4 step.)

To avoid cluttering things I've used letters to indicate where wired connections must be made:
Z, Y, X, W, V - to pins of TMC2208.
U, T - to pins 2 and 12, respectively, of 74HC14.
S - to thermistor.

- The next five pins are connected to an optional "status panel". This panel would be equipped with two indicator LEDs (one for each of the Low/High limit switches on the linear rail) and an on-off switch for (also optional) LEDs mounted inside the cutter head. Note that the 5V pin is an output from this board to the switch on the panel; the switched supply comes back on the pin below.
- The two 3-pin headers adjacent to the panel headers are for powering the Arduino (and the LCD circuit if you're using one).

The other 3-pin headers near the middle of the diagram are connected to the two photo-interrupters which serve as limit switches on the linear rail. Note that your photo-interrupters must be equipped with the appropriate resistors.

The single 3-pin header at the RHS of the diagram is for connecting optional LEDs (for example, for illuminating the inside of the cutter head). Connect R and Q next to the header to the 470 ohm resistors in the lower right corner.

I would reiterate that I originally scribbled this diagram for my own guidance only: it, like everything else, is supplied here under Creative Commons and without warranty of any kind. Please let me know if you find any errors!