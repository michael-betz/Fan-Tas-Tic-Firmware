# Fan-Tas-Tic-Controller
Controller for Pinball machines based on an TM4C123G LaunchPad™ Evaluation Kit, compatible with the [Mission Pinball API](https://missionpinball.com/).

## Hardware features
 * 8x8 Switch matrix inputs
 * 12 onboard drivers for solenoids, 4 of them can do hardware PWM (> 100 kHz)
 * 4 x I2C channels for extension boards
 * 2 x SPI channels for running [WS2811 / WS2812 LED strings](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR0.TRC0.H0.Xled+strand+ws2811.TRS5&_nkw=led+strand+ws2811&_sacat=0)
 * In- / Outputs can be easily and cheaply added with [PCF8574](http://www.ti.com/product/pcf8574) I2C GPIO extenders (check eBay for [cheap I/O modules](http://www.ebay.com/sch/i.html?_sacat=0&_nkw=i2c+expander&_frs=1))
 * Super fast USB virtual serial connection to host PC
 * KiCad PCB files available (soon), no tiny SMD components, can be easily assembled by hand

## Software features 
 * Software can handle up to 320 In- / Outputs
 * All Outputs support 4 bit PWM with > 125 Hz (using [binary code modulation](http://www.batsocks.co.uk/readme/art_bcm_1.htm))
 * All inputs are debounced and read 333 times per second
 * Software easily extendable by running [FreeRTOS](http://www.freertos.org/)

# Digital inputs / outputs

We have 4 I2C channels where we can connect 8 x PC8574 each. The addresses of the chips need to be configured from 0x40 - 0x47.

To use an PC8574 as input we have to _write_ a 0xFF to it (to prevent the open drain outputs pulling the pin low). 
Then the pin state can be acquired with a 1 byte read transaction over I2C.

So all I2C extenders (no matter if used as in- or output) can be read in bulk. 
I2C is fast enough to read all 32 of them (on 4 channels in parallel) with 625 Hz repetition rate. 
Reading an empty address will give a NO_ACK read error, which will be silently ignored.



## Debouncing

The taskDebouncer() is running with 333 Hz, reading all switches (matrix and I2C) into an array
 * Switch Matrix (SM) is read in foreground while I2C transactions happen in background (TI I2Cm driver)
 * When both reads are finished, start the Debouncing routine
 * If a bit changes, it increments a 2 bit [vertical counter](http://www.compuphase.com/electronics/debouncing.htm).
   The ARM CPU can process 32 bit in a single instruction, which speeds things up a lot!
 * If the change persists for 4 read cycles, the vertical counter of that bit will overflow and trigger a definite change.
 * The current debounced state and any bits which toggled during the current iteration are stored in global variables.
 * The serial reporter function looks for changed bits, encodes them and reports them on the serial port
 * The quick-fire rule function checks a list of rules, which can be triggered by changed bits, which can lead to immediate actions, like coils firing.

## `hwIndex` identifies an input / output pin
It is a 16 bit number which uniquely identifies each input / output pin. Note that depending on the number of IO expanders usend in a setup,
not all addresses will be valid. 

         Switch input hwIndex addresses:
        ---------------------------------
         0 - 63  --> Switch matrix on mainboard
         64 - 71 --> Solenoid driver on mainboard     (not an input!)
         72 - 79 --> I2Cch. 0, I2Cadr. 0x41, bit 0-7  (First  external PCL GPIO extender on channel 0)
         80 - 87 --> I2Cch. 0, I2Cadr. 0x42, bit 0-7  (Second external PCL GPIO extender on channel 0)
         312-319 --> I2Cch. 3, I2Cadr. 0x47, bit 0-7  (7th    external PCL GPIO extender on channel 3)

### Calculating the hwIndex for the Switch matrix
        hwIndex = SMrow * 8 + SMcol
where SMrow is the row wire number (from 0-7) and SMcol is the column wire number (from 0-7).

### Calculating the hwIndex for I2C inputs
        hwIndex = 64 + I2Cchannel * 64 + ( I2Cadr - 64 ) * 8 + PinIndex
 where I2Cchannel is the output channel on the mainboard (from 0-3), I2Cadr is the configured I2C address
 set by dip switches on the port extender (0x40 - 0x47) and PinIndex is the output pin (0-7).
 
### What about outputs 
Output extension boards (solenoid driver / digital outputs / etc.) also use the PC8574 I2C chip and hence the same
addresses. The chip automatically siwtches its pins to output mode once an I2C `write` operation has been carried out. So outputs do not need to be
specially configured, just fire the OUT command with the known output address.
 
The PC8574 has open drain output drivers and can only sink current to ground. If the output driver is disabled (by writing the bit to 1), the pin
can be used as an input. If the output driver is enabled (by writing the bit to 0) the pin cannot be used as an input as it is constantly pulled low.
  
!!! To avoid this, the user must take care to not send `OUT` commands on an input `hwIndex` !!!
 
Also note that hwIndex 0 - 63 are not valid output addresses as they refer to the Switch matrix which can never be an output.
That beeing said, I might map hwIndex 60 - 63 to the internal hardware PWM outputs on the mainboard, which do not
use the PCF GPIO extenders. To be seen.

# Serial command API

        Available commands
        ------------------
        ?     : Display list of commands
        *IDN? : Display ID and version info
        SW?   : Return the state of ALL switches (40 bytes)
        OUT   : OUT hwIndex tPulse PWMhigh PWMlow
        OUT   : OUT hwIndex PWMvalue
        RUL   : RUL ID IDin IDout trHoldOff tPulse pwmOn pwmOff
                bPosEdge bAutoOff bLevelTr
        RULE  : Enable  a previously disabled rule: RULE ID
        RULD  : Disable a previously defined rule:  RULD ID
        LED   : LED <CH>,<led0>,<led1> ...

## Switch events
When a switch input flips its state, its hwIndex and new state is immediately reported on the USB serial port

__Example__

The input with hwIndex 0x0F8 changed to 1, 0x0FC changed to 0 and 0x0FE changed to 1 
Received:

        SE:0f8=1 0fa=1 0fc=0 0fe=1;

## `SW?` returns the state of all Switch inputs
Returns 40 bytes as 8 digit hex numbers. This encodes all 320 bits which can be addressed by a hwIndex.

__Example__

Sent:

        SW?
Received:

        SW:00000000 12345678 9ABCDEF0 AFFE0000 DEAD0000 BEEF0000 C0FFEE00 00000000 00000000 00000000

## `OUT` set a solenoid driver output
 * hwIndex 
 * pulse time [ms] (0 - 32767), optional
 * pwm pulse power (0-15), optional
 * pwm hold power  (0-15)
 
__Example__

Set output pin with hwIndex 0x100 to a pwm power level of 10 and keep it there.

        OUT 0x100 10

Pulse output with hwIndex 0x110 for 300 ms with a pwm power level of 10 and then keep it at a power level of 2.

        OUT 0x110 300 10 2

 
## `RUL` setup and enable a quick-fire rule
 * quickRuleId (0-64)
 * input switch ID number
 * driver output ID number
 * post trigger hold-off time [ms]
 * pulse duration [ms]
 * pulse pwm (0-15)
 * hold pwm  (0-15)
 * Enable trigger on pos edge?
 * Enable auto. output off once input releases
 * Enable level Trigger (no edge check)

### Notes
When enabling level trigger, the edge detecion is disabled and the rule will stay in triggered state
as long as the input is high (or low)
 
When auto. output off is enabled, the rule stays in triggered state as long as the level is high. 
When the level is low again, it disables the outputs and arms the trigger again.

Warning: When auto. output off is disabled and level trigger is enabled it leads to a periodic trigger condition.
Sending trigger events every `triggerHoldOffTime` as long as the level is there (not so good)

__Example__

        RUL 0 0x23 0x100 4 1 15 3 1 1 0
    
Setup ruleId 0. Input hwIndex is 0x23, output hwIndex is 0x100. After triggering, at least 4 ms need to ellapse
 before the trigger becomes armed again. Once triggered it pulses the output for 1 ms with pwmPower 15, then it
  holds the output with pwmPower 3. The trigger happens on a positive edge. Once the input is released (and at
   least 4 ms ellapsed), the output is switched off again.

## `LED` dump data to WS2811 RGB LED strings
Raw data values are attached to serial command and just sent on one of the channels over the SPI hardware

## `I2C` does a custom I2C transaction
Not yet implemented. There will be commands to do send / receive of custom bytes to custom addresses. Mutex of I2C hardware!

