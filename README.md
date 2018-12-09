# Fan-Tas-Tic-Controller
Controller for Pinball machines based on an TM4C123G LaunchPad™ Evaluation Kit, compatible with the [Mission Pinball API](https://missionpinball.com/).

## Design ideas
  * Make the system as modular as possible
  * Use cheap and easily available components
  * [Free and open-source software (FOSS)](https://en.wikipedia.org/wiki/Free_and_open-source_software)

## Hardware features
 * 8x8 Switch matrix inputs
 * 12 onboard drivers for solenoids, 4 of them can do hardware PWM (> 100 kHz)
 * 4 x I2C channels for extension boards (solenoid drivers, switch inputs, LED drivers, servo drivers, ...)
 * 3 x SPI channels for running [WS2811 / WS2812 LED strings](http://www.ebay.com/sch/i.html?_from=R40&_trksid=p2050601.m570.l1313.TR0.TRC0.H0.Xled+strand+ws2811.TRS5&_nkw=led+strand+ws2811&_sacat=0) with up to 1024 LEDs per channel
 * In- / Outputs can be easily and cheaply added with [PCF8574](http://www.ti.com/product/pcf8574) I2C GPIO extenders (check eBay for [cheap I/O modules](http://www.ebay.com/sch/i.html?_sacat=0&_nkw=i2c+expander&_frs=1))
 * Super fast USB virtual serial connection to host PC
 * KiCad PCB files available, no tiny SMD components, can be assembled by hand

## Software features
 * Software can handle up to 320 channels which can be used as In- or Outputs
 * All In- / Output channels are identified by a 16 bit unique ID. No configuration necessary.
 * All Outputs support 3 bit PWM with > 125 Hz (using [binary code modulation](http://www.batsocks.co.uk/readme/art_bcm_1.htm) over I2C)
 * All inputs are read 1000 times per second and optionally debounced. A switch toggles after keeping its state for 4 ms.
 * The timing for the WS2811 LEDs is kept within spec by using the hardware SPI module and DMA transfers
 * Firmware is based on [FreeRTOS](http://www.freertos.org/)
 * Compatible with the [Mission Pinball](http://missionpinball.org/) API with this [Hardware platform driver](https://github.com/yetifrisstlama/Fan-Tas-Tic-platform)

# Digital inputs / outputs
There are 4 I2C channels which can handle up to 8 x PCF8574 each. The addresses of the chips needs to be configured from 0x20 - 0x27.

To use an PCF8574 as an input, a 0xFF needs to be _written_ to it (to prevent the open drain outputs pulling the pin low). This can be achieved with the `HI` command.
Then the pin state can be acquired with a 1 byte read transaction over I2C.

## Debouncing
The task_pcf_io() is running at 1000 Hz, reading all switches (matrix and I2C) into an array
 * Switch Matrix (SM) is read in foreground while I2C transactions happens in interrupt context
 * When both reads are finished, the Debouncing routine is called
 * More info about the I2C engine in this [blog post](https://yetifrisstlama.github.io/fan-tas-tic-firmware-upgrade/)
 * If a changed input bit was detected, it increments a 2 bit [vertical counter](http://www.compuphase.com/electronics/debouncing.htm).
   The ARM CPU can process 32 input bits in a single instruction, which speeds things up!
 * If the change persists for 4 read cycles, the vertical counter of that bit will overflow and trigger a definite change.
 * The current debounced state and any bits which toggled during the current iteration are stored in global variables.
 * The serial reporter function looks for changed bits, encodes them and reports them as `Switch Events` on the serial port
 * The quick-fire rule function checks a list of rules, which can be triggered by changed bits, which can lead to immediate actions, like coils firing

## `hwIndex` identifies an input / output pin
It is a 16 bit number which uniquely identifies each input / output pin. Note that depending on the number of IO expanders usend in a setup,
not all addresses will be valid.

         hwIndex for inputs
        ---------------------------------
         0x000 - 0x03F --> Switch matrix inputs on mainboard
       [ 0x040 - 0x047 --> I2C Solenoid driver on mainboard  NOT AN INPUT! ]
         0x048 - 0x04F --> I2Cch. 0, I2Cadr. 0x21, bit 0-7  (First  external PCL GPIO extender on channel 0)
         0x050 - 0x057 --> I2Cch. 0, I2Cadr. 0x22, bit 0-7  (Second external PCL GPIO extender on channel 0)
         0x138 - 0x13F --> I2Cch. 3, I2Cadr. 0x27, bit 0-7  (7th    external PCL GPIO extender on channel 3)

### Calculating the hwIndex for the Switch matrix
        hwIndex = SMrow * 8 + SMcol
where SMrow is the row wire number (from 0 - 7) and SMcol is the column wire number (from 0 - 7).

### Calculating the hwIndex for I2C inputs
        hwIndex = 0x40 + I2Cchannel * 0x40 + (I2Cadr - 0x20) * 8 + PinIndex
 where I2Cchannel is the output channel on the mainboard (from 0 - 3), I2Cadr is the configured I2C address
 set by dip switches on the port extender (0x20 - 0x27) and PinIndex is the output pin (0 - 7).

### What about I2C outputs
Output extension boards (solenoid driver / digital outputs / etc.) also use the PCF8574 I2C chip and hence the same
addresses. The chip automatically switches its pins to output mode once an I2C `write` operation has been carried out.
So outputs do not need to be specially configured, just fire the OUT command with the known output address.

The PC8574 has open drain output drivers and can only sink current to ground. If the output driver is disabled
(by writing the bit to 1), the pin can be used as an input. If the output driver is enabled
(by writing the bit to 0) the pin cannot be used as an input as it is constantly pulled low.

__!!! To avoid this, the user must take care to not send `OUT` commands on an `hwIndex` which is used as an input !!!__

The mainboard features a built-in PCF8574 to drive 8 solenoids. use hwIndex 64 - 71 to address these channels.

### What about the HW. PWM outputs
The mainboard features 4 high resolution PWM solenoid drivers running at 50 kHz.
For these, the pwm hold power and pwm pulse power can be specified from 0 - 1500.
The hwIndexes 0x3C - 0x3F are mapped in any `OUT` command to the HW. PWM channels.
For any `IN` command, these addresses are mapped to the highest 4 Switch Matrix inputs.

# Building
Instructions for Ubuntu 18

### Install gcc for arm
```bash
$ sudo apt-get install gcc-arm-none-eabi openocd build-essential make
# to get miniterm.py
$ pip install pyserial
```

### Download TivaWare Full Release
From here: [SW-TM4C-2.1.4.178.exe](http://www.ti.com/tool/SW-TM4C)
It's free but registration at ti.com is necessary.

```bash
$ echo "c74ef1da07246d4ad20a92521ba44a7d567f6bc62556cd1b76e7fd6a8a75bf8c  SW-TM4C-2.1.4.178.exe" | sha256sum -c -
SW-TM4C-2.1.4.178.exe: OK

$ unzip SW-TM4C-2.1.4.178.exe
```

### Build and flash the project
Now edit `Makefile` and change the path `ROOT = ` to the tivaware folder from above.

Plug in the debug connector and:
```bash
$ make
$ make flash
$ miniterm.py /dev/ttyACM0 115200
```

# Serial command API
The Tiva board has two physical USB connectors. The `DEBUG` port is used to load and debug the firmware.
It also provides a virtual serial port, which can be opened in a terminal to enter commands manually and
to see status and debug messages from the Fan-Tas-Tic firmware.

The `DEVICE` port provides a virtual serial port
which is meant to communicate with the (Python) host application. This port listens to the same commands but does not echo any
input characters or status messages, which makes it easier to talk to programatically.

Note that the `DEVICE` port reports errors in the form of an `ER:xxxx\n` error code. They can be looked up in [this](https://docs.google.com/spreadsheets/d/1QlxT6QhTLHodxV4uOGEEIK3jQQLPyiI4lmSObMyx4UE/edit?usp=sharing) (slightly out of date) table.

    **************************************************
     Available commands   <required>  [optional]
    **************************************************
    ?     : Display list of commands
    *IDN? : Display ID and version info
    IL    : I2C: List status of GPIO expanders
    IR    : I2C: Reset I2C system
    OL    : I2C: List output writers
    HI    : <hwIndex> set all ports of PCF high (input mode)
    SWE   : <OnOff> En./Dis. reporting of switch events
    DEB   : <hwIndex> <OnOff> En./Dis. 12 ms debouncing
    SW?   : Return the state of ALL switches (40 bytes)
    SOE   : <OnOff> En./Dis. 24 V solenoid power (careful!)
    OUT   : <hwIndex> <PWMlow> [tPulse] [PWMhigh]
    RUL   : <ID> <IDin> <IDout> <trHoldOff>
            <tPulse> <pwmOn> <pwmOff> <bPosEdge>
    RULE  : En./Dis a prev. def. rule: RULE <ID> <OnOff>
    LEC   : <channel> <spiSpeed [Hz]> [frameFmt]
    LED   : <channel> <nBytes>\n<binary blob of nBytes>
    I2C   : <channel> <I2Caddr> [hexSendData] <nBytesRx>


## `IL` I2C input list
Show status of the 4 I2C channels in 4 columns. The PCF8574 GPIO expanders have
an I2C address between 0x20 and 0x27. Each address corresponds to one row.
`R` indicates the chip is read (inputs), `W` indicates it is written (outputs).
The `[I2C_ADDRESS]` follows. Then the last read value. Then the `(I2C_ERROR_COUNT)`.
The latter is incremented on every transaction without ACK.

__Example__

Sent:

    IL\n

Received:

    R/W[I2C_ADDR]: VAL (ERR_CNT)
    W[20]:    (    0)   [20]:              [20]:              [20]:
    R[21]: ff (    0)   [21]:              [21]:              [21]:
     [22]:              [22]:              [22]:              [22]:
     [23]:              [23]:              [23]:              [23]:
     [24]:              [24]:              [24]:              [24]:
     [25]:              [25]:              [25]:              [25]:
     [26]:              [26]:              [26]:              [26]:
     [27]:             R[27]:  0 (    0)   [27]:              [27]:

## `OL` I2C output list
Show a list of active output channels. These channels are continously updated
to achieve the `bcm` intensity modulation. Syntax is:
  * array index `N:`
  * `[I2C_CHANNEL, I2C_ADDRESS]`
  * Intensity values for each of the 8 bits of this PCF8574

__Example__

Sent:

    OL\n

Received:

    N: [CH,I2C] PWM0 PWM1 ...
    0: [0,20]    3    0    0    0    0    0    0    0

## `SWE` enables the reporting of Switch events
When a switch input flips its state, its hwIndex and new state is immediately reported on the USB serial port.
This feature is disabled by default and needs to be enabled with the `SWE 1\n` command.

__Example__

Sent:

        SWE 1\n

Afterwards, the input with hwIndex 0x0F8 changed to 1, 0x0FC changed to 0 and 0x0FE changed to 1

Received:

        SE:0f8=1 0fa=1 0fc=0 0fe=1\n

## `DEB` disables the debouncing timer for certain inputs
By default, each input is buffered by a deboucning timer, which recognizes a change in input level only after it has been kept stable for 4 ms. This can be disabled to minimize input latency (for example for jet bumpers).

 __Example__

 Sent:

         DEB 0x0012 0\n

Disables debouncing for the input with `hwIndex` 0x0012.

## `SW?` returns the state of all Switch inputs
Returns 40 bytes as 8 digit hex numbers. This encodes all 320 bits which can be addressed by a hwIndex.

__Example__

Sent:

        SW?\n

Received:

        SW:00000000123456789ABCDEF0AFFE0000DEAD0000BEEF0000C0FFEE00000000000000000000000000\n

## `SOE` enable 24 V solenoid power

The Fan-Tas-Tic mainboard foresees a relay to disable the 24 V supply voltage to the solenoids. This is for safety reasons (in case of firmware hang-ups) -- but also to make sure the solenoid drivers do not have power until they are initialized.
So the solenoids have to be manually activated by the user.

__Example__

Sent:

         SOE 1\n

Enables the 24 V supply to the solenoids (sets PE0 high).

## `OUT` set a solenoid driver output
 * hwIndex
 * pwm hold power  (0-7)
 * pulse time in ms  (0 - 32767), optional
 * pwm pulse power (0-7), optional

__Example__

Set output pin with hwIndex 0x100 to a pwm power level of 2 and keep it there.

Sent:

        OUT 0x100 2\n

Pulse output with hwIndex 0x110 for 300 ms with a pwm power level of 7 and then keep it at a power level of 2.

Sent:

        OUT 0x110 2 300 7\n


## `RUL` setup and enable a quick-fire rule
 * quickRuleId (0-64)
 * input switch ID number
 * driver output ID number
 * post trigger hold-off time [ms]
 * pulse duration [ms]
 * pulse pwm (0-7)
 * hold pwm  (0-7)
 * Enable trigger on pos edge?

### Logic for each rule

        If a rule is enabled:
            If it is currently triggered:
                If holdOff time expired:
                    set Rule to untriggered state
                Else:
                    decrement holdOff time
            else:
                Check if the input matches the trigger condition:
                    Set Rule to triggered state
                    switch output ON

__Example__

Sent:

        RUL 0 0x23 0x100 40 20 7 2 1\n

Setup ruleId 0. Input hwIndex is 0x23, output hwIndex is 0x100. After triggering, at least 40 ms need to ellapse
 before the trigger becomes armed again. Once triggered it pulses the output for 20 ms with pwmPower 7, then it
  holds the output with pwmPower 2. The trigger happens on a positive edge.

## `LED` dump data to WS2811 RGB LED strings
There are 3 channels which can address up to 1024 LEDs each.
First argument is the channel address (0-2).
Second argument is the number of bytes which will be sent (nBytes) and must be an integer multiple of 3.
Each LED needs to be set with 1 byte per color. For the WS2811 LED chip, the order of the bytes is RGB.
For performance reasons, data must be sent as raw binary values (not ascii encoded!).

__Example__

Sent:

        LED 1 6\n
        \xFF\xFF\xFF\x7F\x00\x00

Set the first two LEDs on channel 1. The first LED will glow white at full power, the second red at half power.

### Troubleshooting glitches
If you get glitches and artifacts on your LEDs, you can try the following:

  * Play with the SPI speed setting (`LEC` command). Some LED strings,
    especially the cheaper ones can significantly deviate from specifications
  * Try shorter cables to the first LED
  * If nothing else helps, get out the scope

## `LEC` configure the WS2811 data rate
Set the output data-rate of the SPI module in bits / s.
Note that 4 bits are needed to transmit 1 WS2811 `baud`.
The WS2811 chip supports low speed (400 kBaud) and high speed (800 kBaud) mode. This setting is applied by
the voltage level on a physical pin on the chip. The WS2812 LED only supports 800 kBaud mode.

    -------------------------
     kBaud     spiSpeed [Hz]
       400   =       1600000
       800   =       3200000
    -------------------------

This command allows the timing to be fine-tuned, to remove glitches.

The second argument `frameFmt` is optional and allows to experiment with the SPI frame format.
Refer to the [TM4C1294 datasheet](http://www.ti.com/lit/gpn/tm4c123gh6pm) for details.

    ---------------------------------------------
     frameFmt    Description
         0x00    Moto fmt, polarity 0, phase 0
         0x02    Moto fmt, polarity 0, phase 1
         0x01    Moto fmt, polarity 1, phase 0
         0x03    Moto fmt, polarity 1, phase 1
         0x10    TI frame format
         0x20    National MicroWire frame format
    ---------------------------------------------

__Example__

Sent:

        LEC 0 1700000\n

Set the SPI speed of the first LED channel to 1.7 Mbit/s.
As 4 SPI bits encode 1 WS2811 clock period, the effective speed is 425 kBaud.


## `I2C` do a custom I2C transaction

__work in progress__

This command does a send / receive transaction on one of the I2C busses. Use this to communicate with custom extension boards from python.

__Example__

Sent:

         I2C 3 0x20 ABCDEF 2\n

Received:

         I2:E3B4\n

Do an I2C transaction on channel 3. The right shifted device address (without R/W bit) is 0x20. Send the 3 bytes of data 0xAB, 0xCD, 0xEF. Then read 2 bytes of data from the device, which are 0xE3 and 0xB4.

