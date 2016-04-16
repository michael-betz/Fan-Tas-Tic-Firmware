"""
Contains the hardware interface and drivers for the fantastic Pinball platform
hardware. See https://github.com/yetifrisstlama/Fan-Tas-Tic-Firmware for details.
"""

#Contains template code for a new hardware platform interface for MPF. You
#can use this as the starting point for your own platform.

#Note that if you create your own platform interface, we will be happy to add it
#to the MPF package. That way we can help maintain it moving forward.

#You can search-and-replace the word "template" (not case sensitive) with the
#name of your own platform.



import logging, queue, serial, threading, time
from mpf.core.platform import Platform
from mpf.core.utility_functions import Util
from mpf.core.rgb_color import RGBColor
from mpf.platforms.interfaces.rgb_led_platform_interface import RGBLEDPlatformInterface
from mpf.platforms.interfaces.matrix_light_platform_interface import MatrixLightPlatformInterface
from mpf.platforms.interfaces.gi_platform_interface import GIPlatformInterface
from mpf.platforms.interfaces.driver_platform_interface import DriverPlatformInterface
from collections import defaultdict # For dict of lists
import struct


# you might have to add additional imports here for modules you need for your
# platform


class HardwarePlatform(Platform):
    """This is the base class for your hardware platform. Note that at this
    time, this class *must* be called HardwarePlatform."""
    MAX_QUICK_RULES = 64    #How many hardware slots are available for quick-fire rules

    def __init__(self, machine):
        super().__init__(machine)
        self.machine = machine
        self.log = logging.getLogger("FanTasTic Platform")
        self.log.debug("Configuring FanTasTic hardware interface.")
        self.initial_states_sent = False
        # User defined options for the FanTasTic hardware platform
        platformConfig = self.machine.config["fantastic"]
        platformConfig['config_number_format'] = 'int'

        # The following "features" are supposed to be constants that you can
        # use to define was is or is not in your own platform. However they
        # have not been implemented at this time. So for now just keep them
        # as-is below and we'll deal with them in some future version of MPF.
        self.features['max_pulse'] = 32000              #on-time in [ms]
        self.features['hw_rule_coil_delay'] = False
        self.features['variable_recycle_time'] = True   #trigger hold-off time
        self.features['variable_debounce_time'] = True  #12 ms or 0 ms
        # Make the platform features available to everyone
        self.machine.config['platform'] = self.features

        #----------------------------------------------------------------
        # Global (state) variables
        #----------------------------------------------------------------
        # State of _ALL_ posisble input switches as Binary bit-field
        self.hw_switch_data = None
        # Keep the state of the WS2811 LEDs in bytearrays
        # This is efficient and close to the hardware
        # as data can be dumped to serial port without conversion
        # We got 3 channels of up to 1024 LEDs with 3 bytes
        # These arrays start with size 0 and are extended on demand
        self.ledByteData = [bytearray(), bytearray(), bytearray()]
        self.flag_led_tick_registered = False
        # List to store all configured rules (active and inactive) in tuple format
        self.configuredRules = [None] * HardwarePlatform.MAX_QUICK_RULES
        self.swNameToRuleIdDict = defaultdict( list )

        #----------------------------------------------------------------
        # The serial communication object (buffered, separate threads)
        #----------------------------------------------------------------
        self.serialCom = SerialCommunicator( platformConfig["port"], 115200 )
        self.serialCom.send(b"\n")
        self.serialCom.send(b"SWE 1\n")
        self.serialCom.send(b"*IDN?\n")
        #----------------------------------------------------------------
        # Disable all quickfire rules
        #----------------------------------------------------------------
        CMD = ""
        for rulId in range( HardwarePlatform.MAX_QUICK_RULES ):
            CMD += "RULE {0} 0\n".format( rulId )
        self.serialCom.send( CMD )
        #----------------------------------------------------------------
        # Configure LED channel speeds
        #----------------------------------------------------------------
        for i in range(3):
            try:
                ledKey = "led_clock_{0}".format(i)
                if ledKey in platformConfig:
                    tempSpeed = int( platformConfig[ledKey] )
                    self.serialCom.send( "LEC {0} {1}\n".format(i,tempSpeed) )
                    self.log.info( "LEC {0} {1}\n".format(i,tempSpeed) )
            except Exception as e:
                self.log.warning(e)
                pass

        # Prefixes which can be received from the controller
        self.serialCommands = {
            b'ID' : self.receive_id,  # processor ID and version
            b'SW' : self.receive_sw,  # all switch states
            b'SE' : self.receive_se,  # switch event. State of a switch changed.
        }

    def __repr__(self):
        """String name you'd like to show up in logs and stuff when a
        reference to this platform is printed."""
        return '<Platform.FanTasTic>'

    def tick(self, dt):
        while not self.serialCom.receive_queue.empty():
            self.process_received_message( self.serialCom.receive_queue.get(False) )

    def process_received_message( self, msg ):
        """Sends an incoming message from the fantastic controller to the proper
        method for servicing.
        """
        if msg[2:3] == b':':
            cmd = msg[0:2]
            payload = msg[3:]
        else:
            self.serialCom.send(b"\n")   # Clear previous commands
            self.log.warning("Received malformed message: %s", msg)
            return
        # Can't use try since it swallows too many errors for now
        if cmd in self.serialCommands:
            self.serialCommands[cmd]( payload )
        else:
            self.log.warning("Received unknown serial command? %s", msg)

    def receive_id(self, msg):
        self.log.debug("Received Hardware ID %s", msg)

    def receive_se(self, msg):
        # msg = "0f8=1 0fa=1 0fc=0 0fe=1"
        for se in msg.split( b" " ):
            swId, swState = se.split( b"=" )
            self.machine.switch_controller.process_switch_by_num( num=int(swId, 16), state=int(swState, 16) )

    def receive_sw(self, msg):
        # msg = b"00000000123456789ABCDEF0AFFE0000DEAD0000BEEF0000C0FFEE00000000000000000000000000"
        # Process Hex values in groups of 8 (little endian) hwIndex[0] = 0: b"FFFFFFFE...
        self.log.debug("Received SW: %s", msg)
        hwBytes = bytearray.fromhex( msg.decode() )
        hwLongs = struct.unpack( ">{0}I".format(len(hwBytes)//4), hwBytes )
        # Now we have an array of bytes, but MPF expects an array of bits
        # (Which is a super inefficient btw.) but hey, I will do it ...
        hwBits = bytearray( len(hwLongs)*32 )
        i = 0
        for hwLong in hwLongs:
            for n in range(32):
                hwBits[i] = ((hwLong >> n) & 0x01)  # == 0
                i += 1
        self.hw_switch_data = hwBits

    def update_leds(self, dt):
        """Updates all the WS2811 LEDs. This is done
        once per game loop for efficiency (i.e. the byteBuffer is dumped
        to the hardware. All LEDs are sent as a single
        update rather than lots of individual ones).
        """
        for channel, ledDat in enumerate( self.ledByteData ):
            if len(ledDat) > 0:
                msg = bytes("LED {0} {1}\n".format(channel, len(ledDat)), "utf8") + ledDat
                self.serialCom.send( msg )

    def configure_driver(self, config, device_type='coil'):
        """This method is called once per driver when MPF is starting up. It
        passes the config for the driver and returns a hardware driver object
        that can be used to control the driver.

        Args:
            config: A dict of configuration key/value settings that will be
                passed on to your driver class to create an instance of your
                driver. These can be whatever you want--whatever you need to
                setup a driver.
            device_type: String name of the type of MPF device that's being
                configured. This is needed since some platforms (like Williams
                WPC) use "drivers" to control coils, lights, GI, flashers, etc.
                Whether you need to act on this is up to you. Just know that
                when MPF calls this method, it will pass the config dict for
                this device as well as a string name of what type of device
                it's trying to setup.

        Returns:
            driver object, config number. The driver object that is returned
            is a your hardware driver that should have driver-like methods like
            pulse(), enable(), disable(), etc. It should already be mapped to
            the proper driver so it can be called directly like driver.pulse().
            Different types of drivers (e.g. coils vs. flashers) will have
            different methods available. (Keep reading for details.)

            The config number that's returned isn't really used. It's just
            stored by MPF in case this driver needs to be referenced by number
            in the future.

        """

        # In this example, we're passing the complete config dictionary to the
        # FanTasTicDriver constructor so it can set itself up as needed. You
        # might choose to only pass certain k/v pairs, or whatever else you
        # want.
        driver = FanTasTicDriver( config, self.serialCom )
        return driver, config['number']

    def configure_switch(self, config):
        """Called once per switch when MPF boots. It's used to setup the
        hardware switch interface.

        The main MPF loop will expect to receive notifications of switch
        changes in the main platform.tick() events. It will not poll every
        single switch. (If you need that behavior, you need to write it in the
        tick() method.

        Note: This method is similar to the configure_driver() method.

        Args:
            config: A dict of configuration key/value settings that will be
                passed on to your switch class to create an instance of your
                switch. These can be whatever you want--whatever you need to
                setup a switch.

        Returns:
            A reference to the instance of your harware switch class. This will
            be queried later to read the state of the switch.

        """
        switch = FanTasTicSwitch( config, self.serialCom )
        return switch, switch.number

    def get_hw_switch_states(self):
        """ Called by MPF when it wants to get the state of all Switches at once """
        self.hw_switch_data = None
        self.serialCom.send(b'SW?\n')
        # Wait until the serial receiver thread updates the value
        while not self.hw_switch_data:
            time.sleep(.01)
            self.tick( 0 )
        return self.hw_switch_data

    def configure_led(self, config):
        """ A WS2811 led is identified by its channel number (0-3)
            and position along the chain (0-1023)
        """
        if not self.flag_led_tick_registered:
            # Update leds every frame
            self.machine.clock.schedule_interval(self.update_leds, 0)
            self.flag_led_tick_registered = True

        # we expect the LED number is in <channel> - <led> format
        if '-' in config['number_str']:
            num = config['number_str'].split('-')
            config['channel'] = int(num[0])
            config['number']  = int(num[1])
            if not (0 <= config['channel'] <= 3):
                raise ValueError("Invalid LED Channel: {0}".format(num[0]) )
            if not (0 <= config['number']  <= 1023):
                raise ValueError("Invalid LED number: {0}".format(num[1]) )
            config['config_number_format'] = 'int'

        ledBytes = self.ledByteData[ config['channel'] ]
        this_led = FantasticDirectLED( config, ledBytes )
        return this_led

    def _findFreeSpotForRules( self, nFree = 1 ):
        """ Find `nFree` consequtive `None` elements in a list and return the first index """
        nFound = 0
        for i, rulTuple in enumerate( self.configuredRules ):
            if rulTuple is None:
                nFound += 1
            else:
                nFound = 0
            if nFound >= nFree:
                return i - (nFree-1)
        raise OverflowError("_findFreeSpotForRules(): No free slot for quick-fire rule found!")

    def write_hw_rule(self, switch_obj, sw_activity, driver_obj, driver_action,
                      disable_on_release=True, drive_now=False,
                      **driver_settings_overrides):
        """Used to write (or update) a hardware rule to the FanTasTic controller.

        *Hardware Rules* are used to configure the hardware controller to
        automatically change driver states based on switch changes. These rules
        are completely handled by the hardware (i.e. with no interaction from
        the Python game code). They're used for things that you want to happen
        fast, like firing coils when flipper buttons are pushed, slingshots, pop
        bumpers, etc.

        You can overwrite existing hardware rules at any time to change or
        remove them.

        Args:
            switch_obj: Which switch you're creating this rule for. The
                parameter is a reference to the switch object itself.
            sw_activity: Int which specifies whether this coil should fire when
                the switch becomes active (1) or inactive (0)
            driver_obj: Driver object this rule is being set for.
            driver_action: String 'pulse' or 'hold' which describe what action
                will be applied to this driver
            disable_on_release: Actually put a second rule to disable the coil again
            drive_now: Should the hardware check the state of the switches when
                this rule is first applied, and fire the coils if they should
                be? Typically this is True, especially with flippers because you
                want them to fire if the player is holding in the buttons when
                the machine enables the flippers (which is done via several
                calls to this method.)
        """
        # driver_settings_overrides =
        # {'enable_events': {'machine_reset_phase_3': 0},
        #  'tags': [],
        #  'eos_switch': None,
        #  'pwm_off_ms': None,
        #  'hold_pwm_mask': None,
        #  'enable_no_hold_events': {},
        #  'pulse_pwm_mask': None,
        #  'pulse_power32': None,
        #  'hold_power': None,
        #  'debounced': False,
        #  'invert_events': {},
        #  'use_eos': False,
        #  'pulse_power': None,
        #  'main_coil': <coil.c_flipper_right>,
        #  'label': '%',
        #  'pulse_ms': None,
        #  'disable_events': {'ball_ending': 0},
        #  'recycle_ms': None,
        #  'hold_coil': None,
        #  'hold_power32': None,
        #  'activation_switch': <switch.s_flipper_right>,
        #  'debug': False,
        #  'pwm_on_ms': None }
        isPosEdge = sw_activity == 1
        tPulse  = driver_obj.hw_driver.tPulse
        pwmHigh = driver_obj.hw_driver.pwmHigh
        # pwmLow is the power setting after the ON time, which is zero if a `pulse` is requested
        if driver_action == "pulse":
            pwmLow = 0
        else:
            pwmLow = driver_obj.hw_driver.pwmLow
        hwIndexSw  = switch_obj.number
        hwIndexOut = driver_obj.number
        #`recycle_ms` in MPF lingo is equivalent to the trigger-hold-off time on a scope
        trHoldOff = driver_settings_overrides["recycle_ms"]
        if trHoldOff is None:
            trHoldOff = 0
        # Get the next free index for a quick-fire-rule slot
        rulId = self._findFreeSpotForRules( 1 )
        rulTuple = (rulId, hwIndexSw, hwIndexOut, trHoldOff, tPulse, pwmHigh, pwmLow, int(isPosEdge))
        CMD = "RUL {0} {1} {2} {3} {4} {5} {6} {7}\n".format( *rulTuple )
        # Remember which rules are associated with this switch-name in the `swNameToRuleIdDict`
        self.swNameToRuleIdDict[ switch_obj.name ].append( rulId )
        # We keep the current state of all rule-slots in a the `configuredRules` list.
        self.configuredRules[ rulId ] =  rulTuple
        #--------------------------------------------------
        # For `disable_on_release` we need to configure a
        # second quickfirerule, triggering on the other
        # edge, disabling the coil
        #--------------------------------------------------
        if disable_on_release:
            rulId = self._findFreeSpotForRules( 1 )
            rulTuple = (rulId, hwIndexSw, hwIndexOut, 0, 0, 0, 0, int(not isPosEdge) )
            CMD += "RUL {0} {1} {2} {3} {4} {5} {6} {7}\n".format( *rulTuple )
            self.swNameToRuleIdDict[ switch_obj.name ].append( rulId )
            self.configuredRules[rulId] = rulTuple
        self.log.info( "{0} [{1}]".format(CMD.replace('\n',', '),switch_obj.name) )
        self.serialCom.send( CMD )

    def clear_hw_rule(self, sw_name):
        rulIds = self.swNameToRuleIdDict.pop( sw_name )
        #print( "clear_hw_rule:", rulIds, self.configuredRules )
        CMD = ""
        for rulId in rulIds:
            rulTuple = self.configuredRules[rulId]
            self.configuredRules[rulId] = None
            # Disable the rule
            CMD += "RULE {0} 0\n".format( rulId )
            # Just in case the flipper still in hold state, reset the coil
            CMD += "OUT {0} 0\n".format( rulTuple[2] )
        del rulIds
        self.log.info( "{0} [{1}]".format(CMD.replace('\n',', '),sw_name) )
        self.serialCom.send( CMD )

    def i2c_write8(self, address, register, value):
        pass

    def i2c_read8(self, address, register):
        del address
        del register
        return None

    def i2c_read16(self, address, register):
        del address
        del register
        return None

class FanTasTicSwitch(object):
    """Represents a switch in a pinball machine used with virtual hardware."""

    def __init__(self, config, serCom):
        # config =
        # {'debounce_open': 0,
        # 'type': 'NO',
        # 'events_when_activated': [],
        # 'debounce': 'True',
        # 'tags': [],
        # 'number_str': '78',
        # 'debounce_close': 0,
        # 'debug': False,
        # 'number': '78',
        # 'platform': None,
        # 'label': '%',
        # 'recycle_time': 0.0,
        # 'events_when_deactivated': [] }
        self.log = logging.getLogger('FanTasTicSwitch')
        self.serCom = serCom
        self.number  = int( config["number"] )  #hwIndex and bitNumber!
        if( self.number<0 or self.number>319 or self.number in range(64,71) ):
            raise ValueError("Invalid input hwIndex: 0x{0:02x}".format(config["number"]) )
        self.isNormallyOpen = config["type"] == "NO"
        self.isDebounce = bool( config["debounce"] )
        if( self.isDebounce ):
            serCom.send( "DEB {0:d} 1\n".format(self.number) )
        else:
            serCom.send( "DEB {0:d} 0\n".format(self.number) )

class FantasticDirectLED( RGBLEDPlatformInterface ):
    # WS2811 / WS2812 strips propably come in all kind of R-G-B permutations
    # This function converts an RGBcolor object to a triplet in the right order
    colorConverterFcts = {
        "rgb": lambda color: (color.red,   color.green, color.blue),
        "rbg": lambda color: (color.red,   color.blue,  color.green),
        "grb": lambda color: (color.green, color.red,   color.blue),
        "gbr": lambda color: (color.green, color.blue,  color.red),
        "bgr": lambda color: (color.blue,  color.green, color.red),
        "brg": lambda color: (color.blue,  color.red,   color.green),
    }

    def __init__( self, config, ledBytes ):
        self.log = logging.getLogger('FantasticDirectLED')
        self.number = config["number"]
        self.channel = config["channel"]
        self._current_color = config["default_color"] #an RGBColor object
        self.colorOrder = config["color_channel_map"]
        if not self.colorOrder in FantasticDirectLED.colorConverterFcts:
            raise ValueError( "Unknonw Color-order: {0}".format(self.colorOrder) )
        # Find fct. to convert logical order to HW order
        self.colorConverterFct = FantasticDirectLED.colorConverterFcts[ self.colorOrder ]
        # If the user configures the LED with index 5,
        # we need to be sure that index (5*3, 5*3+1, 5*3+2) exists
        # in ledData (len must be >= 18)
        self.ledBytes = ledBytes
        minLedIndex = self.number * 3
        maxLedIndex = self.number * 3 + 2
        self.ledIndex = minLedIndex
        if maxLedIndex >= len( self.ledBytes ):  #We need to extend the array
            self.ledBytes[minLedIndex:maxLedIndex] = b"\x00"*((maxLedIndex+1)-len(self.ledBytes))
        self.log.info("Creating RGB LED. Channel = %d,  ByteArraySize = %d",
            self.channel, len(self.ledBytes) )

    def color(self, color):
        """Instantly sets this LED to the color passed.
        Args:
            color: an RGBColor object
        """
        # ToDo: Why is this called multiple times in each light-show step?
        #print( self.channel, self.number, color )
        self._current_color = color
        triplet = self.colorConverterFct( color )
        # We modify the ledBytes array in the right place
        self.ledBytes[ self.ledIndex : self.ledIndex+3 ] = triplet

    def disable(self):
        """Disables (turns off) this LED instantly. For multi-color LEDs it
        turns all elements off.
        """
        self.color( RGBColor("off") )

    def enable(self):
        self.color( RGBColor("white") )

    @property
    def current_color(self):
        return self._current_color

class FanTasTicDriver(DriverPlatformInterface):
    """The base class for a hardware driver on your platform. Several methods
    are required, including:

    disable()
    enable()
    pulse()

    The following are optional:
    state()
    tick()
    reconfigure()

    """
    def __init__(self, config, serCom):
        self.log = logging.getLogger('VirtualDriver')
        self.serCom = serCom
        self.driver_settings = config
        self.number  = int( config["number"] )
        self.tPulse  = int( config["pulse_ms"] )
        # User specified pulse power from 0 - 100
        self.pwmHighNorm = float( config["pulse_power"] )
        if config["hold_power"] is None:
            self.pwmLowNorm = 0.0
        else:
            self.pwmLowNorm = float( config["hold_power"] )
        # Calculate pwm setting
        if( self.number in (0x3C, 0x3D, 0x3E, 0x3F) ): #Hardware PWM channel
            self.pwmLow  = int( self.pwmLowNorm  / 100.0 * 8000.0 + 0.5 )
            self.pwmHigh = int( self.pwmHighNorm / 100.0 * 8000.0 + 0.5 )
        else:                                          #I2C BCM channel
            self.pwmLow  = int( self.pwmLowNorm  / 100.0 * 7 + 0.5 )
            self.pwmHigh = int( self.pwmHighNorm / 100.0 * 7 + 0.5 )
        print( "Driver: 0x{0:02X}, Lo: {1}, Hi: {2}".format(self.number, self.pwmLow, self.pwmHigh) )

    def __repr__(self):
        return "VirtualDriver.{}".format(self.number)

    def disable(self):
        #  OUT   : <hwIndex> <PWMlow> [tPulse] [PWMhigh]
        cmd = "OUT {0} 0\n".format(self.number)
        print( cmd )
        self.serCom.send( cmd )

    def enable(self):
        """Enables this driver, which means it's held "on" indefinitely until
        it's explicitly disabled.
        """
        # The actual code here will be the call to your library or whatever
        # that physically enables this driver. Remember this object is just for
        # this single driver, so it's up to you to set or save whatever you
        # nned in __init__() so that you know which driver to call from this
        # instance of the class.
        # for example (pseudocode):
        # self.serial_connection.send(self.number, 1)
        # or
        # self.my_driver(self.driver_number, command='enable', time=-1)
        #  OUT   : <hwIndex> <PWMlow> [tPulse] [PWMhigh]
        cmd = "OUT {0} {1}\n".format( self.number, self.pwmHigh )
        print( cmd )
        self.serCom.send( cmd )

    def pulse(self, milliseconds=None):
        """Pulses this driver for a pre-determined amount of time, after which
        this driver is turned off automatically. Note that on most platforms,
        pulse times are a max of 255ms. (Beyond that MPF will send separate
        enable() and disable() commands.

        Args:
            milliseconds: The number of ms to pulse this driver for. You should
                raise a ValueError if the value is out of range for your
                platform. If this value is None, you should pulse this driver
                with a default setting. You can set the default in the driver
                config via your __init__() method, or you can pick some default
                that's hard coded.

        Returns:
            A integer of the actual time this driver is going to be pulsed for.
            MPF uses this for timing in certain situations to make sure too
            many drivers aren't activated at once.

        """
        if not milliseconds:
            milliseconds = self.tPulse
        if not ( 0 < milliseconds < 32760):
            raise ValueError("tPulse is out of range: {0}".format(milliseconds))
        # do the actual hardware pulse... whatever that looks like for your
        # platform
        #  OUT   : <hwIndex> <PWMlow> [tPulse] [PWMhigh]
        cmd = "OUT {0} {1} {2} {3}\n".format( self.number, self.pwmLow, milliseconds, self.pwmHigh )
        print( cmd )
        self.serCom.send( cmd )
        return milliseconds

class SerialCommunicator(object):
    """ Does asynchronous buffered serial communication in a separate thread """
    def __init__(self, port, baud):
        self.log = logging.getLogger('SerialCommunicator')
        self.send_queue = queue.Queue()
        self.receive_queue = queue.Queue()
        self.log.info("Connecting to %s at %sbps", port, baud)
        self.serial_io = serial.Serial( port=port, baudrate=baud, timeout=1 )
        self._start_threads()

    def _start_threads(self):
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self.sending_thread = threading.Thread(target=self._sending_loop)
        self.sending_thread.daemon = True
        self.sending_thread.start()

    def stop(self):
        """Stops and shuts down this serial connection."""
        self.serial_io.close()
        self.serial_io = None  # child threads stop when this is None
        # todo clear the hw?

    def send(self, msg):
        """Sends a message to the remote processor over the serial connection.

        Args:
            msg: String of the message you want to send.

        """
        if type(msg) == str:
            msg = bytes( msg, "utf8" )
        self.send_queue.put(msg)

    def _sending_loop(self):
        try:
            while self.serial_io:
                msg = self.send_queue.get()
                self.serial_io.write( msg )
                #self.log.info("TX: %s", msg)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
            msg = ''.join(line for line in lines)
            self.log.error( msg )
            #self.machine.crash_queue.put(msg)

    def _receive_loop(self):
        try:
            while self.serial_io:
                msg = self.serial_io.readline().strip()  # strip the \n
                if len(msg) > 0:
                    self.receive_queue.put( msg )
                    # self.log.info("RX: %s", msg)
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            lines = traceback.format_exception(exc_type, exc_value,
                                               exc_traceback)
            msg = ''.join(line for line in lines)
            self.log.error( msg )
            #self.machine.crash_queue.put(msg)
