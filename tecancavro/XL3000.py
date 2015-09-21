"""
XL3000.py

Tecan Cavro XL3000 Pump.

"""
import time
import logging

from math import sqrt
from time import sleep
from contextlib import contextmanager

try:
    from gevent import monkey; monkey.patch_all(thread=False)
    from gevent import sleep
except:
    from time import sleep

from syringe import Syringe, SyringeError, SyringeTimeout, execWrap


class XL3000(Syringe):
    """
    Class to control XCalibur pumps with distribution valves. Provides front-
    end validation and convenience functions (e.g. smartExtract) -- see
    individual docstrings for more information.
    """

    SPEED_CODES = {0: 6000, 1: 5600, 2: 5000, 3: 4400, 4: 3800, 5: 3200,
                   6: 2600, 7: 2200, 8: 2000, 9: 1800, 10: 1600, 11: 1400,
                   12: 1200, 13: 1000, 14: 800, 15: 600, 16: 400, 17: 200,
                   18: 190, 19: 180, 20: 170, 21: 160, 22: 150, 23: 140,
                   24: 130, 25: 120, 26: 110, 17: 100, 28: 90, 29: 80,
                   30: 70, 31: 60, 32: 50, 33: 40, 34: 30, 35: 20, 36: 18,
                   37: 16, 38: 14, 39: 12, 40: 10}

    def __init__(self, com_link, num_ports=8, syringe_ul=1000, init_port=1,
                 microstep=True, slope=14, init_force=-1,
                 debug=False, debug_log_path='.'):
        """
        Object initialization function.

        Args:
            `com_link` (Object) : instantiated TecanAPI subclass / transport
                                  layer (see transport.py)
                *Must have a `.sendRcv(cmd)` instance method to send a command
                    string and parse the reponse (see transport.py)
        Kwargs:
            `num_ports` (int) : number of ports on the distribution valve
                [default] - 8
            `syringe_ul` (int) : syringe volume in microliters
                [default] - 1000
            `init_port` (int) : port for initialization. It must be the first
                                or the last port.
                [default] - 1
            `microstep` (bool) : whether or not to operate in microstep mode
                [default] - True
            `slope` (int) : slope setting
                [default] - 14 (factory default)
            `init_force` (int) : initialization force or speed
               -1 [default] - calculated according to `syringe_ul`.
                0 - full plunger force and default speed
                1 - half plunger force and default speed
                2 - one third plunger force and default speed
                10-40 - full force and speed code X
            `debug` (bool) : turns on debug file, which logs extensive debug
                             output to 'XL3000_debug.log' at
                             `debug_log_path`
                [default] - False
            `debug_log_path` : path to debug log file - only relevant if
                               `debug` == True.
                [default] - '.' (cwd)

        """
        super(XL3000, self).__init__(com_link, debug=debug, debug_log_path=debug_log_path)
        self.num_ports = num_ports
        self.syringe_ul = syringe_ul
        self.init_port = init_port
        self.init_force = init_force
        self.state = {
            'plunger_pos': None,
            'port': None,
            'microstep': microstep,
            'start_speed': None,
            'top_speed': None,
            'cutoff_speed': None,
            'slope': slope
        }

        self.setMicrostep(on=microstep)

        self.sim_speed_change = False
        self.sim_state = {k: v for k,v in self.state.items()}

        # Init functions
        self.updateSpeeds()
        self.getPlungerPos()
        self.getCurPort()
        self.updateSimState()

    def updateSimState(self):
        """
        Copies the current state dictionary (`self.state`) to the
        simulation state dictionary (`self.sim_state`)

        """
        self.logCall('updateSimState', locals())

        self.sim_state = {k: v for k,v in self.state.items()}

    def cacheSimSpeeds(self):
        """
        Caches the simulation state speed settings when called. May
        be used for convenience functions in which speed settings
        need to be temporarily changed and then reverted

        """
        self.logCall('cacheSimSpeeds', locals())

        self._cached_start_speed = self.sim_state['start_speed']
        self._cached_top_speed = self.sim_state['top_speed']
        self._cached_cutoff_speed = self.sim_state['cutoff_speed']

    def restoreSimSpeeds(self):
        """ Restores simulation speeds cached by `self.cacheSimSpeeds` """
        self.logCall('restoreSimSpeeds', locals())

        self.sim_state['start_speed'] = self._cached_start_speed
        self.sim_state['top_speed'] = self._cached_top_speed
        self.sim_state['cutoff_speed'] = self._cached_cutoff_speed
        self.setTopSpeed(self._cached_top_speed)
        if 50 <= self._cached_start_speed <= 1000:
            self.setStartSpeed(self._cached_start_speed)
        if 50 <= self._cached_cutoff_speed <= 2700:
            self.setCutoffSpeed(self._cached_cutoff_speed)

    def resetChain(self, on_execute=False, wait_ready=False):
        Syringe.resetChain(self, on_execute, wait_ready)
        if (on_execute and self.sim_speed_change):
                self.state['slope'] = self.sim_state['slope']
                self.state['microstep'] = self.sim_state['microstep']
                self.updateSpeeds()
                self.getCurPort()
                self.getPlungerPos()
        self.sim_speed_change = False
        self.updateSimState()

    #########################################################################
    # Pump initialization                                                   #
    #########################################################################

    def init(self, init_force=None, init_port=None):
        """
        Initialize pump. Uses instance `self.init_force` and `self.init_port`
        if not provided. Blocks until initialization is complete.

        """
        self.logCall('init', locals())

        init_force = init_force if init_force is not None else self.init_force
        init_port = init_port if init_port is not None else self.init_port

        if init_port == 1:
            init_cmd = "Z"
        elif init_port == self.num_ports:
            init_cmd = "Y"
        else:
            raise ValueError('`init_port` [{0}] must be 1 or {1}'.format(init_port, self.num_ports))

        if init_force < 0:
            init_force = self._calcInitForce()

        cmd_string = '{0}{1}'.format(
                     init_cmd,
                     init_force)
        self.sendRcv(cmd_string, execute=True)
        self.waitReady()
        return 0  # 0 seconds left to wait

    #########################################################################
    # Command set commands                                                  #
    #########################################################################

    def setMicrostep(self, on=False):
        """ Turns microstep mode on or off """
        self.logCall('setMicrostep', locals())

        cmd_string = 'N{0}'.format(int(on))
        self.sendRcv(cmd_string, execute=True)
        self.microstep = on

    @execWrap
    def setSpeed(self, speed_code):
        """ Set top speed by `speed_code` (see OEM docs) """
        self.logCall('setSpeed', locals())

        if not 0 <= speed_code <= 40:
            raise(ValueError('`speed_code` [{0}] must be between 0 and 40'
                             ''.format(speed_code)))
        cmd_string = 'S{0}'.format(speed_code)
        self.sim_speed_change = True
        self._simIncToPulses(speed_code)
        self.cmd_chain += cmd_string

    @execWrap
    def setStartSpeed(self, pulses_per_sec):
        """ Set start speed in `pulses_per_sec` [50-1000] """
        self.logCall('setStartSpeed', locals())

        cmd_string = 'v{0}'.format(pulses_per_sec)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    @execWrap
    def setTopSpeed(self, pulses_per_sec):
        """ Set top speed in `pulses_per_sec` [5-6000] """
        self.logCall('setTopSpeed', locals())

        cmd_string = 'V{0}'.format(pulses_per_sec)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    @execWrap
    def setCutoffSpeed(self, pulses_per_sec):
        """ Set cutoff speed in `pulses_per_sec` [50-2700] """
        self.logCall('setCutoffSpeed', locals())

        cmd_string = 'c{0}'.format(pulses_per_sec)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    @execWrap
    def setSlope(self, slope_code, chain=False):
        self.logCall('setSlope', locals())

        if not 1 <= slope_code <= 20:
            raise(ValueError('`slope_code` [{0}] must be between 1 and 20'
                             ''.format(slope_code)))
        cmd_string = 'L{0}'.format(slope_code)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    #########################################################################
    # Report commands (cannot be chained)                                   #
    #########################################################################

    def updateSpeeds(self):
        self.logCall('updateSpeeds', locals())

        self.getStartSpeed()
        self.getTopSpeed()
        self.getCutoffSpeed()

    def getPlungerPos(self):
        """ Returns the absolute plunger position as an int (0-3000) """
        self.logCall('getPlungerPos', locals())

        cmd_string = '?'
        data = self.sendRcv(cmd_string)
        self.state['plunger_pos'] = int(data)
        return self.state['plunger_pos']

    def getStartSpeed(self):
        """ Returns the start speed as an int (in pulses/sec) """
        self.logCall('getStartSpeed', locals())

        cmd_string = '?1'
        data = self.sendRcv(cmd_string)
        self.state['start_speed'] = int(data)
        return self.state['start_speed']

    def getTopSpeed(self):
        """ Returns the top speed as an int (in pulses/sec) """
        self.logCall('getTopSpeed', locals())

        cmd_string = '?2'
        data = self.sendRcv(cmd_string)
        self.state['top_speed'] = int(data)
        return self.state['top_speed']

    def getCutoffSpeed(self):
        """ Returns the cutoff speed as an int (in pulses/sec) """
        self.logCall('getCutoffSpeed', locals())

        cmd_string = '?3'
        data = self.sendRcv(cmd_string)
        self.state['cutoff_speed'] = int(data)
        return self.state['cutoff_speed']

    def getEncoderPos(self):
        """ Returns the current encoder count on the plunger axis """
        self.logCall('getEncoderPos', locals())

        cmd_string = '?4'
        data = self.sendRcv(cmd_string)
        return int(data)

    def getCurPort(self):
        """ Returns the current port position (1-num_ports) """
        self.logCall('getCurPort', locals())

        cmd_string = '?6'
        data = self.sendRcv(cmd_string)
        with self._syringeErrorHandler():
            try:
                port = int(data)
            except ValueError:
                raise SyringeError(7, self.__class__.ERROR_DICT)
            self.state['port'] = port
            return port

    #########################################################################
    # Chainable functions                                         #
    #########################################################################

    @execWrap
    def changePort(self, to_port, clockwise=None):
        """
        Change port to `to_port`. If `clockwise` is None, it
        will be calculated to minimize travel time. `clockwise` may also be
        provided directly.

        Args:
            `to_port` (int) : port to which to change
        Kwargs:
            `clockwise` : direction of valve movement
                None  - calculated by current port and `to_port`
                True  - clockwise
                False - counterclockwise

        """
        self.logCall('changePort', locals())

        if not 0 < to_port <= self.num_ports:
            raise(ValueError('`to_port` [{0}] must be between 1 and '
                             '`num_ports` [{1}]'.format(to_port,
                             self.num_ports)))
        
        if self.sim_state['port']:
                from_port = self.sim_state['port']
        else:
                from_port = self.init_port

        # calculate the travel time
        diff = to_port - from_port
        if diff >= 0:
            cw_delta = diff
            ccw_delta = self.num_ports - diff
        else:
            cw_delta = self.num_ports + diff
            ccw_delta = -diff

        # if `clockwise` is None, choose rotation direction
        if clockwise is None:
            if cw_delta <= ccw_delta:
                clockwise = True
            else:
                clockwise = False
        
        # select `delta` and `port_cmd`
        if clockwise:
            port_cmd = "I"
            delta = cw_delta
        else:
            port_cmd = "O"
            delta = ccw_delta

        cmd_string = '{0}{1}'.format(port_cmd,
                                     to_port)
        self.sim_state['port'] = to_port
        self.cmd_chain += cmd_string
        self.exec_time += 0.02 * delta
        self.exec_time += 0.1

    @execWrap
    def movePlungerAbs(self, abs_position):
        """
        Moves the plunger to absolute position `abs_position`

        Args:
            `abs_position` (int) : absolute plunger position
                (0-24000) in microstep mode
                (0-3000) in standard mode

        """
        self.logCall('movePlungerAbs', locals())

        if self.sim_state['microstep']:
            if not 0 <= abs_position <= 24000:
                raise(ValueError('`abs_position` must be between 0 and 40000'
                                 ' when operating in microstep mode'.format(
                                 self.port_num)))
        else:
            if not 0 <= abs_position <= 3000:
                raise(ValueError('`abs_position` must be between 0 and 40000'
                                 ' when operating in microstep mode'.format(
                                 self.port_num)))
        cmd_string = 'A{0}'.format(abs_position)
        cur_pos = self.sim_state['plunger_pos']
        delta_pos = cur_pos-abs_position
        self.sim_state['plunger_pos'] = abs_position
        self.cmd_chain += cmd_string
        self.exec_time += self._calcPlungerMoveTime(abs(delta_pos))

    @execWrap
    def movePlungerRel(self, rel_position):
        """
        Moves the plunger to relative position `rel_position`. There is no
        front-end error handling -- invalid relative moves will result in
        error code 3 from the XCalibur, raising a `SyringeError`

        Args:
            `rel_position` (int) : relative plunger position
                if rel_position < 0 : plunger moves up (relative dispense)
                if rel_position > 0 : plunger moves down (relative extract)

        """
        self.logCall('movePlungerRel', locals())

        if rel_position < 0:
            cmd_string = 'D{0}'.format(abs(rel_position))
        else:
            cmd_string = 'P{0}'.format(rel_position)
        self.sim_state['plunger_pos'] += rel_position
        self.cmd_chain += cmd_string
        self.exec_time += self._calcPlungerMoveTime(abs(rel_position))

    @execWrap
    def repeatCmdSeq(self, num_repeats):
        self.logCall('repeatCmdSeq', locals())

        if not 0 < num_repeats < 30000:
            raise(ValueError('`num_repeats` [{0}] must be between 0 and 30000'
                             ''.format(num_repeats)))
        cmd_string = 'G{0}'.format(num_repeats)
        self.cmd_chain += cmd_string
        self.exec_time *= num_repeats

    @execWrap
    def markRepeatStart(self):
        self.logCall('markRepeatStart', locals())

        cmd_string = 'g'
        self.cmd_chain += cmd_string

    @execWrap
    def delayExec(self, delay_ms):
        """ Delays command execution for `delay` milliseconds """
        self.logCall('delayExec', locals())

        if not 0 < delay_ms < 30000:
            raise(ValueError('`delay` [{0}] must be between 0 and 40000 ms'
                             ''.format(delay_ms)))
        cmd_string = 'M{0}'.format(delay_ms)
        self.cmd_chain += cmd_string
        self.exec_time += delay_ms/1000.0

    #########################################################################
    # Control commands                                                      #
    #########################################################################

    @execWrap
    def haltExec(self):
        """
        Used within a command string to halt execution until another [R]
        command is sent, or until TTL pin goes low
        """
        self.logCall('haltExec', locals())

        cmd_string = 'H'
        return self.sendRcv(cmd_string)

    def terminateCmd(self):
        self.logCall('terminateCommand', locals())

        cmd_string = 'T'
        return self.sendRcv(cmd_string, execute=True)

    #########################################################################
    # Communication handlers and special functions                          #
    #########################################################################

    @contextmanager
    def _syringeErrorHandler(self):
        """
        Context manager to handle `SyringeError` based on error code. Right
        now this just handles error codes 7, 9, and 10 by initializing the
        pump and then re-running the previous command.

        """
        try:
            yield
        except SyringeError as e:
            self.logDebug('ErrorHandler: caught error code {}'.format(
                          e.err_code))
            if e.err_code in [7, 9, 10]:
                last_cmd = self.last_cmd
                self.resetChain()
                try:
                    self.logDebug('ErrorHandler: attempting re-init')
                    self.init()
                except SyringeError as e:
                    self.logDebug('ErrorHandler: Error during re-init '
                                  '[{}]'.format(e.err_code))
                    if e.err_code in [7, 9, 10]:
                        pass
                    else:
                        raise e
                self._waitReady()
                self.logDebug('ErrorHandler: resending last command {} '
                              ''.format(last_cmd))
                self.sendRcv(last_cmd)
            else:
                self.logDebug('ErrorHandler: error not in [7, 9, 10], '
                              're-raising [{}]'.format(e.err_code))
                self.resetChain()
                raise e
        except Exception as e:
            self.resetChain()
            raise e

    def waitReady(self, timeout=10, polling_interval=0.3, delay=None):
        """
        Waits a maximum of `timeout` seconds for the syringe to be
        ready to accept another set command, polling every `polling_interval`
        seconds. If a `delay` is provided, the function will sleep `delay`
        seconds prior to beginning polling.

        """
        self.logCall('waitReady', locals())
        with self._syringeErrorHandler():
            self._waitReady(timeout=timeout, polling_interval=polling_interval,
                            delay=delay)


    def _calcPlungerMoveTime(self, move_steps):
        """
        Calculates plunger move time using equations provided by Tecan.
        Assumes that all input values have been validated

        """
        sd = self.sim_state
        start_speed = sd['start_speed']
        top_speed = sd['top_speed']
        cutoff_speed = sd['cutoff_speed']
        slope = sd['slope']
        microstep = sd['microstep']

        slope *= 2500.0
        if microstep:
            move_steps = move_steps / 8.0
        theo_top_speed = sqrt((4.0 * move_steps*slope) + start_speed ** 2.0)
        # If theoretical top speed will not exceed cutoff speed
        if theo_top_speed < cutoff_speed:
            move_t = theo_top_speed - (start_speed/slope)
        else:
            theo_top_speed = sqrt(((2.0*move_steps*slope) +
                             ((start_speed**2.0+cutoff_speed**2.0)/2.0)))
        # If theoretical top speed with exceed cutoff speed but not
        # reach the set top speed
        if cutoff_speed < theo_top_speed < top_speed:
            move_t = ((1 / slope) * (2.0 * theo_top_speed - start_speed -
                     cutoff_speed))
        # If start speed, top speed, and cutoff speed are all the same
        elif start_speed == top_speed == cutoff_speed:
            move_t = (2.0 * move_steps) / top_speed
        # Otherwise, calculate time spent in each phase (start, constant,
        # ramp down)
        else:
            ramp_up_halfsteps = ((top_speed ** 2.0 - start_speed ** 2.0) /
                                (2.0 * slope))
            ramp_down_halfsteps = ((top_speed ** 2.0 - cutoff_speed ** 2.0) /
                                  (2.0 * slope))
            if (ramp_up_halfsteps + ramp_down_halfsteps) < (2.0 * top_speed):
                ramp_up_t = (top_speed - start_speed) / slope
                ramp_down_t = (top_speed - cutoff_speed) / slope
                constant_halfsteps = (2.0 * move_steps - ramp_up_halfsteps -
                                     ramp_down_halfsteps)
                constant_t = constant_halfsteps / top_speed
                move_t = ramp_up_t + ramp_down_t + constant_t
        return move_t

    def _ulToSteps(self, volume_ul, microstep=None):
        """
        Converts a volume in microliters (ul) to encoder steps.

        Args:
            `volume_ul` (int) : volume in microliters
        Kwargs:
            `microstep` (bool) : whether to convert to standard steps or
                                 microsteps

        """
        if not microstep: microstep = self.state['microstep']
        if microstep:
            steps = volume_ul * (24000/self.syringe_ul)
        else:
            steps = volume_ul * (3000/self.syringe_ul)
        return steps

    def _simIncToPulses(self, speed_inc):
        """
        Updates simulation speeds given a speed increment setting (`speed_inc`)
        following XCalibur handling of speed changes (i.e. cutoff speed cannot
        be higher than top speed, so it is automatically adjusted on the pump)

        """
        top_speed = self.__class__.SPEED_CODES[speed_inc]
        self.sim_state['top_speed'] = top_speed
        if self.sim_state['start_speed'] > top_speed:
            self.sim_state['start_speed'] = top_speed
        if self.sim_state['cutoff_speed'] > top_speed:
            self.sim_state['cutoff_speed'] = top_speed

    def _calcInitForce(self):
        if self.syringe_ul <= 100:
            return 2
        elif self.syringe_ul <= 500:
            return 1
        else:
            return 0