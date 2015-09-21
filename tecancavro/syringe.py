import time

try:
    from gevent import monkey; monkey.patch_all(thread=False)
    from gevent import sleep
except:
    from time import sleep

import logging
from functools import wraps

class SyringeError(Exception):
    """
    Error raise when a Cavro pump returns a non-zero error code.

    Args:
        `error_code` (int): the error code returned by the cavro pump
        `error_dict` (dict): dictionary of model-specific error msgs, keyed
                             by error code
    """

    def __init__(self, error_code, error_dict):
        super(SyringeError, self).__init__(self)
        self.err_code = error_code
        try:
            err_str = error_dict[error_code]
            self.err_msg = '{0} [{1}]'.format(err_str, self.err_code)
        except KeyError:
            self.err_msg = 'Unknown Error [{0}]'.format(error_code)

    def __str__(self):
        return self.err_msg


class SyringeTimeout(Exception):
    """ Raised when a syringe wait command times out """
    pass


class Syringe(object):
    """
    General syringe class that may be subclassed for specific syringe models
    or advanced functionality.
    """

    ERROR_DICT = {
        1: 'Initialization Error',
        2: 'Invalid Command',
        3: 'Invalid Operand',
        4: 'Invalid Command Sequence',
        6: 'EEPROM Failure',
        7: 'Device Not Initialized',
        9: 'Plunger Overload',
        10: 'Valve Overload',
        11: 'Plunger Move Not Allowed',
        15: 'Command Overflow'
    }

    def __init__(self, com_link, debug=False, debug_log_path='.'):
        self.com_link = com_link
        self._ready = False
        self._prev_error_code = 0
        self._repeat_error = False

        # Handle debug mode init
        self.debug = debug
        if self.debug:
            self.initDebugLogging(debug_log_path)

        # Command chaining state information
        self.cmd_chain = ''
        self.exec_time = 0

    def _sendRcv(self, cmd_string):
        response = self.com_link.sendRcv(cmd_string)
        ready = self._checkStatus(response['status_byte'])[0]
        data = response['data']
        return data, ready

    def _checkStatus(self, status_byte):
        """
        Parses a bit string representation of a Tecan API status byte for
        potential error codes (and subsequently raises `SyringeError`) and
        returns the status code as a boolean (True = ready, False = busy).

        Defaults to the error code dictionary (`ERROR_DICT`) defined in the
        `Syringe` class; however, this can be overridden in a subclass.

        """
        error_code = int(status_byte[4:8], 2)
        ready = int(status_byte[2])
        if ready == 1:
            self._ready = True
        else:
            self._ready = False
        if error_code == self._prev_error_code:
            self._repeat_error = True
        else:
            self._repeat_error = False
        self._prev_error_code = error_code
        if error_code != 0:
            error_dict = self.__class__.ERROR_DICT
            raise SyringeError(error_code, error_dict)
        return ready, error_code

    def _checkReady(self):
        """
        Checks to see if the syringe is ready to accept a new command (i.e.
        is not busy). Returns `True` if it is ready, or `False` if it is not.

        """
        if self._ready:
            return True
        try:
            ready = self._sendRcv('Q')[1]
            return ready
        except SyringeError as e:
            if self._repeat_error:
                return self._ready
            else:
                raise e

    def _waitReady(self, polling_interval=0.3, timeout=10, delay=None):
        """
        Waits for the syringe to be ready to accept a command

        Kwargs:
            `polling_interval` (int): frequency of polling in seconds
            `timeout` (int): max wait time in seconds

        """
        if delay:
            sleep(delay)
        start = time.time()
        while (start-time.time()) < (start+timeout):
            ready = self._checkReady()
            if not ready:
                sleep(polling_interval)
            else:
                return
        raise(SyringeTimeout('Timeout while waiting for syringe to be ready'
                             ' to accept commands [{}]'.format(timeout)))

    #########################################################################
    # Debug functions                                                       #
    #########################################################################

    def initDebugLogging(self, debug_log_path):
        """ Initialize logger and log file handler """

        self.logger = logging.getLogger(self.__class__.__name__)
        fp = debug_log_path.rstrip("/") + "/" + self.__class__.__name__ + '_debug.log'
        hdlr = logging.FileHandler(fp)
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr)
        self.logger.setLevel(logging.DEBUG)

    def logCall(self, f_name, f_locals):
        """ Logs function params at call """

        if self.debug:
            self.logger.debug('-> {}: {}'.format(f_name, f_locals))

    def logDebug(self, msg):
        """ Handles debug logging if self.debug == True """

        if self.debug:
            self.logger.debug(msg)

    #########################################################################
    # Command chain functions                                               #
    #########################################################################

    def sendRcv(self, cmd_string, execute=False):
        """
        Send a raw command string and return a tuple containing the parsed
        response data: (Data, Ready). If the syringe is ready to accept
        another command, `Ready` with be 'True'.

        Args:
            `cmd_string` (bytestring) : a valid Tecan XCalibur command string
        Kwargs:
            `execute` : if 'True', the execute byte ('R') is appended to the
                        `cmd_string` prior to sending
        Returns:
            `parsed_reponse` (tuple) : parsed pump response tuple

        """
        self.logCall('sendRcv', locals())

        if execute:
            cmd_string += 'R'
        self.last_cmd = cmd_string
        self.logDebug('sendRcv: sending cmd_string: {}'.format(cmd_string))
        with self._syringeErrorHandler():
            parsed_response = self._sendRcv(cmd_string)
            self.logDebug('sendRcv: received response: {}'.format(
                          parsed_response))
            data = parsed_response[0]
            return data

    def executeChain(self, wait_ready=False):
        """
        Executes and resets the current command chain (`self.cmd_chain`).
        Returns the estimated execution time (`self.exec_time`) for the chain.

        """
        self.logCall('executeChain', locals())

        # Compensate for reset time (tic/toc) prior to returning extra_wait_time
        tic = time.time()
        self.sendRcv(self.cmd_chain, execute=True)
        exec_time = self.exec_time
        self.resetChain(on_execute=True, wait_ready=wait_ready)
        if wait_ready:
            extra_wait_time = 0
        else:
            toc = time.time()
            extra_wait_time = exec_time - (toc-tic)
            if extra_wait_time < 0:
                extra_wait_time = 0
        return extra_wait_time

    def resetChain(self, on_execute=False, wait_ready=False):
        """
        Resets the command chain (`self.cmd_chain`) and execution time
        (`self.exec_time`). Optionally updates `slope` and `microstep`
        state variables, speeds, and simulation state.

        Kwargs:
            `on_execute` (bool) : should be used to indicate whether or not
                                  the chain being reset was executed, which
                                  will cue slope and microstep state
                                  updating (as well as speed updating).
        """
        self.logCall('resetChain', locals())

        if wait_ready:
            self._waitReady(delay = self.exec_time)

        self.cmd_chain = ''
        self.exec_time = 0

def execWrap(func):
    """
    Decorator to wrap chainable commands, allowing for immediate execution
    of the wrapped command by passing in an `execute=True` kwarg.

    """
    @wraps(func)
    def addAndExec(self, *args, **kwargs):
        execute = False
        if 'execute' in kwargs:
            execute = kwargs.pop('execute')
        if 'wait_ready' in kwargs:
            wait_ready = kwargs.pop('wait_ready')
        else:
            wait_ready = False
        func(self, *args, **kwargs)
        if execute:
            return self.executeChain(wait_ready=wait_ready)
    return addAndExec
