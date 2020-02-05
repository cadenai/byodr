#!/usr/bin/env python
import argparse
import json
import logging
import select
import sys
import termios
import threading
import time
import traceback
import tty
import webbrowser
from functools import partial

import pygame
import websocket

logger = logging.getLogger(__name__)


class AttrDict(dict):
    """ Access dictionary items through instance attributes."""

    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self


class DebugAttr(AttrDict):
    def __init__(self, *args, **kwargs):
        super(DebugAttr, self).__init__(*args, **kwargs)
        self.debug1 = kwargs.get('debug1')
        self.debug2 = kwargs.get('debug2')
        self.debug3 = kwargs.get('debug3')
        self.debug4 = kwargs.get('debug4')
        self.debug5 = kwargs.get('debug5')
        self.debug6 = kwargs.get('debug6')
        self.debug7 = kwargs.get('debug7')


class NavigationAttr(AttrDict):
    def __init__(self, *args, **kwargs):
        super(NavigationAttr, self).__init__(*args, **kwargs)
        self.image_features = kwargs.get('image_features')
        self.route = kwargs.get('route')
        self.point = kwargs.get('point')
        self.similarity = kwargs.get('similarity')
        self.debug1 = kwargs.get('debug1')


class Blob(AttrDict):
    def __init__(self, *args, **kwargs):
        super(Blob, self).__init__(*args, **kwargs)
        self.client_command = kwargs.get('client_command')
        self.control = kwargs.get('control')
        self.steering_driver = kwargs.get('steering_driver')
        self.speed_driver = kwargs.get('speed_driver')
        self.forced_steering = kwargs.get('forced_steering')
        self.forced_throttle = kwargs.get('forced_throttle')
        self.forced_acceleration = kwargs.get('forced_acceleration')
        self.forced_deceleration = kwargs.get('forced_deceleration')
        self.recorder_active = kwargs.get('recorder_active')
        self.recorder_mode = kwargs.get('recorder_mode')
        self.button_event = kwargs.get('button_event')
        self.save_event = kwargs.get('save_event')
        self.steering = kwargs.get('steering')
        self.throttle = kwargs.get('throttle')
        self.reversed = kwargs.get('reversed')
        self.timestamp = kwargs.get('timestamp')
        self.velocity_x = kwargs.get('velocity_x')
        self.velocity_y = kwargs.get('velocity_y')
        self.x_coordinate = kwargs.get('x_coordinate')
        self.y_coordinate = kwargs.get('y_coordinate')
        self.road_speed = kwargs.get('road_speed')
        self.desired_speed = kwargs.get('desired_speed')
        self.heading = kwargs.get('heading')
        self.turn_direction = kwargs.get('turn_direction')
        self.norm_heading = kwargs.get('norm_heading')
        self.publish = kwargs.get('publish')
        #
        self.debug = kwargs.get('debug', DebugAttr())
        self.navigation = kwargs.get('navigation', NavigationAttr())


class Control(object):
    CTL_LAST = 0
    CTL_RADIO = 1
    CTL_JOYSTICK = 2
    CTL_STATIC_CRUISE_CONTROL = 3
    CTL_ADAPTIVE_CRUISE_CONTROL = 4
    CTL_DNN_DRIVER = 5
    CTL_AUTOPILOT = 6
    CTL_DNN_DAGGER = 7


class Term(object):
    def __init__(self, timeout=0.):
        self._stdin_settings = termios.tcgetattr(sys.stdin)
        self._timeout = timeout

    def get_key(self):
        tty.setcbreak(sys.stdin.fileno())
        r_list, _, _ = select.select([sys.stdin], [], [], self._timeout)
        _key = '-1'
        if r_list:
            _key = sys.stdin.read(1)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)
        return _key


class JoystickEvent(object):
    def __init__(self, **kwargs):
        self.steering = kwargs.get('axis_steering', 0)
        self.throttle = kwargs.get('axis_throttle', 0)
        self.button_back = kwargs.get('button_back', 0)
        self.button_start = kwargs.get('button_start', 0)
        self.button_left = kwargs.get('button_left', 0)
        self.button_right = kwargs.get('button_right', 0)
        self.button_a = kwargs.get('button_a', 0)
        self.button_b = kwargs.get('button_b', 0)
        self.button_x = kwargs.get('button_x', 0)
        self.button_y = kwargs.get('button_y', 0)
        self.arrow_up = kwargs.get('arrow_up', 0)
        self.arrow_down = kwargs.get('arrow_down', 0)
        self.arrow_left = kwargs.get('arrow_left', 0)
        self.arrow_right = kwargs.get('arrow_right', 0)
        self._empty = sum(list(kwargs.values())) == 0

    def is_empty(self):
        return self._empty


class NoJoystickError(BaseException):
    def __init__(self, *args):
        super(NoJoystickError, self).__init__(*args)


class Keyboard(object):
    def __init__(self):
        pass

    @staticmethod
    def get_event(key_press=None):
        return JoystickEvent(
            button_y=1 if key_press == 'y' else 0,
            button_x=1 if key_press == 'x' else 0,
            button_b=1 if key_press == ' ' else 0,
            button_left=1 if key_press == 'i' else 0,
            button_back=1 if key_press == 'o' else 0,
            button_right=1 if key_press == 'p' else 0,
            axis_steering=-.3 if key_press == 'a' else .3 if key_press == 'd' else 0
        )


class XboxController(object):
    AXIS_STEERING, AXIS_THROTTLE, AXIS_BRAKES = 2, 4, 5
    ARROW_UP, ARROW_DOWN, ARROW_LEFT, ARROW_RIGHT = 0, 1, 2, 3
    BUTTON_BACK, BUTTON_START, BUTTON_LEFT, BUTTON_RIGHT, BUTTON_A, BUTTON_B, BUTTON_X, BUTTON_Y = 5, 4, 8, 9, 11, 12, 13, 14
    BUTTON_X_BOX = 10

    def __init__(self, steering_scale=.6):
        super(XboxController, self).__init__()
        if pygame.joystick.get_count() < 1:
            raise NoJoystickError("No joystick found.")
        self._steering_scale = steering_scale
        self._controller = pygame.joystick.Joystick(0)
        self._controller.init()
        logger.info('Initialized: %s' % self._controller.get_name())

    def _get_throttle(self):
        _accelerator = (self._controller.get_axis(self.AXIS_THROTTLE) + 1.) / 2.
        _brakes = (self._controller.get_axis(self.AXIS_BRAKES) + 1.) / 2.
        return -1. * _brakes if _brakes > 1e-2 else _accelerator

    @staticmethod
    def _collapse(v, zone):
        return 0. if abs(v) <= zone else v - zone if v > 0. else v + zone

    def _get_steering(self):
        # This is a property of the x-box controller.
        _threshold = .18
        _scale = self._steering_scale
        steering = self._controller.get_axis(self.AXIS_STEERING)
        return self._collapse(steering, zone=_threshold) * _scale if abs(steering) > _threshold else 0

    def get_event(self, _):
        return JoystickEvent(
            arrow_up=self._controller.get_button(XboxController.ARROW_UP),
            arrow_down=self._controller.get_button(XboxController.ARROW_DOWN),
            arrow_left=self._controller.get_button(XboxController.ARROW_LEFT),
            arrow_right=self._controller.get_button(XboxController.ARROW_RIGHT),
            axis_steering=self._get_steering(),
            axis_throttle=self._get_throttle(),
            button_back=self._controller.get_button(XboxController.BUTTON_X_BOX),
            button_start=self._controller.get_button(XboxController.BUTTON_START),
            button_left=self._controller.get_button(XboxController.BUTTON_LEFT),
            button_right=self._controller.get_button(XboxController.BUTTON_RIGHT),
            button_a=self._controller.get_button(XboxController.BUTTON_A),
            button_b=self._controller.get_button(XboxController.BUTTON_B),
            button_x=self._controller.get_button(XboxController.BUTTON_X),
            button_y=self._controller.get_button(XboxController.BUTTON_Y)
        )


class GTController(object):
    AXIS_STEERING, AXIS_THROTTLE, AXIS_BRAKE = 0, 1, 2

    BUTTON_EX = 0
    BUTTON_SQU = 1
    BUTTON_CIR = 2
    BUTTON_TRI = 3

    BUTTON_GT = 19
    BUTTON_PS = 20
    BUTTON_ENTER = 14
    BUTTON_PLUS, BUTTON_MINUS = 15, 18

    BUTTON_L2, BUTTON_R2, BUTTON_L3, BUTTON_R3 = 7, 6, 11, 10

    def __init__(self, steering_scale=3.3):
        super(GTController, self).__init__()
        if pygame.joystick.get_count() < 1:
            raise NoJoystickError("No wheel found.")
        self._steering_scale = steering_scale
        self._controller = pygame.joystick.Joystick(0)
        self._controller.init()
        logger.info('Initialized: %s' % self._controller.get_name())

    def _get_steering(self):
        _threshold = .003
        _scale = self._steering_scale
        steering = self._controller.get_axis(self.AXIS_STEERING)
        return steering * _scale if abs(steering) > _threshold else 0

    def _get_throttle(self):
        # Pedal up position yields 1 and down -1.
        thr_th = .025
        # Assume driver uses one pedal at a time.
        throttle = (self._controller.get_axis(self.AXIS_THROTTLE) - 1.) / -2.
        throttle -= (self._controller.get_axis(self.AXIS_BRAKE) - 1.) / -2.
        return throttle if abs(throttle) > thr_th else 0

    def get_event(self, _):
        return JoystickEvent(
            arrow_up=self._controller.get_button(GTController.BUTTON_PLUS),
            arrow_down=self._controller.get_button(GTController.BUTTON_MINUS),
            arrow_left=self._controller.get_button(GTController.BUTTON_L2),
            arrow_right=self._controller.get_button(GTController.BUTTON_R2),
            axis_steering=self._get_steering(),
            axis_throttle=self._get_throttle(),
            button_back=self._controller.get_button(GTController.BUTTON_GT),
            button_start=self._controller.get_button(GTController.BUTTON_PS),
            button_left=self._controller.get_button(GTController.BUTTON_L3),
            button_right=self._controller.get_button(GTController.BUTTON_R3),
            button_a=self._controller.get_button(GTController.BUTTON_EX),
            button_b=self._controller.get_button(GTController.BUTTON_CIR),
            button_x=self._controller.get_button(GTController.BUTTON_SQU),
            button_y=self._controller.get_button(GTController.BUTTON_TRI)
        )


class G920Controller(object):
    AXIS_STEERING, AXIS_THROTTLE, AXIS_BRAKE = 0, 1, 2
    BUTTON_A, BUTTON_B, BUTTON_X, BUTTON_Y = 0, 1, 2, 3
    BUTTON_RIGHT_PADDLE, BUTTON_LEFT_PADDLE, BUTTON_MENU, BUTTON_SELECT = 4, 5, 6, 7
    BUTTON_RSB, BUTTON_LSB, BUTTON_XBOX = 8, 9, 10
    HAT_ARROWS = 0

    def __init__(self, steering_scale=2.5):
        super(G920Controller, self).__init__()
        if pygame.joystick.get_count() < 1:
            raise NoJoystickError("No wheel found.")
        self._steering_scale = steering_scale
        self._controller = pygame.joystick.Joystick(0)
        self._controller.init()
        logger.info('Initialized: %s' % self._controller.get_name())

    def _get_steering(self):
        _threshold = .008
        _scale = self._steering_scale
        steering = self._controller.get_axis(self.AXIS_STEERING)
        return steering * _scale if abs(steering) > _threshold else 0

    def _get_throttle(self):
        # Pedal up position yields 1 and down -1.
        thr_th = .001
        break_factor_first_half = 1. + (1. / 3.)
        break_factor_second_half = 4.
        # Assume driver uses one pedal at a time.
        # The brake pedal is made to be more powerful early on.
        # For the first half of the pedal depth the braking power goes from 0 to 75%.
        # For the last half the braking power increases from 75% to 100%
        throttle = (self._controller.get_axis(self.AXIS_THROTTLE) - 1.) / -2.
        if self._controller.get_axis(self.AXIS_BRAKE) > 0.:
            throttle -= (self._controller.get_axis(self.AXIS_BRAKE) - 1.) / -break_factor_first_half
        else:
            throttle -= (self._controller.get_axis(self.AXIS_BRAKE) - 3.) / -break_factor_second_half
        return throttle if abs(throttle) > thr_th else 0

    def get_event(self, _):
        return JoystickEvent(
            arrow_up=self._controller.get_button(G920Controller.BUTTON_RIGHT_PADDLE),
            arrow_down=self._controller.get_button(G920Controller.BUTTON_LEFT_PADDLE),
            arrow_left=1 if (self._controller.get_hat(G920Controller.HAT_ARROWS)[0] == -1) else 0,
            arrow_right=1 if (self._controller.get_hat(G920Controller.HAT_ARROWS)[0] == 1) else 0,
            axis_steering=self._get_steering(),
            axis_throttle=self._get_throttle(),
            button_back=self._controller.get_button(G920Controller.BUTTON_XBOX),
            button_start=self._controller.get_button(G920Controller.BUTTON_MENU),
            button_left=self._controller.get_button(G920Controller.BUTTON_LSB),
            button_right=self._controller.get_button(G920Controller.BUTTON_RSB),
            button_a=self._controller.get_button(G920Controller.BUTTON_A),
            button_b=self._controller.get_button(G920Controller.BUTTON_B),
            button_x=self._controller.get_button(G920Controller.BUTTON_X),
            button_y=self._controller.get_button(G920Controller.BUTTON_Y)
        )


class WebSocketThread(threading.Thread):
    def __init__(self, connect_string,
                 fn_open=(lambda _: None),
                 fn_message=(lambda x, y: None),
                 fn_error=(lambda x, y: None)):
        super(WebSocketThread, self).__init__()
        self.socket = websocket.WebSocketApp(connect_string, on_message=fn_message, on_error=fn_error)
        self.socket.on_open = fn_open
        self._quit_event = threading.Event()

    def send(self, data):
        try:
            self.socket.send(data)
        except IOError as e:
            logger.error(e)
        except websocket.WebSocketConnectionClosedException:
            pass

    def run(self):
        while not self._quit_event.is_set():
            self.socket.run_forever()

    def quit(self):
        self._quit_event.set()
        self.socket.close()


class MessageParser(object):
    def __init__(self):
        # I18n
        pass

    @staticmethod
    def to_float(v, default=0.):
        return default if v is None else float(v)

    @staticmethod
    def to_int(v, default=0):
        return default if v is None else int(v)

    @staticmethod
    def to_bool(v, default=False):
        return default if v is None else bool(v)

    @staticmethod
    def to_string(v, default='n/a', str_format='{:2.2f}'):
        return default if v is None else str_format.format(v)

    @staticmethod
    def str_turn_short(turn):
        return 'L' if turn == 'intersection.left' \
            else 'R' if turn == 'intersection.right' \
            else 'X' if turn == 'intersection.ahead' \
            else ' '

    @staticmethod
    def str_turn_long(turn):
        return 'take the next left' if turn == 'intersection.left' \
            else 'take the next right' if turn == 'intersection.right' \
            else 'straight ahead'

    @staticmethod
    def get_control(state):
        return state['ctl'] if (state is not None and 'ctl' in state) else None

    def get_steering(self, state):
        return self.to_float(state['ste']) if (state is not None and 'ste' in state) else 0

    def to_blob(self, state):
        # Deserialize the state.
        blob = Blob()
        blob.control = {None: "n/a",
                        Control.CTL_LAST: "n/a",
                        Control.CTL_RADIO: "radio",
                        Control.CTL_JOYSTICK: "joystick",
                        Control.CTL_STATIC_CRUISE_CONTROL: "static_cruise",
                        Control.CTL_ADAPTIVE_CRUISE_CONTROL: "adaptive_cruise",
                        Control.CTL_DNN_DRIVER: "dnn_driver",
                        Control.CTL_DNN_DAGGER: "dnn_dagger",
                        Control.CTL_AUTOPILOT: "autopilot"}[self.get_control(state)]
        blob.recorder_mode = {None: "n/a",
                              -999: "n/a",
                              551: "driving",
                              594: "interventions"}[state['rec_mod']]
        blob.recorder_mode = "{} {}".format(blob.recorder_mode, "[ON]" if state['rec_act'] else "(off)")
        blob.steering = self.get_steering(state)
        blob.throttle = self.to_float(state['thr'])
        blob.reversed = self.to_bool(state['rev'])
        blob.velocity_y = self.to_float(state['vel_y'])
        blob.debug.debug1 = self.to_float(state['debug1'])
        blob.debug.debug2 = self.to_float(state['debug2'])
        blob.debug.debug3 = self.to_float(state['debug3'])
        blob.debug.debug4 = self.to_float(state['debug4'])
        blob.debug.debug5 = self.to_float(state['debug5'])
        blob.debug.debug6 = self.to_float(state['debug6'])
        blob.debug.debug7 = self.to_float(state['debug7'])
        blob.x_coordinate = self.to_float(state['x'])
        blob.y_coordinate = self.to_float(state['y'])
        blob.desired_speed = self.to_float(state['speed'])
        blob.road_speed = self.to_float(state['max_speed'])
        blob.heading = self.to_string(state['head'], str_format='{:5.1f}')
        blob.navigation.route = "n/a" if state['route'] is None else state['route']
        blob.navigation.point = "n/a" if state['route_np'] is None else state['route_np']
        blob.navigation.similarity = self.to_float(state['route_np_sim'])
        blob.navigation.debug1 = self.to_float(state['route_np_debug1'])
        blob.turn_direction = state['turn']
        return blob


class ConsoleDisplay(object):
    DEBUG, ROUTING, PRETTY = 0, 1, 2
    next_mode = {DEBUG: ROUTING, ROUTING: PRETTY, PRETTY: DEBUG}

    def __init__(self, calibration_mode=False):
        super(ConsoleDisplay, self).__init__()
        self.calibration_mode = calibration_mode
        self._parser = MessageParser()
        self._display = ConsoleDisplay.ROUTING
        self._lock = threading.Lock()

    def print_help(self, clear=True):
        # Clears the screen.
        if clear:
            print(chr(27) + "[2J"),
        if self.calibration_mode:
            print(
                """
                  'q'       to quit
                  '<'       shift the steering calibration value to the left
                  '>'       shift the steering calibration value to the right
                  'h'       print this help
                """)
        else:
            print(
                """
                  'q'       to quit
                  'm'       switch display mode
                  'k'       reset the environment and set the console steering calibration value to zero
                  '3/4'     navigator - next route - next point
                  '8/9/0'   reload weights
                  '+/-'     increase/decrease speed
                  'h'       print this help
                """)

    def switch_display(self):
        with self._lock:
            self._display = self.next_mode[self._display]

    def display(self, state):
        blob = self._parser.to_blob(state)
        mode = self._display
        if mode == ConsoleDisplay.ROUTING:
            turn = self._parser.str_turn_short(blob.turn_direction)
            blob_str = "\r({:+2.3f}, {:+2.3f}) [vy={:+2.2f} T={:s}]  " \
                       "1: {:2.2f}  2: {:2.2f}  ds: {:+2.2f}  " \
                       "route: {:20s} np: {:10s} {:2.2f} {:2.2f}  ctl: {:12s}  rec: {:12s} {:30s}" \
                .format(blob.steering, blob.throttle, blob.velocity_y, turn,
                        blob.debug.debug1, blob.debug.debug2, blob.desired_speed,
                        blob.navigation.route, blob.navigation.point, blob.navigation.similarity, blob.navigation.debug1,
                        blob.control, blob.recorder_mode, ""
                        )
        elif mode == ConsoleDisplay.DEBUG:
            turn = self._parser.str_turn_short(blob.turn_direction)
            blob_str = "\r({:+2.3f}, {:+2.3f}) [vy={:+2.2f} T={:s}]  " \
                       "mu: {:2.2f}  ob: {:2.2f}  sum: {:2.2f}  _surprise: {:+2.5f}  _critic: {:+2.5f}  " \
                       "_brake: {:+2.4f}   _entropy: {:+2.5f} {:40s}" \
                .format(blob.steering, blob.throttle, blob.velocity_y, turn,
                        blob.debug.debug1, blob.debug.debug2, blob.debug.debug3, blob.debug.debug4,
                        blob.debug.debug5, blob.debug.debug6, blob.debug.debug7, ""
                        )
        elif mode == ConsoleDisplay.PRETTY:
            # Convert meters per second to kilometers per hour.
            speed_display_scale = 3.6
            turn = self._parser.str_turn_long(blob.turn_direction)
            steering = blob.steering
            road_speed = blob.road_speed * speed_display_scale
            desired_speed = blob.desired_speed * speed_display_scale
            str_left = "\rCaden input = {:s} at max. {:2.0f} km/h.".format(turn, road_speed)
            str_right = "Caden output = steering: {:+2.3f} throttle: {:+2.2f} target speed: {:2.1f} km/h.{:20s}" \
                .format(steering, blob.throttle, desired_speed, "")
            blob_str = "{:100s}{:s}".format(str_left, str_right)
        else:
            raise AssertionError("Unknown mode {}".format(mode))

        print(blob_str),
        sys.stdout.flush()


class CommunicationProtocol(object):
    _STEER_CALIBRATION_STEP = .005

    def __init__(self, calibration_mode=False):
        super(CommunicationProtocol, self).__init__()
        self.calibration_mode = calibration_mode
        self._steer_calibration_shift = .0

    def steer_calibrate_left(self):
        if self.calibration_mode:
            self._steer_calibration_shift -= self._STEER_CALIBRATION_STEP

    def steer_calibrate_right(self):
        if self.calibration_mode:
            self._steer_calibration_shift += self._STEER_CALIBRATION_STEP

    def payload(self, event=None, **kwargs):
        payload = {} if event is None else {
            'steering': max(-1, min(1, event.steering + self._steer_calibration_shift)),
            'throttle': event.throttle
        }
        if kwargs is not None:
            payload.update(kwargs)

        # Handle the recording buttons separately.
        # Nothing much is supported in calibration mode to prevent steering errors.
        if not self.calibration_mode and event is not None:
            if event.button_back == 1:
                payload['button_back'] = 1
            elif event.button_start == 1:
                payload['button_start'] = 1
            elif event.button_left == 1:
                payload['button_left'] = 1
            elif event.button_right == 1:
                payload['button_right'] = 1
            elif event.button_a == 1:
                payload['button_a'] = 1
            elif event.button_b == 1:
                payload['button_b'] = 1
            elif event.button_x == 1:
                payload['button_x'] = 1
            elif event.button_y == 1:
                payload['button_y'] = 1
            elif event.arrow_up == 1:
                payload['arrow_up'] = 1
            elif event.arrow_down == 1:
                payload['arrow_down'] = 1
            elif event.arrow_left == 1:
                payload['arrow_left'] = 1
            elif event.arrow_right == 1:
                payload['arrow_right'] = 1
        return payload


class CommunicationClient(object):

    def __init__(self, fn_create_controller, display, ws_host='localhost', ws_port=9001, sleep=0.05, calibration_mode=False):
        self._controller = None
        self._fn_create_controller = fn_create_controller
        self._display = display
        self._protocol = CommunicationProtocol(calibration_mode=calibration_mode)
        self._term = Term(timeout=sleep)
        self._quit_event = threading.Event()
        self._control_socket = WebSocketThread(connect_string="ws://{}:{}/ws/ctl".format(ws_host, ws_port),
                                               fn_message=self._on_ctl_message,
                                               fn_error=(lambda x, y: time.sleep(.01)))
        self._log_socket = WebSocketThread(connect_string="ws://{}:{}/ws/log".format(ws_host, ws_port),
                                           fn_open=(lambda ws: ws.send('0')),
                                           fn_message=self._on_log_message,
                                           fn_error=(lambda x, y: time.sleep(.01)))
        self._control_socket.start()
        self._log_socket.start()
        self._on_ctl_open('')

    def _on_ctl_open(self, _):
        try:
            self._controller = self._fn_create_controller()
        except NoJoystickError as e:
            logger.error(e)
            self.quit()
        self._display.print_help(clear=False)

    def _on_log_message(self, ws, msg):
        try:
            msg = json.loads(msg)
            if len(msg) > 0:
                self._display.display(msg)
            ws.send('{}')
        except Exception as e:
            logger.error("Log socket error {} msg={}.".format(e, msg))
            logger.error("Trace: {}".format(traceback.format_exc(e)))

    def _on_ctl_message(self, ws, msg):
        pass

    def quit(self):
        self._quit_event.set()
        self._log_socket.quit()
        self._control_socket.quit()

    def is_running(self):
        return not self._quit_event.is_set()

    def publish(self):
        if self.is_running() and self._controller is not None:
            ws = self._control_socket
            # Use the timeout on the terminal as a throttle for this method.
            # Ch value is -1 on key timeout reached.
            _c = self._term.get_key()
            kwargs = {}
            if _c == 'q':
                self.quit()
                return
            if _c == 'm':
                self._display.switch_display()
            elif _c == 'k':
                kwargs['reset'] = 1
            elif _c == '<':
                self._protocol.steer_calibrate_left()
            elif _c == '>':
                self._protocol.steer_calibrate_right()
            elif _c == '3':
                kwargs['nav_next_route'] = 1
            elif _c == '4':
                kwargs['nav_next_point'] = 1
            elif _c in ('8', '9', '0'):
                kwargs['reload_weights'] = 1
            elif _c == '-':
                kwargs['cc_speed'] = 'less'
            elif _c == '=':
                kwargs['cc_speed'] = 'more'
            elif _c == '1':
                kwargs['autopilot'] = 1
            elif _c == 'h':
                self._display.print_help()
            _event = self._controller.get_event(_c)
            ws.send(json.dumps(self._protocol.payload(_event, **kwargs)))


def _create_controller(name, steering_scale):
    if name in ('G920', 'g920', '920'):
        controller = G920Controller() if steering_scale is None else G920Controller(steering_scale=steering_scale)
    elif name in ('GT', 'gt', 'Gt', 'wheel'):
        controller = GTController() if steering_scale is None else GTController(steering_scale=steering_scale)
    elif name in ('x-box', 'xbox'):
        controller = XboxController() if steering_scale is None else XboxController(steering_scale=steering_scale)
    else:
        controller = Keyboard()
    return controller


def main():
    parser = argparse.ArgumentParser(description='Teleop client.')
    parser.add_argument('--interface', '-i', type=str, default='localhost', help='Host ip')
    parser.add_argument('--port', '-p', type=int, default=9100, help='Websocket port')
    parser.add_argument('--console', '-c', type=str, default='keys', help='Console (keys, x-box, wheel, g920)')
    parser.add_argument('--scale_steering', '-s', type=float, default=None, help='Scale the steering output of the controller')
    parser.add_argument('--browser', type=str, default='chrome', help="Which browser - chrome, firefox or safari or none to skip.")
    parser.add_argument('--calibration', default=False, type=lambda x: (str(x).lower() == 'true'))
    args = parser.parse_args()

    socket_client = None
    try:
        pygame.init()
        fn_create_controller = partial(_create_controller, name=args.console, steering_scale=args.scale_steering)
        websocket.setdefaulttimeout(2)
        socket_client = CommunicationClient(
            fn_create_controller=fn_create_controller,
            display=ConsoleDisplay(calibration_mode=args.calibration),
            ws_host=args.interface,
            ws_port=args.port,
            sleep=0.010,
            calibration_mode=args.calibration
        )
        if args.browser not in (None, 'None', 'none', 'null', 'false'):
            p_browser = webbrowser.get() if str(args.browser).lower() == 'true' else webbrowser.get(args.browser)
            p_browser.open_new_tab('http://{}:{}'.format(args.interface, args.port))
        # Enter the main loop.
        while socket_client.is_running():
            events = pygame.event.get()
            pygame.event.pump()
            socket_client.publish()
            for event in events:
                if event.type == pygame.QUIT:
                    return
    except KeyboardInterrupt:
        logger.info("^C")
    except NoJoystickError as no_joy:
        logger.error(no_joy)
    finally:
        if socket_client is not None:
            socket_client.quit()
        pygame.quit()


if __name__ == "__main__":
    logging.captureWarnings(True)
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    logging.getLogger('py.warnings').setLevel(logging.ERROR)
    main()
