import argparse
import glob
import logging
import os

import rclpy
from actionlib_msgs.msg import GoalID
from configparser import ConfigParser as SafeConfigParser
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Float32

from byodr.utils import Application, timestamp
from byodr.utils.ipc import CollectorThread, JSONReceiver, JSONPublisher


class Bridge(Node):
    def __init__(self, node_name, get_pilot_message, publish_internal):
        super().__init__('byodr1' if node_name is None else node_name)
        self._get_pilot = get_pilot_message
        self._publish_internal = publish_internal
        self.create_subscription(GoalID, '{}/v10/pilot/set_mode'.format(self.get_name()), self._receive_mode, 10)
        self.create_subscription(Float32, '{}/v10/pilot/set_maximum_speed'.format(self.get_name()), self._receive_max_speed, 10)
        self._ros_publisher = self.create_publisher(DiagnosticArray, '{}/v10/pilot/diagnostics'.format(self.get_name()), 10)
        # The timer period is specified in seconds.
        self.timer = self.create_timer(1. / 10, self._publish_diagnostics)

    def _publish_diagnostics(self):
        pilot = self._get_pilot()
        if pilot:
            status = DiagnosticStatus(name='mode', message='not_available')
            status.values = [
                KeyValue(key='steer', value='{:+2.5f}'.format(pilot.get('steering'))),
                KeyValue(key='throttle', value='{:+2.5f}'.format(pilot.get('throttle')))
            ]
            driver_mode = pilot.get('driver')
            if driver_mode == 'driver_mode.teleop.direct':
                status.message = 'teleoperation'
            elif driver_mode == 'driver_mode.inference.dnn':
                status.message = 'autopilot'
                status.values += [
                    KeyValue(key='maximum_speed', value='{:+2.5f}'.format(pilot.get('cruise_speed'))),
                    KeyValue(key='desired_speed', value='{:+2.5f}'.format(pilot.get('desired_speed')))
                ]
            diagnostics = DiagnosticArray()
            diagnostics.header.stamp = self.get_clock().now().to_msg()
            diagnostics.status = [status]
            self._ros_publisher.publish(diagnostics)

    def _send_internal_command(self, cmd):
        if cmd:
            cmd['time'] = timestamp()
            self._publish_internal(cmd)

    def _receive_mode(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        internal = dict()
        if msg.id == 'teleoperation':
            internal['pilot.driver.set'] = 'driver_mode.teleop.direct'
        elif msg.id == 'autopilot':
            internal['pilot.driver.set'] = 'driver_mode.inference.dnn'
        self._send_internal_command(internal)

    def _receive_max_speed(self, msg):
        internal = dict()
        # The value may equal 0 (zero) and bool(0) evaluates to False.
        if msg.data is not None:
            internal['pilot.maximum.speed'] = msg.data
            self._send_internal_command(internal)


class RosApplication(Application):
    def __init__(self, config_dir=os.getcwd()):
        # Just check for restarts - no need for high frequency processing.
        super(RosApplication, self).__init__(run_hz=1)
        self._config_dir = config_dir
        self._bridge = None
        self.pilot = None
        self.publish = None
        self.ipc_chatter = None

    def _config(self):
        parser = SafeConfigParser()
        [parser.read(_f) for _f in ['config.ini'] + glob.glob(os.path.join(self._config_dir, '*.ini'))]
        return dict(parser.items('ros2'))

    def setup(self):
        if self.active():
            if self._bridge is not None:
                self._bridge.destroy_node()
            bridge = Bridge(node_name=self._config().get('rover.node.name'), get_pilot_message=self.pilot, publish_internal=self.publish)
            rclpy.spin(bridge)
            self._bridge = bridge

    def finish(self):
        if self._bridge is not None:
            self._bridge.destroy_node()

    def step(self):
        chat = self.ipc_chatter()
        if chat and chat.get('command') == 'restart':
            self.setup()


def main():
    parser = argparse.ArgumentParser(description='ROS2 rover node.')
    parser.add_argument('--config', type=str, default='/config', help='Config directory path.')
    args = parser.parse_args()

    application = RosApplication(config_dir=args.config)
    quit_event = application.quit_event
    logger = application.logger

    publisher = JSONPublisher(url='ipc:///byodr/ros.sock', topic='aav/ros/input')
    pilot = JSONReceiver(url='ipc:///byodr/pilot.sock', topic=b'aav/pilot/output')
    ipc_chatter = JSONReceiver(url='ipc:///byodr/teleop_c.sock', topic=b'aav/teleop/chatter', pop=True)
    collector = CollectorThread(receivers=(pilot, ipc_chatter), event=quit_event, hz=20)
    application.publish = lambda m: publisher.publish(m)
    application.pilot = lambda: collector.get(0)
    application.ipc_chatter = lambda: collector.get(1)
    threads = [collector]
    if quit_event.is_set():
        return 0

    rclpy.init()
    [t.start() for t in threads]
    application.run()

    logger.info("Waiting on threads to stop.")
    [t.join() for t in threads]
    rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s: %(filename)s %(funcName)s %(message)s')
    logging.getLogger().setLevel(logging.INFO)
    main()
