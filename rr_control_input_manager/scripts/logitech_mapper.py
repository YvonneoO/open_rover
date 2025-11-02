#!/usr/bin/env python3
# Logitech F710 mapper (XInput or DirectInput) -> /cmd_vel/joystick (TwistStamped)
# Buttons A/B/X/Y are latched on /joystick/{a,b,x,y}_button (Bool, latch=True)
# Mode: ~mode:=x (XInput, xbox/xpad-style) or ~mode:=d (DirectInput table) https://wiki.ros.org/joy

# left joystick up = forward (+x)
# left joystick down = backward (-x)
# right joystick left = turn left (-z)
# right joystick right = turn right (+z)
# DPAD up = speed up (throttle +)
# DPAD down = speed down (throttle -)

import time
import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from actionlib_msgs.msg import GoalID


class LogitechMapperNode:
    def __init__(self):
        rospy.init_node('logitech_mapper_node', anonymous=True)

        # params
        self.mode = rospy.get_param('~mode', 'x').lower()
        self.joy_topic = rospy.get_param('~joy_topic', 'joystick')
        self.max_vel_fwd = rospy.get_param('~max_vel_drive', 2.6)
        self.max_vel_trn = rospy.get_param('~max_vel_turn', 9.0)
        self.max_vel_flp = rospy.get_param('~max_vel_flipper', 1.4)
        self.drive_throttle = rospy.get_param('~default_drive_throttle', 0.15)
        self.flipper_throttle = rospy.get_param('~default_flipper_throttle', 0.6)
        self.adj_throttle = rospy.get_param('~adjustable_throttle', True)
        self.drive_incr = 20.0
        self.flipper_incr = 20.0

        # mapping
        m = self.setup_mapping(self.mode)
        self.triggers_are_axes = m['TRIGGERS_ARE_AXES']
        self.LSx, self.LSy = m['L_STICK_H_AXES'], m['L_STICK_V_AXES']
        self.RSx, self.RSy = m['R_STICK_H_AXES'], m['R_STICK_V_AXES']
        self.DPx, self.DPy = m['DPAD_H_AXES'], m['DPAD_V_AXES']
        self.A, self.B, self.X, self.Y = m['A_BUTTON'], m['B_BUTTON'], m['X_BUTTON'], m['Y_BUTTON']
        self.LB, self.RB = m['LB_BUTTON'], m['RB_BUTTON']
        self.BACK = m['BACK_BUTTON']
        self.START = m['START_BUTTON']
        # triggers (axes or buttons)
        self.LT_AX, self.RT_AX = m['L_TRIG_AXES'], m['R_TRIG_AXES']
        self.LT_BTN, self.RT_BTN = m['LT_BUTTON'], m['RT_BUTTON']

        # state
        self.seq = 0
        self.dpad_active = False
        self.last_a = self.last_b = self.last_x = self.last_y = self.last_back = self.last_start = time.time()
        self.prev_fwd = 0.0
        self.prev_trn = 0.0
        self.FWD_ACC_LIM = 0.2
        self.TRN_ACC_LIM = 0.4

        # publishers
        self.pub_cmd = rospy.Publisher('/cmd_vel/joystick', TwistStamped, queue_size=3)
        self.pub_delay = rospy.Publisher('/joystick/delay', Float32, queue_size=3)
        self.pub_movebase_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.pub_a = rospy.Publisher('/joystick/a_button', Bool, queue_size=1, latch=True)
        self.pub_b = rospy.Publisher('/joystick/b_button', Bool, queue_size=1, latch=True)
        self.pub_x = rospy.Publisher('/joystick/x_button', Bool, queue_size=1, latch=True)
        self.pub_y = rospy.Publisher('/joystick/y_button', Bool, queue_size=1, latch=True)
        self.pub_back = rospy.Publisher('/joystick/back_button', Bool, queue_size=1, latch=True)
        self.pub_start = rospy.Publisher('/joystick/start_button', Bool, queue_size=1, latch=True)

        # publish initial latched states (False)
        self.pub_a.publish(Bool(data=False))
        self.pub_b.publish(Bool(data=False))
        self.pub_x.publish(Bool(data=False))
        self.pub_y.publish(Bool(data=False))
        self.pub_back.publish(Bool(data=False))
        self.pub_start.publish(Bool(data=False))

        # subscriber
        rospy.Subscriber(self.joy_topic, Joy, self.joy_cb)

    def setup_mapping(self, mode: str):
        if mode == 'x':
            rospy.logwarn("LOGITECH F710: XInput mode (xpad-style mapping)")
            return dict(
                TRIGGERS_ARE_AXES=True,
                L_STICK_H_AXES=0, L_STICK_V_AXES=1,
                L_TRIG_AXES=2, R_STICK_H_AXES=3, R_STICK_V_AXES=4, R_TRIG_AXES=5,
                DPAD_H_AXES=6, DPAD_V_AXES=7,
                A_BUTTON=0, B_BUTTON=1, X_BUTTON=2, Y_BUTTON=3, LB_BUTTON=4, RB_BUTTON=5,
                BACK_BUTTON=6, START_BUTTON=7,  # BACK and START buttons
                LT_BUTTON=6, RT_BUTTON=7,  # not used in x mode but defined for completeness
            )
        else:
            rospy.logwarn("LOGITECH F710: DirectInput mode (ros wiki)")
            return dict(
                TRIGGERS_ARE_AXES=False,
                L_STICK_H_AXES=0, L_STICK_V_AXES=1,
                R_STICK_H_AXES=2, R_STICK_V_AXES=3,
                DPAD_H_AXES=4, DPAD_V_AXES=5,
                X_BUTTON=0, A_BUTTON=1, B_BUTTON=2, Y_BUTTON=3, LB_BUTTON=4, RB_BUTTON=5,
                BACK_BUTTON=6, START_BUTTON=7,  # BACK and START buttons
                LT_BUTTON=6, RT_BUTTON=7,
                L_TRIG_AXES=2, R_TRIG_AXES=5,  # not used in d mode
            )

    def limit_acc(self, fwd, trn):
        df, dt = fwd - self.prev_fwd, trn - self.prev_trn
        if df > self.FWD_ACC_LIM: fwd = self.prev_fwd + self.FWD_ACC_LIM
        elif df < -self.FWD_ACC_LIM: fwd = self.prev_fwd - self.FWD_ACC_LIM
        if dt > self.TRN_ACC_LIM: trn = self.prev_trn + self.TRN_ACC_LIM
        elif dt < -self.TRN_ACC_LIM: trn = self.prev_trn - self.TRN_ACC_LIM
        self.prev_fwd, self.prev_trn = fwd, trn
        return fwd, trn

    def joy_cb(self, msg: Joy):
        # latency report (control_input_manager uses header.stamp timing)
        cmd_time = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9
        self.pub_delay.publish(Float32(data=(time.time() - cmd_time)))

        # latched A/B/X/Y (debounced 0.5s)
        now = time.time()
        if msg.buttons[self.A] == 1 and now - self.last_a > 0.5:
            self.last_a = now; self.pub_a.publish(Bool(data=True)); rospy.loginfo("User button A")
        if msg.buttons[self.B] == 1 and now - self.last_b > 0.5:
            self.last_b = now; self.pub_b.publish(Bool(data=True)); rospy.loginfo("User button B")
        if msg.buttons[self.X] == 1 and now - self.last_x > 0.5:
            self.last_x = now; self.pub_x.publish(Bool(data=True)); rospy.loginfo("User button X")
        if msg.buttons[self.Y] == 1 and now - self.last_y > 0.5:
            self.last_y = now; self.pub_y.publish(Bool(data=True)); rospy.loginfo("User button Y")
        if msg.buttons[self.BACK] == 1 and now - self.last_back > 0.5:
            self.last_back = now; self.pub_back.publish(Bool(data=True)); rospy.loginfo("User button BACK")
        if msg.buttons[self.START] == 1 and now - self.last_start > 0.5:
            self.last_start = now; self.pub_start.publish(Bool(data=True)); rospy.loginfo("User button START")

        # throttle tuning via dpad + LB/RB
        if self.adj_throttle:
            if int(msg.axes[self.DPy]) == 1 and not self.dpad_active:
                self.drive_throttle += 1.0 / self.drive_incr; self.dpad_active = True
            if int(msg.axes[self.DPy]) == -1 and not self.dpad_active:
                self.drive_throttle -= 1.0 / self.drive_incr; self.dpad_active = True

            if msg.buttons[self.LB] == 1:
                self.flipper_throttle -= 1.0 / self.flipper_incr; rospy.loginfo(self.flipper_throttle)
            if msg.buttons[self.RB] == 1:
                self.flipper_throttle += 1.0 / self.flipper_incr; rospy.loginfo(self.flipper_throttle)

            # clamp + reset dpad latch
            self.drive_throttle = min(1.0, max(self.drive_throttle, 1.0 / self.drive_incr))
            self.flipper_throttle = min(1.0, max(self.flipper_throttle, 1.0 / self.flipper_incr))
            if (msg.axes[self.DPy], msg.axes[self.DPx]) == (0, 0):
                self.dpad_active = False

            fwd_db = 0.2 * self.drive_throttle * self.max_vel_fwd
            trn_db = 0.2 * self.drive_throttle * self.max_vel_trn
            flp_db = 0.2 * self.flipper_throttle * self.max_vel_flp
        else:
            fwd_db = 0.2 * self.max_vel_fwd
            trn_db = 0.2 * self.max_vel_trn
            flp_db = 0.2 * self.max_vel_flp

        # compute commands
        drive = self.drive_throttle * self.max_vel_fwd * msg.axes[self.LSy]
        if -fwd_db < drive < fwd_db: drive = 0.0

        turn = (1.1 - (drive / self.max_vel_fwd if self.max_vel_fwd else 0.0)) \
               * self.drive_throttle * self.max_vel_trn * - msg.axes[self.RSx]
        if -trn_db < turn < trn_db: turn = 0.0

        if self.triggers_are_axes:
            flipper = (self.flipper_throttle * self.max_vel_flp * msg.axes[self.LT_AX]) - \
                      (self.flipper_throttle * self.max_vel_flp * msg.axes[self.RT_AX])
        else:
            flipper = 0.0
            if msg.buttons[self.LT_BTN] == 1: flipper -= self.flipper_throttle * self.max_vel_flp
            if msg.buttons[self.RT_BTN] == 1: flipper += self.flipper_throttle * self.max_vel_flp
        if -flp_db < flipper < flp_db: flipper = 0.0

        # Optional accel limit
        # drive, turn = self.limit_acc(drive, turn)

        # publish TwistStamped to /cmd_vel/joystick
        out = TwistStamped()
        out.header.seq = self.seq
        out.header.stamp = rospy.Time.now()
        out.twist.linear.x = drive
        out.twist.angular.y = flipper
        out.twist.angular.z = turn
        self.pub_cmd.publish(out)
        self.seq += 1

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = LogitechMapperNode()
    node.spin()
