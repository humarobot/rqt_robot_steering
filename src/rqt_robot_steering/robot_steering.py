# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
import rospkg

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget
from rqt_gui_py.plugin import Plugin


class RobotSteering(Plugin):

    slider_factor = 1000.0

    def __init__(self, context):
        super(RobotSteering, self).__init__(context)
        self.setObjectName('RobotSteering')

        self._publisher = None

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('rqt_robot_steering'), 'resource', 'RobotSteering.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RobotSteeringUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._widget.topic_line_edit.textChanged.connect(
            self._on_topic_changed)
        self._widget.stop_push_button.pressed.connect(self._on_stop_pressed)

        self._widget.x_slider.valueChanged.connect(
            self._on_x_slider_changed)
        self._widget.y_slider.valueChanged.connect(
            self._on_y_slider_changed)
        self._widget.z_slider.valueChanged.connect(
            self._on_z_slider_changed)
        self._widget.roll_slider.valueChanged.connect(
            self._on_roll_slider_changed)
        self._widget.pitch_slider.valueChanged.connect(
            self._on_pitch_slider_changed)
        self._widget.yaw_slider.valueChanged.connect(
            self._on_yaw_slider_changed)

        # timer to consecutively send twist messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)

    @Slot(str)
    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        if topic == '':
            return
        try:
            self._publisher = rospy.Publisher(
                topic, PoseStamped, queue_size=10)
        except TypeError:
            self._publisher = rospy.Publisher(topic, PoseStamped)

    def _on_stop_pressed(self):
        # TODO: set default value here.
        # If the current value of sliders is zero directly send stop twist msg
        if self._widget.z_angular_slider.value() == 0:
            self.zero_cmd_sent = False
            self._on_parameter_changed()
        else:
            self._widget.z_angular_slider.setValue(0)

    def _on_x_slider_changed(self):
        # self._widget.current_joint_1_angular_label.setText(
        #     '%0.2f rad/s' % (self._widget.z_angular_slider.value() / RobotSteering.slider_factor))
        self._on_parameter_changed()

    def _on_y_slider_changed(self):
        self._on_parameter_changed()

    def _on_z_slider_changed(self):
        self._on_parameter_changed()

    def _on_roll_slider_changed(self):
        self._on_parameter_changed()

    def _on_pitch_slider_changed(self):
        self._on_parameter_changed()

    def _on_yaw_slider_changed(self):
        self._on_parameter_changed()

    def _on_parameter_changed(self):
        x = self._widget.x_slider.value() / RobotSteering.slider_factor
        y = self._widget.y_slider.value() / RobotSteering.slider_factor
        z = self._widget.z_slider.value() / RobotSteering.slider_factor
        roll = self._widget.roll_slider.value() / RobotSteering.slider_factor
        pitch = self._widget.pitch_slider.value() / RobotSteering.slider_factor
        yaw = self._widget.yaw_slider.value() / RobotSteering.slider_factor
        self._send_posestamped(x, y, z, roll, pitch, yaw)

    def _send_posestamped(self, x, y, z, roll, pitch, yaw):
        if self._publisher is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self._publisher.publish(msg)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value(
            'topic', self._widget.topic_line_edit.text())

    def restore_settings(self, plugin_settings, instance_settings):
        value = instance_settings.value('topic', '/legged_robot_EE_pose')
        # value = rospy.get_param('~default_topic', value)
        self._widget.topic_line_edit.setText(value)
