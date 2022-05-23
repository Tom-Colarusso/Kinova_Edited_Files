#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
from std_msgs.msg import Bool
from kortex_driver.srv import *
from kortex_driver.msg import *
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.InterconnectConfigClientRpc import InterconnectConfigClient

from kortex_api.autogen.messages import Common_pb2
from kortex_api.autogen.messages import InterconnectConfig_pb2
import argparse

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

def parseConnectionArguments(parser = argparse.ArgumentParser()):
    parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.10")
    parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
    parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
    return parser.parse_args([])

class DeviceConnection:

    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args):
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args):
        """
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):

        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):

        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000

            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()


class GpioBridge:
    '''
    Implements methods for establishing and operating GPIO bridge through
    the base
    '''
    GpioEnum = (
        InterconnectConfig_pb2.GPIO_IDENTIFIER_1
    )

    def __init__(self, router):

        self.router = router

        self.device_manager = DeviceManagerClient(self.router)
        self.interconnect_config = InterconnectConfigClient(self.router)

        self.interconnect_device_id = self.GetDeviceIdFromDevType(Common_pb2.INTERCONNECT, 0)
        if (self.interconnect_device_id is None):
            print ("Could not find the Interconnect in the device list, exiting...")
            sys.exit(0)

    def GetDeviceIdFromDevType(self, device_type, device_index = 0):
        devices = self.device_manager.ReadAllDevices()

        current_index = 0
        for device in devices.device_handle:
            if device.device_type == device_type:
                if current_index == device_index:
                    print ("Found the Interconnect on device identifier {}".format(device.device_identifier))
                    return device.device_identifier
                current_index += 1
        return None

    def InitGpioInputsAndOutputs(self):
        gpio_config                  = InterconnectConfig_pb2.GPIOConfiguration()

        # Pins 1 and 2 as output
        gpio_config.mode             = InterconnectConfig_pb2.GPIO_MODE_OUTPUT_PUSH_PULL
        gpio_config.pull             = InterconnectConfig_pb2.GPIO_PULL_NONE
        gpio_config.default_value    = InterconnectConfig_pb2.GPIO_VALUE_LOW
        gpio_config.identifier       = InterconnectConfig_pb2.GPIO_IDENTIFIER_1
        print ("Setting pin #1 as output...")
        self.interconnect_config.SetGPIOConfiguration(gpio_config, deviceId=self.interconnect_device_id)
        time.sleep(1)
        time.sleep(1)


    def SetOutputPinValue(self, identifier, value):
        gpio_state = InterconnectConfig_pb2.GPIOState()
        gpio_state.identifier = identifier
        gpio_state.value = value
        print ("GPIO pin {} will be put at value {}".format(InterconnectConfig_pb2.GPIOIdentifier.Name(identifier), InterconnectConfig_pb2.GPIOValue.Name(value)))
        self.interconnect_config.SetGPIOState(gpio_state,deviceId=self.interconnect_device_id)


    def ExampleSetAndReadValues(self):
        # We sleep a bit between the reads and the writes
        # Technically the InterconnectConfig service runs at 25ms but we sleep 100ms to make sure we let enough time
        sleep_time_sec = 3

        # The Arduino reads pin 1 and sets pin 3 the same
        # The Arduino reads pin 2 and sets pin 4 the same
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_HIGH)

        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_LOW)
        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_HIGH)

        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_LOW)
        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_HIGH)

        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_LOW)
        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_HIGH)

        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_LOW)
        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_HIGH)

        time.sleep(sleep_time_sec)
        self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_LOW)
    	#def callback(self):
	#self.SetOutputPinValue(InterconnectConfig_pb2.GPIO_IDENTIFIER_1, InterconnectConfig_pb2.GPIO_VALUE_HIGH)

class ExampleFullArmMovement:
    def __init__(self):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_send_joint_angles(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        # Angles to send the arm to vertical position (all zeros)

        angularWaypoint.angles.append(180)
        angularWaypoint.angles.append(-15)
        angularWaypoint.angles.append(215)
        angularWaypoint.angles.append(0)
        angularWaypoint.angles.append(55)
        angularWaypoint.angles.append(90)

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded.
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION) :
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Send the angles
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_joint_angles2(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        # Angles to send the arm to vertical position (all zeros)

        angularWaypoint.angles.append(180)
        angularWaypoint.angles.append(51.88)
        angularWaypoint.angles.append(279.81)
        angularWaypoint.angles.append(0)
        angularWaypoint.angles.append(55)
        angularWaypoint.angles.append(90)

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded.
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION) :
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Send the angles
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()


    def callback(self, data):
        if data.data == 0:
            return
        self.example_send_joint_angles()
    def callback2(self, data):
        if data.data == 0:
            return
        self.example_send_joint_angles2()
    def callback3(self, data):
        if data.data == 0:
            return
        # Parse arguments
        parser = argparse.ArgumentParser()
        args = parseConnectionArguments(parser)

        # Create connection to the device and get the router
        with DeviceConnection.createTcpConnection(args) as router:

            # Create the gpio bridge object. It implements kortex methods used
            # configure and use interconnect's expansion GPIO
            bridge = GpioBridge(router)

            # Configure all interconnect's GPIO as push pull outputs
            #bridge.InitGpioInputsAndOutputs()
            #print ("GPIO bridge object initialized")

            # Example core
            bridge.ExampleSetAndReadValues()

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************

            #*******************************************************************************
            # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action

            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's fully open the gripper

            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            #success &= self.example_set_cartesian_reference_frame()

            # Example of cartesian pose
            # Let's make it move in Z
            #success &= self.example_send_cartesian_pose()
            #*******************************************************************************

            #*******************************************************************************
            # Example of angular position
            # Let's send the arm to vertical position
            #success &= self.example_send_joint_angles()
            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's close the gripper at 50%

            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            #success &= self.example_home_the_robot()
            #*******************************************************************************

            #*******************************************************************************
            # Example of waypoint
            # Let's move the arm
            #success &= self.example_cartesian_waypoint_action()

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            #success &= self.example_home_the_robot()
            #*******************************************************************************


        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)
        rospy.Subscriber("/home", Bool, self.callback)
        rospy.Subscriber("/scan", Bool, self.callback2)
        rospy.Subscriber("/press_button", Bool, self.callback3)
        #rospy.Subscriber("/my_gen3/joint_states", Bool, self.force)
        if not success:
            rospy.logerr("The example encountered an error.")


        rospy.spin()

if __name__ == "__main__":
    ex = ExampleFullArmMovement()
    ex.main()
