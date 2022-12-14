#!/usr/bin/env python

# niryo_one_ros_setup.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# This file mimics the beahavior of Niryo robot launch. Thank you.

from __future__ import print_function
from fileinput import FileInput
import rospy
import time
import subprocess
import rospkg
import fnmatch
import os
import shutil

from farol_msgs.msg import ProcessState
from farol_msgs.srv import ManageProcess

PROCESS_TIMEOUT_RESTART = 5.0  # sec


def kill_rosmaster():
    subprocess.Popen(['killall', '-9', 'rosmaster'])


class ProcessNotFound(Exception): pass


class ProcessActionType(object):
    START = 1
    STOP = 2
    RESTART = 3
    KILL = 4
    START_ALL = 5
    STOP_ALL = 6


class Process:

    def __init__(self, name, cmd, vehicle_name, vehicle_id,  config_package_path, folder, namespace, vehicle_configuration=None, args=None, launch_on_startup=False,
                 delay_before_start=0.0, dependencies=None):
        self.name = name
        self.config_package_path = config_package_path
        self.cmd = cmd
        self.folder = folder
        self.namespace = namespace
        self.vehicle_configuration = vehicle_configuration
        self.args = args if args is not None else []
        self.dependencies = dependencies if dependencies is not None else []
        self.launch_on_startup = launch_on_startup
        self.delay_before_start = delay_before_start
        self.process = None
        self.vehicle_name = vehicle_name
        self.vehicle_id = vehicle_id

    def start(self):
        if not self.is_active():
            cmd = self.cmd.split(' ') + self.args + ["name:=" + self.vehicle_name] + ["config_package_path:=" + self.config_package_path] + ["folder:=" + str(self.folder)] + ["namespace:=" + str(self.namespace), "vehicle_id:=" + str(self.vehicle_id)]

            # If we specify a vehicle configuration argument, then add it to the list of commnad arguments
            if self.vehicle_configuration is not None:
                cmd = cmd + ["vehicle_configuration:=" + str(self.vehicle_configuration)]
            if self.delay_before_start:
                rospy.sleep(self.delay_before_start)
            self.process = subprocess.Popen(cmd)


    def restart(self):
        self.stop()
        timeout = time.time() + PROCESS_TIMEOUT_RESTART
        while self.is_active():
            if time.time() > timeout:
                break
            rospy.sleep(0.1)
        self.start()

    def stop(self):
        if self.process:
            self.process.terminate()

    def kill(self):
        if self.process:
            self.process.kill()

    def is_active(self):
        if not self.process:
            return False

        return_code = self.process.poll()
        if return_code is None or return_code < 0:
            return True
        return False


class FarolSetup:

    def __init__(self, vehicle_name, vehicle_id, config_package_path, folder, namespace, vehicle_configuration=None):
        self.vehicle_name = vehicle_name
        self.config_package_path = config_package_path
        self.folder = folder
        self.namespace = namespace
        self.vehicle_configuration = vehicle_configuration
        self.process_list = []
        self.process_config = rospy.get_param("~processes")
        self.vehicle_id = vehicle_id
        self.vehicle_name_id = vehicle_name + self.vehicle_id
        self.create_processes()       

        # +.+ get farol_bringup path directory
        self.rospack = rospkg.RosPack()
        
        # +.+ get repo specific path with configurations
        self.config_specific_path = config_package_path.replace('(find ', '')
        self.config_specific_path = self.config_specific_path.replace(')', '')

        # check if the folder .ros_tmp existe, otherwise create it
        if not (os.path.exists(self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp')):
            os.mkdir(self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp')

        # +.+ copy the permanent_ros.yaml from farol and replace the keyword #vehicle# with the vehicle_name_id
        shutil.copy2(self.rospack.get_path('farol_bringup')+'/config/dev_configs/permanent_ros.yaml', self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp/permanent_ros_' + self.vehicle_name_id + '.yaml')
        self.find_replace(self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp/', 'permanent_ros_' + self.vehicle_name_id + '.yaml' , '#vehicle#' , self.vehicle_name_id)
        
        # +.+ copy the personal_ros.yaml from your bringup and replace the keyword #vehicle# with the vehicle_name_id
        shutil.copy2(self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/personal_ros.yaml', self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp/personal_ros_' + self.vehicle_name_id + '.yaml')
        self.find_replace(self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp/', 'personal_ros_' + self.vehicle_name_id + '.yaml' , '#vehicle#' , self.vehicle_name_id)
    
        rospy.on_shutdown(self.clean_ros_processes)

        self.process_state_publish_rate = rospy.get_param("~process_state_publish_rate")

        self.process_state_publisher = rospy.Publisher(
            '/' + self.vehicle_name_id + '/process_state', ProcessState, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.process_state_publish_rate), self.publish_process_state)

        self.manage_process_server = rospy.Service(
            '/' + self.vehicle_name_id + '/manage_process', ManageProcess, self.callback_manage_process)

        self.start_init_processes()
            
    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def publish_process_state(self, event):
        msg = ProcessState()
        for p in self.process_list:
            msg.name.append(p.name)
            msg.is_active.append(p.is_active())
        self.process_state_publisher.publish(msg)

    def callback_manage_process(self, req):
        process_name = req.name
        action = req.action

        try:
            if action == ProcessActionType.START_ALL:
                self.start_all_processes()
                return self.create_response(200, "All processes have been started")

            if action == ProcessActionType.STOP_ALL:
                self.stop_all_processes()
                return self.create_response(200, "All processes have been stopped")

            if action == ProcessActionType.START:
                self.start_process_from_name(process_name, start_dependencies=True)
                return self.create_response(200, "Process has been started")
            elif action == ProcessActionType.STOP:
                self.stop_process_from_name(process_name)  # also stop processes that depends on this process ?   
                return self.create_response(200, "Process has been stopped")
            elif action == ProcessActionType.RESTART:
                self.restart_process_from_name(process_name)
                return self.create_response(200, "Process has been restarted")
            elif action == ProcessActionType.KILL:
                self.kill_process_from_name(process_name)
                return self.create_response(200, "Process has been killed")

        except ProcessNotFound as e:
            return self.create_response(400, str(e))

    def clean_ros_processes(self):
        # +.+ delete the temprary files created in .ros_tmp folder
        os.remove(self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp/permanent_ros_' + self.vehicle_name_id + '.yaml')
        os.remove(self.rospack.get_path(self.config_specific_path) + '/config/dev_configs/.ros_tmp/personal_ros_' + self.vehicle_name_id + '.yaml')
        self.stop_all_processes()
        kill_rosmaster()

    def create_processes(self):
        rospy.loginfo("Start creating processes from rosparams")
        for p in self.process_config:
            self.process_list.append(Process(name=p['name'], cmd=p['cmd'], args=p['args'],
                                             launch_on_startup=p['launch_on_startup'],
                                             delay_before_start=p['delay_before_start'],
                                             dependencies=p['dependencies'], 
                                             vehicle_name=self.vehicle_name,
                                             vehicle_id=self.vehicle_id,
                                             config_package_path=self.config_package_path, 
                                             folder = self.folder,
                                             namespace=self.namespace,
                                             vehicle_configuration=self.vehicle_configuration))

    def start_init_processes(self):
        for process in self.process_list:
            if process.launch_on_startup:
                self.start_process(process, start_dependencies=True)

    def start_all_processes(self):
        for process in self.process_list:
            self.start_process(process, start_dependencies=True)

    def stop_all_processes(self):
        for p in self.process_list:
            # rospy.loginfo("STOPPING PROCESS : " + str(p.name))
            self.stop_process(p)

    def get_process_from_name(self, name):
        p = None
        for process in self.process_list:
            if process.name == name:
                p = process
                break
        if p is None:
            raise ProcessNotFound("Process not found : " + str(name))
        return p

    def get_dependency_process_list(self, process):
        dep_name_list = process.dependencies
        try:
            return list(map(lambda dep_name: self.get_process_from_name(dep_name), dep_name_list))
        except ProcessNotFound as e:  # should never happen if yaml file is correct
            rospy.logwarn("Some dependency names are incorrect. Check your setup.yaml file to fix it")
            return []

    def are_dependencies_met(self, process):
        process_dep_list = self.get_dependency_process_list(process)
        for p in process_dep_list:
            if not p.is_active():  # is_active doesn't mean all the nodes are fully started (not a problem if nodes wait for services and actions)
                rospy.loginfo("Unmet dependency for " + str(process.name) + " (depends on : " + str(p.name) + ") !")
                return False
        return True

    # CAREFUL : recursion - todo mettre une securite (pas plus de 5 depth)
    def check_and_start_dependencies(self, process):
        process_dep_list = self.get_dependency_process_list(process)
        for p in process_dep_list:
            if not p.is_active():  # is_active doesn't mean all the nodes are fully started (not a problem if nodes wait for services and actions)
                rospy.loginfo("Unmet dependency for " + str(process.name) + " (depends on : " + str(p.name) + ") !")
                rospy.loginfo("Starting dependency process...")
                self.start_process(p, start_dependencies=True)

    def start_process(self, p, start_dependencies=False):
        rospy.loginfo("Handle process : " + str(p.name))
        if start_dependencies:
            self.check_and_start_dependencies(p)
            rospy.loginfo("Start process : " + str(p.name))
            p.start()
        else:
            if self.are_dependencies_met(p):
                rospy.loginfo("Start process : " + str(p.name))
                p.start()

    @staticmethod
    def stop_process(p):
        p.stop()

    @staticmethod
    def restart_process(p):
        p.restart()

    @staticmethod
    def kill_process(p):
        p.kill()

    def start_process_from_name(self, name, start_dependencies=False):
        p = self.get_process_from_name(name)
        self.start_process(p, start_dependencies=start_dependencies)

    def stop_process_from_name(self, name):
        p = self.get_process_from_name(name)
        self.stop_process(p)

    def restart_process_from_name(self, name):
        p = self.get_process_from_name(name)
        self.restart_process(p)

    def kill_process_from_name(self, name):
        p = self.get_process_from_name(name)
        self.kill_process(p)

    def find_replace(self, topdir, file_pattern, text, replacement):
        """
        Replace the string #vehicle# with the desired vehicle name
        :param topdir: directory with rostopics name parameters to be checked
        :param file_pattern: In this case .yaml
        :param text: string to be replaced
        :param replacement: chosen string (vehicle_name)
        """
        # +.+ Check if folder is not empty
        if self.isNotEmpty(topdir):
            for dirpath, dirs, files in os.walk(topdir, topdown=True):
                files = [os.path.join(dirpath, filename)
                        for filename in fnmatch.filter(files, file_pattern)]
                for line in FileInput(files, inplace=True):
                    print(line.replace(text, replacement), end='')
    @staticmethod
    def isNotEmpty(path):
        """
        Check if folder exists and is not empty
        :param path: directory with parameters to be checked
        """
        if os.path.exists(path) and not os.path.isfile(path):
            # Checking if the directory is empty or not
            if not os.listdir(path):
                return False
            else:
                return True
        else:
            return False

if __name__ == '__main__':
    pass
