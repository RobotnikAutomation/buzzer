#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy 

import time, threading

from robotnik_msgs.msg import State
from robotnik_msgs.srv import set_digital_output, set_digital_outputRequest, SetBuzzer, SetBuzzerResponse

DEFAULT_FREQ = 2.0
MAX_FREQ = 10.0
digital_output_buzzer = 1


# Class Template of Robotnik component for Pyhton
class Buzzer:

    def __init__(self, args):

        self.node_name = rospy.get_name() #.replace('/','')
        self.desired_freq = args['desired_freq'] 
        self.digital_output = args['digital_output']
        self.service_io = args['service_io']
        # Checks value of freq
        if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
            rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
            self.desired_freq = DEFAULT_FREQ


        self.real_freq = 0.0

        # Saves the state of the component
        self.state = State.INIT_STATE
        # Saves the previous state
        self.previous_state = State.INIT_STATE
        # flag to control the initialization of the component
        self.initialized = False
        # flag to control the initialization of ROS stuff
        self.ros_initialized = False
        # flag to control that the control loop is running
        self.running = False
        # Variable used to control the loop frequency
        self.time_sleep = 1.0 / self.desired_freq
        # Variable to control the sound time
        self.time_enabled = float("inf")
        # State msg to publish
        self.msg_state = State()
        # Timer to publish state
        self.publish_state_timer = 1

        # Variable of buzzer
        self.buzzer_enable = False
        self.buzzer_freq = 1
        self.buzzer_called = True


    def setup(self):
        '''
            Initializes de hand
            @return: True if OK, False otherwise
        '''
        self.initialized = True

        return 0


    def rosSetup(self):
        '''
            Creates and inits ROS components
        '''
        if self.ros_initialized:
            return 0

        # Publishers
        self._state_publisher = rospy.Publisher('~state', State, queue_size=10)
        # Service Servers
        rospy.Service('set_buzzer', SetBuzzer, self.setBuzzerSrvCallback)
        # Service Clients
        self.service_digital_output = rospy.ServiceProxy(self.service_io, set_digital_output)

        self.ros_initialized = True

        self.publishROSstate()

        return 0


    def shutdown(self):
        '''
            Shutdowns device
            @return: 0 if it's performed successfully, -1 if there's any problem or the component is running
        '''
        if self.running or not self.initialized:
            return -1
        rospy.loginfo('%s::shutdown'%self.node_name)

        # Cancels current timers
        self.t_publish_state.cancel()

        self._state_publisher.unregister()

        self.initialized = False

        return 0


    def rosShutdown(self):
        '''
            Shutdows all ROS components
            @return: 0 if it's performed successfully, -1 if there's any problem or the component is running
        '''
        if self.running or not self.ros_initialized:
            return -1

        # Performs ROS topics & services shutdown
        self._state_publisher.unregister()

        self.ros_initialized = False

        return 0


    def stop(self):
        '''
            Creates and inits ROS components
        '''
        self.running = False

        return 0


    def start(self):
        '''
            Runs ROS configuration and the main control loop
            @return: 0 if OK
        '''
        self.rosSetup()

        if self.running:
            return 0

        self.running = True

        self.controlLoop()

        return 0


    def controlLoop(self):
        '''
            Main loop of the component
            Manages actions by state
        '''

        while self.running and not rospy.is_shutdown():
            t1 = time.time()

            if self.state == State.INIT_STATE:
                self.initState()

            elif self.state == State.STANDBY_STATE:
                self.standbyState()

            elif self.state == State.READY_STATE:
                self.readyState()

            elif self.state == State.EMERGENCY_STATE:
                self.emergencyState()

            elif self.state == State.FAILURE_STATE:
                self.failureState()

            elif self.state == State.SHUTDOWN_STATE:
                self.shutdownState()

            self.allState()

            t2 = time.time()
            tdiff = (t2 - t1)

            self.time_enabled -= self.time_sleep
            t_sleep = self.time_sleep - tdiff

            if t_sleep > 0.0:
                try:
                    rospy.sleep(t_sleep)
                except rospy.exceptions.ROSInterruptException:
                    rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
                    self.running = False

            t3= time.time()
            self.real_freq = 1.0/(t3 - t1)

        self.running = False
        # Performs component shutdown
        self.shutdownState()
        # Performs ROS shutdown
        self.rosShutdown()
        rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)

        return 0


    def rosPublish(self):
        '''
            Publish topics at standard frequency
        '''

        return 0


    def initState(self):
        '''
            Actions performed in init state
        '''

        if not self.initialized:
            self.setup()

        else:         
            self.switchToState(State.STANDBY_STATE)


        return


    def standbyState(self):
        '''
            Actions performed in standby state
        '''
        self.switchToState(State.READY_STATE)

        return


    def readyState(self):
        '''
            Actions performed in ready state
        '''
        if self.buzzer_enable == True and self.time_enabled > 0:
            if self.buzzer_freq == 0: # constant beep
                self.setDigitalOutput(digital_output_buzzer, True)
            elif self.buzzer_freq != 0 :
                if self.buzzer_called == False:
                    self.setDigitalOutput(digital_output_buzzer, True)
                    self.buzzer_called = True
                elif self.buzzer_called == True:
                    self.setDigitalOutput(digital_output_buzzer, False)
                    self.buzzer_called = False
        else :
            self.setDigitalOutput(digital_output_buzzer, False)


        return


    def shutdownState(self):
        '''
            Actions performed in shutdown state 
        '''
        self.setDigitalOutput(digital_output_buzzer, False)

        if self.shutdown() == 0:
            self.switchToState(State.INIT_STATE)

        return


    def emergencyState(self):
        '''
            Actions performed in emergency state
        '''

        return


    def failureState(self):
        '''
            Actions performed in failure state
        '''


        return


    def switchToState(self, new_state):
        '''
            Performs the change of state
        '''
        if self.state != new_state:
            self.previous_state = self.state
            self.state = new_state
            rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))

        return


    def allState(self):
        '''
            Actions performed in all states
        '''
        self.rosPublish()

        return


    def stateToString(self, state):
        '''
            @param state: state to set
            @type state: State
            @returns the equivalent string of the state
        '''
        if state == State.INIT_STATE:
            return 'INIT_STATE'

        elif state == State.STANDBY_STATE:
            return 'STANDBY_STATE'

        elif state == State.READY_STATE:
            return 'READY_STATE'

        elif state == State.EMERGENCY_STATE:
            return 'EMERGENCY_STATE'

        elif state == State.FAILURE_STATE:
            return 'FAILURE_STATE'

        elif state == State.SHUTDOWN_STATE:
            return 'SHUTDOWN_STATE'
        else:
            return 'UNKNOWN_STATE'


    def publishROSstate(self):
        '''
            Publish the State of the component at the desired frequency
        '''
        self.msg_state.state = self.state
        self.msg_state.state_description = self.stateToString(self.state)
        self.msg_state.desired_freq = self.desired_freq
        self.msg_state.real_freq = self.real_freq
        self._state_publisher.publish(self.msg_state)

        self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
        self.t_publish_state.start()

    """
    def topicCb(self, msg):
        '''
            Callback for inelfe_video_manager state
            @param msg: received message
            @type msg: std_msgs/Int32
        '''
        # DEMO
        rospy.loginfo('Buzzer:topicCb')
    
    def serviceCb(self, req):
        '''
            ROS service server
            @param req: Required action
            @type req: std_srv/Empty
        '''
        # DEMO
        rospy.loginfo('Buzzer:serviceCb')    
    """    
    def setBuzzerSrvCallback(self, req):
        response = SetBuzzerResponse()
        if self.buzzer_freq <= MAX_FREQ : # Intermitent beep start every 2/buzzer_freq. One cycle with beep and one without
            response.ret = True
            response.msg = "Buzzer configuration updated"
            self.buzzer_enable = req.enable
            self.buzzer_freq = req.beep_freq
            if (req.time_enabled == 0):
                self.time_enabled = float("inf")
            else:
                self.time_enabled = req.time_enabled
            if self.buzzer_freq > 0 and req.enable:
                self.time_sleep = 1.0 / (self.buzzer_freq * 2)
            else:
                self.time_sleep = 1.0 / self.desired_freq

        else:
            response.ret = False
            response.msg = 'set_buzzer service: Frequency to high. Limited to '+str(MAX_FREQ)+' Hz'
            self.buzzer_enable = False

        return response


    def setDigitalOutput(self,number, value):
         try:
             request = set_digital_outputRequest()
             request.value = value
             request.output = number
             response = self.service_digital_output(request)
         except rospy.ServiceException, e:
             print("Service call failed")

         return 



def main():

    rospy.init_node("buzzer")


    _name = rospy.get_name().replace('/','')

    arg_defaults = {
    
      'topic_state': 'state',
      'desired_freq': DEFAULT_FREQ,
      'topic_io': "set_digital_output",
      'digital_output': 1

    }

    args = {}

    for name in arg_defaults:
        try:
            if rospy.search_param(name): 
                args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the param has the namespace of the node
            else:
                args[name] = arg_defaults[name]
            #print name
        except rospy.ROSException, e:
            rospy.logerr('%s: %s'%(e, _name))


    rc_node = Buzzer(args)

    rospy.loginfo('%s: starting'%(_name))

    rc_node.start()


if __name__ == "__main__":
    main()
