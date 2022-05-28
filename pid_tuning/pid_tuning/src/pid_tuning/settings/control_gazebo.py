#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest, GetModelState, GetModelStateRequest
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import time

class ControlGazebo(object):
    def __init__(self):
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        service_name = '/gazebo/set_physics_properties'
        rospy.loginfo("Esperando por servicio"+str(service_name))
        rospy.wait_for_service(service_name) 
        rospy.loginfo("Servicio encontrado"+str(service_name))
        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
    def init_values(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException:
            print("Llamada al servicio /gazebo/reset_simulation ha fallado")
        self._time_step = Float64(0.001)
        self._max_update_rate = Float64(1000.0)
        self._gravity = Vector3()
        self._gravity.x = 0.0
        self._gravity.y = 0.0
        self._gravity.z = -9.81
        self._ode_config = ODEPhysics()
        self._ode_config.auto_disable_bodies = False
        self._ode_config.sor_pgs_precon_iters = 0
        self._ode_config.sor_pgs_iters = 50
        self._ode_config.sor_pgs_w = 1.3
        self._ode_config.sor_pgs_rms_error_tol = 0.0
        self._ode_config.contact_surface_layer = 0.001
        self._ode_config.contact_max_correcting_vel = 0.0
        self._ode_config.cfm = 0.0
        self._ode_config.erp = 0.2
        self._ode_config.max_contacts = 20
        self.update()
    def update(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException:
            print("llamada a servicio /gazebo/pause_physics ha fallado")
        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self._time_step.data
        set_physics_request.max_update_rate = self._max_update_rate.data
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config
        result = self.set_physics(set_physics_request)
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException:
            print("Llamada al servicio /gazebo/unpause_physics ha fallado")   
    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException:
            print("llamada a servicio /gazebo/pause_physics ha fallado")
    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException:
            print("Llamada al servicio /gazebo/unpause_physics ha fallado") 