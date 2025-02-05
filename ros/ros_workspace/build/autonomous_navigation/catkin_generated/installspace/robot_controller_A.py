#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import math
import numpy as np
import traceback
import rospy
import tf

import geometry_msgs.msg as geo
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
from autonomous_navigation.msg import Vector3Array

class TurtlebotController():
    '''This class contains a module of control which follows a trayectory and avoids obstables 
        using its own version of the VFF (Virtual Force Field) algorithm '''
    
    def __init__(self, rate):
        #Initiliaze node
        rospy.init_node('TurtlebotController', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Subscribers 
        rospy.Subscriber("/move_base_simple/goal", geo.PoseStamped, self.goal_Callback)                 #Not compatible while following a path
        rospy.Subscriber('/path', Path ,self.path_Callback)
        rospy.Subscriber("/odom", Odometry, self.state_Callback)
        rospy.Subscriber("/obstacles", Vector3Array, self.obstacle_Callback)

        #Publishers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", geo.Twist, queue_size=10)
        self.vector_pub = rospy.Publisher("visualization_marker", MarkerArray , queue_size=10)
        self.debug_pub = rospy.Publisher("debug", Float32 , queue_size=10)

        #tf listener
        self.tf_listener = tf.TransformListener()

        # Parameters 
        self.goal_tol = 0.25                        #m
        self.goal_tol_final = 0.15                                 
        self.rate = rospy.Rate(rate) # Hz  (1/Hz = secs)
        
        
        # Initialize internal variables
        #Trayectory planner
        self.path = Path()
        self.goal = geo.PoseStamped()                   #Actual goal 
        self.goal_received = False
        self.goal_distance = None
        self.final_goal = False                         
        self.num_points = 0.0
        self.num_act_point = 0                          #keep track of actual point within path

        #State of robot
        self.pose_odom = geo.PoseStamped()
        self.pose_base = geo.PoseStamped()
        self.position = geo.PointStamped()
        self.orientation_qt = None
        self.orientation_angle = 0.0

        #Controlador - virtual force field
        self.goal_transformed = geo.PoseStamped()
        self.Ft = [0,0,0]                           #Force vector towards goal
        self.Fr = [0,0,0]                           #Force vector repeling obstacles
        self.result_position = [0,0,0]              #Resulting vector
        self.target_angle = 0.0         
        self.Min_dist_t = 0.5              
        self.Kt = 0.75     #0.75                       #Constant value for Target force 
        self.Ft_mag = 0.0
        
        #Obstacles
        self.obstacles = Vector3Array()
        self.num_obs = 0
        self.Kr = 0.1
        self.max_Fr = 4.0

        #Vector to linear and angular velocity
        self.vel_ang_max = 3.0
        self.vel_lin_max = 3.0
        self.Kv = 1.0
        self.Kw = 4.0            #Proportional constant for angular velocity [s^-1].      ~=vel_ang_max / max ang(=pi rad)  (= Force saturate signal)
        self.Tm = 1/rate
        self.Kd = 0.75
        self.result_angle_1 = 0.0


        rospy.loginfo("Turtlebot controller started")

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("TurtleBot controller is shutting down...")

        self.tf_listener = None  # Stops tf listener

        # A default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel_pub.publish(geo.Twist())
        rospy.loginfo("Commanding lv: 0.0, av: 0.0")
        
        # Sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

    def publish(self, lin_vel, ang_vel):
        #Publish given velocities to /cmd_vel topic to send it to the robot
        move_cmd = geo.Twist()

        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        
        rospy.loginfo("Commanding lv: %.4f, av: %.4f", lin_vel, ang_vel)

        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        # Check if we have received a goal
        if(self.goal_received == False):
            rospy.loginfo("Goal not received. Waiting...")
            return

        # Check if the final goal has been reached
        if(self.goalReached()==True):
            if self.final_goal == True:
                rospy.loginfo("GOAL REACHED!!! Stopping!")
                self.publish(0.0, 0.0)
                self.goal_received = False

                #Inicialize variables related to last target
                self.result_position = [0,0,0]
                self.goal_transformed = geo.PoseStamped()
                self.path = Path()
                self.final_goal = False
                self.num_points = 0
                self.num_act_point = 0 
                return
            else:                           #Reach an intermediate point
                rospy.loginfo("Arrived to intermediate point, giving next goal")
                self.num_act_point = self.num_act_point + 1

                if self.num_act_point == self.num_points - 1:                        
                    self.final_goal = True

                self.goal_Callback(self.path.poses[self.num_act_point])
                      

        [v_lin,v_ang] = self.virtual_Force_Field()

        self.publish(v_lin, v_ang)
        return

    #Functions for Goal management --------------------------------

    def goal_Callback(self,goal_msg):
        rospy.loginfo("Goal received! x: %.2f, y:%.2f", goal_msg.pose.position.x, goal_msg.pose.position.y)
        self.goal = goal_msg  
        self.goal_received = True

        if self.path == Path():                     # Set goal manually not compatible unless no path received
            self.final_goal = True
    
    def goalReached(self):
        # Return True if the actual goal was reached, False otherwise
        if self.goal_received:
            pose_transformed = geo.PoseStamped()

            # Update the goal timestamp to avoid issues with TF transform
            self.goal.header.stamp = rospy.Time()

            try:
                pose_transformed = self.tf_listener.transformPose('base_footprint', self.goal)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Problem TF")
                return False                                #if not able to check, return false

            self.goal_distance = math.sqrt(pose_transformed.pose.position.x ** 2 + pose_transformed.pose.position.y ** 2)

            if self.final_goal == True: 
                if(self.goal_distance < self.goal_tol_final):
                    return True
            else:
                if(self.goal_distance < self.goal_tol):
                    return True
        return False            

    def path_Callback(self, path_msg):
        #Dealing with path message received:
        if path_msg.poses == self.path.poses:                       #do nothing if path recieved is the same 
            return
        rospy.loginfo("Path received!")
        self.goal_received = True
        self.num_points = len(path_msg.poses)
        self.path = path_msg

        if self.num_points  == 1:
            self.final_goal = True
        else:
            self.final_goal = False  

        self.num_act_point = 0

        self.goal_Callback(self.path.poses[0])

    #Other callback functions for subcribed topics --------------------------------        
    def state_Callback(self,odom_msg):
        if self.tf_listener is None:
            rospy.logwarn("Intento de usar tf_listener después del cierre.")
            return None        
        
        self.pose_odom.header.frame_id = "odom" 
        self.pose_odom.pose = odom_msg.pose.pose            #Pose - position and quaternion orientation
        
        try:            
            self.pose_odom.header.stamp = rospy.Time(0)     #Try with last transform avaible
            self.pose_base = self.tf_listener.transformPose('base_footprint', self.pose_odom)

        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        except Exception as e: 
            #err = traceback.format_exc()
            rospy.logerr(f"Problem TF : {e}")
            
        self.position = self.pose_odom.pose.position
        self.orientation_qt = self.pose_odom.pose.orientation

        #Convert orientation into a list 
        quaternion = [self.orientation_qt.x , self.orientation_qt.y, self.orientation_qt.z, self.orientation_qt.w]
        try:                                
            #Transform form quaternion orientation to euler angles
            orientation_angle_tuple = tf.transformations.euler_from_quaternion(quaternion)
            self.orientation_angle = orientation_angle_tuple[2] * 180/math.pi   # euler angle yaw == z - degrees

        except Exception as e:
            err = traceback.format_exc()
            rospy.logwarn(f"Problem TF in State Callback qt to euler transformation: {err}")


    def obstacle_Callback(self,obs_msg):
        ''' Recibe obstacles' positions refered to 'base_footprint' frame as a array of 'geo.Vector3' messages'''
        self.obstacles = obs_msg
        self.num_obs = int(len(self.obstacles.vectors))

        rospy.loginfo(f"Obstacles received : {len(self.obstacles.vectors)}")

    # VFF implementation --------------------------------
    def virtual_Force_Field(self):
        if self.tf_listener is None:
            rospy.logwarn("Intento de usar tf_listener después del cierre.")
            return None
        
        #---Target vector-------------------
        try:
            self.goal_transformed.header.stamp = rospy.Time(0)
            self.goal_transformed = self.tf_listener.transformPose('base_footprint', self.goal)         #Tranform each time bc robot frame moves
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF - goal not converted")
        
        target = self.goal_transformed.pose.position     #Vector3

        #rospy.loginfo(f", target x = {target.x},target y = {target.y},target z = {target.z}")
        
        # Goal orientation
        #target_qt = [self.goal_transformed.pose.orientation.x, self.goal_transformed.pose.orientation.y, self.goal_transformed.pose.orientation.z, self.goal_transformed.pose.orientation.w]
        #try:                                
            #Transform form quaternion orientation to euler angles
        #    target_angle_tuple = tf.transformations.euler_from_quaternion(target_qt, axes='sxyz')
        #    self.target_orientation = target_angle_tuple[2]   # euler angle yaw == z rad
        #    rospy.loginfo(f"Target angle = {self.target_orientation * 180/math.pi}")
        #except Exception as e:'
        #    err = traceback.format_exc()
        #    rospy.logwarn(f"Problem TF in target qt to euler transformation: {err}")
        #disctance to target
        dist_t = math.sqrt(target.x ** 2 + target.y ** 2 + target.z **2)

        # Virtual Force Vector towards target 
        self.Ft[0] = self.Kt * (target.x/dist_t)            #Same direction with constan magnitud
        self.Ft[1] = self.Kt * (target.y/dist_t)  
        self.Ft[2] = self.Kt * (target.z/dist_t)

        #Decrease force close to goal to allow full stop in final goal
        if self.final_goal == True and dist_t < self.Min_dist_t:                
            self.Ft[0] = self.Kt * dist_t * (target.x/dist_t)            #Same direction with Force magnitud
            self.Ft[1] = self.Kt * dist_t * (target.y/dist_t)  
            self.Ft[2] = self.Kt * dist_t * (target.z/dist_t)



        #---Obstacles vector------------------

        obs_matriz = np.zeros((self.num_obs, 3))      # Matrix with the normalized vectors
        obs_dist = np.zeros(self.num_obs)             # Distances to obstacles
        F = np.zeros((self.num_obs, 3))               # Repulsive force for each obstacle

        # Save normalized vectors:
        for i in range(self.num_obs):
            #get position vector of obstacle    (modified into 2d space)
            v = self.obstacles.vectors[i]  
            # Calculate distance
            obs_dist[i] = math.sqrt(v.x**2 + v.y**2 ) 
            #Normalize vector
            if obs_dist[i] > 0:  # Avoid division by zero
                obs_matriz[i] = np.array([v.x, v.y, 0])/obs_dist[i]

                F[i] = - (self.Kr / obs_dist[i]) * obs_matriz[i]

                if np.linalg.norm(F[i]) > self.max_Fr :                 # Saturate max force
                    F[i] = self.max_Fr * obs_matriz[i]

        #Total repulsive force is the sum of all obstacles forces
        self.Fr  = np.sum(F, axis=0)

        #---Result vector---------------------
        self.result_position[0] = self.Ft[0]  +  self.Fr[0] 
        self.result_position[1] = self.Ft[1]  +  self.Fr[1]  
        self.result_position[2] = self.Ft[2]  +  self.Fr[2] 



        #Visualize vector in rviz:
        #self.marker_publish( self.result_position, [target.x, target.y, target.z], self.Ft , F )
        self.marker_publish( self.result_position, [0,0,0], self.Ft , F )

        [v_lin,v_ang] = self.vector2vel_cmd(self.result_position)

        return v_lin,v_ang


    def vector2vel_cmd(self, vector):
        # Transform direction vector into linear velocity ang angular velocity
        # Proportional control for linear velocity
        v_lin = self.Kv * vector[0]     #Tangencial velocity = X component of result vector 
                                  
        # Angle between resulting vector and robot frame (to calculate v ang.) 
        vector_angle = math.atan2(vector[1] , vector[0])        # [-1,1]

        #rospy.loginfo(f"Result angle = {vector_angle * 180/math.pi}")   

        #Avoid sudden changes of sign around +-180º by keeping last direction  -  Implementation not finished
        #if abs(vector_angle) > 1 - 0.1 :
        #    if abs(vector_angle - self.result_angle_1) > 1:
        #        vector_angle = self.result_angle_1
        #        rospy.loginfo("Correcting +-180º singularity")

        self.debug_pub.publish(vector_angle)

        #PD for angular velocity control
        v_ang = self.Kw * (vector_angle/math.pi + self.Kd*(vector_angle - self.result_angle_1)/(self.Tm*math.pi) ) #rad/s

        #Saturation
        if abs(v_ang) > self.vel_ang_max:
            if v_ang > 0:
                v_ang = self.vel_ang_max
            else:
                v_ang = - self.vel_ang_max

        if v_lin < 0:               #Forbiden going backwards 
            v_lin = 0.0
        if v_lin > self.vel_lin_max:             #Max velocity 
            v_lin = self.vel_lin_max
        if v_lin < 0.05 and abs(v_ang) >= 0.1:            #If targer only a bit in front, use only vel ang to avoid doing spirals
            v_lin = 0.0

        #Variable actualization
        self.result_angle_1 = vector_angle 

        return  [v_lin,v_ang]

    #Publish vectors to visualize 
    def marker_publish(self, Vector1, Vector2 , Vector3, Matriz):
        marker_array = MarkerArray()

        wide = 3 + len(Matriz)
        vectores = np.zeros((3, wide)) 

        vectores[:3, 0] = Vector1                # Blue vector = Result vector
        vectores[:3, 1] = Vector2                # Purple vector = Real target
        vectores[:3, 2] = Vector3                # Green Vector = Target Force

        #Tranform matriz into last colums of vectores
        m = np.array(Matriz).T
        for j in range(0,wide-4):
            vectores[:,j+3] = m[:,j]             # Red Vectors = obstacles

        for i in range(vectores.shape[1]):      #For each column:
            marker = Marker()
            marker.header.frame_id = "base_footprint" 
            marker.header.stamp = rospy.Time.now()
            marker.ns = "vectores"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Inicial and final points      (All vectors start in robot position)
            marker.points = [geo.Point(0, 0, 0), geo.Point(vectores[0, i], vectores[1, i], vectores[2, i])] 

            # Color and size of vector
            marker.scale.x = 0.05  
            marker.scale.y = 0.1  
            marker.scale.z = 0.1  
            if i == 0:
                marker.color.b = 1.0    #Blue
            if i == 1:
                marker.color.b = 0.5    #Purple
                marker.color.r = 0.5
            if i == 2:
                marker.color.g = 1.0    #Green
            else:
                marker.color.r = 1.0    #red

            marker.color.a = 1.0  # Opacity

            marker_array.markers.append(marker)     #Add vector to Vector Array
        
        self.vector_pub.publish(marker_array)

if __name__ == '__main__':
    ''' Create controller instance and run periodically'''
    rate = 20 # Frecuency (Hz) for commanding the robot
    controller = TurtlebotController(rate)
    r = rospy.Rate(rate)

    while not (rospy.is_shutdown()):
        try: 
            controller.run()
        except Exception as e:
            err = traceback.format_exc()
            rospy.logerr(f"Error inesperado: {err}")

        r.sleep()
