#!/usr/bin/env python3

import math
import rospy
import tf
import geometry_msgs.msg as geo 
import sensor_msgs.msg as sens 
from autonomous_navigation.msg import Vector3Array

NEW_OBS_DIST = 0.45

class Detector():
    ''' This class provides simple obstacle avoidance functionalities to a ROS robot '''

    def __init__(self, obstacle_threshold=0.65, regional_angle=30):
        self.obs = Vector3Array()

        # Este diccionario mantiene un registro de las medidas de distancia para cada región y el ángulo de esa distancia
        self.Regions_Report = {
                            "front_C": [], "front_L": [], "left_R" : [],
                            "left_C" : [], "left_L" : [], "back_R" : [],
                            "back_C" : [], "back_L" : [], "right_R": [],
                            "right_C": [], "right_L": [], "front_R": [],
                        }

        self.OBSTACLE_DIST  = obstacle_threshold
        self.REGIONAL_ANGLE = regional_angle

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber("/scan", sens.LaserScan, self.indentify_regions)

        self.obs_pub = rospy.Publisher("/obstacles", Vector3Array, queue_size = 10)

        rospy.loginfo("Iniciando detector de obstaculos")


    def indentify_regions(self, scan):
        REGIONS = [
                        "front_C", "front_L", "left_R" ,
                        "left_C" , "left_L" , "back_R" ,
                        "back_C" , "back_L" , "right_R",
                        "right_C", "right_L", "front_R",
                    ]

        # Si es la primera vez que se lee el Lidar
        if scan.ranges:
            # Primeros y últimos 15 puntos de la región frontal central
            intermediary = scan.ranges[:int(self.REGIONAL_ANGLE/2)] + scan.ranges[-int(self.REGIONAL_ANGLE/2):]
            filtered = [x for x in intermediary if x <= self.OBSTACLE_DIST and x != 'inf']
            if filtered:
                min_value = min(filtered)
                angle = next(i for i, v in enumerate(scan.ranges) if v == min_value)
            else:
                min_value = 0
                angle = 0
            x = min_value * math.cos(math.radians(angle))
            y = min_value * math.sin(math.radians(angle))
            self.Regions_Report["front_C"] = [x, y, 0, min_value]

            # Para todas las regiones menos la primera
            for i, region in enumerate(REGIONS[1:], start = 1):
                if i <= 2 or i >= 10:
                    # Encontrar el valor mínimo en cada región de interés
                    filtered = [x for x in scan.ranges[self.REGIONAL_ANGLE*i:self.REGIONAL_ANGLE*(i+1)] if x <= self.OBSTACLE_DIST and x != 'inf']
                    if filtered:
                        min_value = min(filtered)
                        angle = next(i for i, v in enumerate(scan.ranges) if v == min_value)
                    else:
                        min_value = 0
                        angle = 0
                    x = min_value * math.cos(math.radians(angle))
                    y = min_value * math.sin(math.radians(angle))
                    self.Regions_Report[region] = [x, y, 0, min_value]

            for key, value in self.Regions_Report.items():
                if value:
                    # Crear el punto en el frame del LIDAR ("base_scan")
                    punto = geo.PoseStamped()
                    punto.header.frame_id = "base_scan"
                    punto.header.stamp = rospy.Time(0)
                    punto.pose.position.x = self.Regions_Report[key][0]
                    punto.pose.position.y = self.Regions_Report[key][1]
                    punto.pose.position.z = 0
                    punto.pose.orientation.w = 1.0  # Sin rotación

                    try:
                        # Esperar la transformación antes de aplicarla
                        self.tf_listener.waitForTransform("base_scan", "base_footprint", rospy.Time(0), rospy.Duration(1.0))

                        # Transformar el punto al frame "base_footprint"
                        transformed_punto = self.tf_listener.transformPose("base_footprint", punto)

                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logwarn("Error transformando coordenadas de obstáculo Regions_Report")

                    try:
                        # Esperar la transformación antes de aplicarla
                        self.tf_listener.waitForTransform("base_footprint", "odom", rospy.Time(0), rospy.Duration(1.0))

                        # Transformar el punto al frame "odom"
                        transformed_punto2 = self.tf_listener.transformPose("odom", transformed_punto)

                        # Obtener coordenadas transformadas
                        x_tf = transformed_punto2.pose.position.x
                        y_tf = transformed_punto2.pose.position.y
                        z_tf = transformed_punto2.pose.position.z

                        self.Regions_Report[key] = [x_tf, y_tf, z_tf, self.Regions_Report[key][3]]

                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logwarn("Error transformando coordenadas de obstáculo Regions_Report")

            self.tracker()


    def tracker(self):
        for key, value in self.Regions_Report.items():
            if value:
                if value[3] > 0:
                    x = value[0]
                    y = value[1]
                    z = value[2]

                    # Crear el punto en el frame del Mundo ("odom")
                    punto = geo.PoseStamped()
                    punto.header.frame_id = "odom"
                    punto.header.stamp = rospy.Time(0)
                    punto.pose.position.x = x
                    punto.pose.position.y = y
                    punto.pose.position.z = z
                    punto.pose.orientation.w = 1.0  # Sin rotación

                    try:
                        # Esperar la transformación antes de aplicarla
                        self.tf_listener.waitForTransform("odom", "base_footprint", rospy.Time(0), rospy.Duration(1.0))

                        # Transformar el punto al frame "odom"
                        transformed_punto = self.tf_listener.transformPose("base_footprint", punto)

                        # Obtener coordenadas transformadas
                        x_tf = transformed_punto.pose.position.x
                        y_tf = transformed_punto.pose.position.y
                        z_tf = transformed_punto.pose.position.z

                        self.obs.vectors.append(geo.Vector3(x_tf, y_tf, z_tf))

                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logwarn("Error transformando coordenadas de obstáculo Final")

        # Publicar los obstáculos detectados
        self.obs_pub.publish(self.obs)
        self.obs.vectors = []
        
        
    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            rate.sleep()
        

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('obstacle_detection')

    # Crear objeto
    nodo = Detector()

    # Correr el nodo
    nodo.run()