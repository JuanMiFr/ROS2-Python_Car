#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import ChannelFloat32

from math import atan2,pi,cos,sin
import numpy as np 
from scipy.integrate import odeint

#Etat : (x, y, theta, wd, wg, thetad, thetag)

class mod_act(Node):
    def __init__(self):
        super().__init__("mod_act")
        self.get_logger().info("demarrage mod_act")
        
        self.publisher = self.create_publisher(ChannelFloat32,"Etat_estime",1)
        self.subscriber_cmd = self.create_subscription(Twist,"Command",self.callback_cmd,1)    
        
        self.declare_parameter('x0',100.0)
        self.declare_parameter('y0',100.0)
        self.declare_parameter('theta0',0.0)
        self.declare_parameter('Wmax',10000.0)
        (x0,y0,theta0,self.Wmax)=self.get_parameters(['x0','y0','theta0','Wmax'])
        self.R=3.0	#radio
        self.L=10.0 #eje


        self.tau=0.1 #tiempo de respuesta. Ne pas mettre trop grand sinon divergence du robot. Si trop pret de 0  on a comportement du controleur du prof
        self.Etat = ChannelFloat32()
        self.Etat.values=[x0.value,y0.value,theta0.value,0.0,0.0,0.0,0.0]
        #important publication du nouvel etat toutes les 0.05s.Ne pas mettre trop grandes les periodes, ni trop petites car divergence
        self.cmd = Twist()
        self.timer=self.create_timer(0.05,self.publish_Etat)
                    
    #Debut du publisheur
    def publish_Etat(self):
        #declaration des variables
        msg = ChannelFloat32()
        msg.values = [0.0 , 0.0 , 0.0]

        #partie integration numerique, reprise du code python donné en exemple
        dt=0.05
        ts=[0,dt]
        #nouvel etat a integrer(issu du calcul du modele du robot)
        nouvel_etat = odeint(self.robot,self.Etat.values,ts,args=(self.cmd.linear.x,self.cmd.angular.z))
        for i in range (7):
            self.Etat.values[i]=nouvel_etat[1,i]
        for i in range (3):
            msg.values[i]=self.Etat.values[i]


        self.publisher.publish(msg)





    #model du robot pour estuimation realiste de l'etat
    def robot(self,Etat,t,u,w):
        #definitions du cours de omegaconst droite et gauche
        wdc = u*(1/self.R) + w*(self.L/self.R)
        wgc = u*(1/self.R) - w*(self.L/self.R)

        #estimation de l'etat suivant pour le vecteur de commande, ainsi que le vecteur d'etat
        dx_dt=(self.R/2)*(self.Etat.values[3]+self.Etat.values[4])*cos(self.Etat.values[2])
        dy_dt=(self.R/2)*(self.Etat.values[3]+self.Etat.values[4])*sin(self.Etat.values[2])
        dtheta_dt=(self.R/(2*self.L))*(self.Etat.values[3]-self.Etat.values[4])
        #equations temporelles des moteurs
        dwd_dt=(1/self.tau)*(wdc-self.Etat.values[3])
        dwg_dt=(1/self.tau)*(wgc-self.Etat.values[4])
        #avec ces 5 equations on a le nouveau vecteur d'etat pour simuler le systeme

        #on remet le nouveau etat suivant dans etat
        dthetad_dt= self.Etat.values[3]
        dthetag_dt= self.Etat.values[4]
        
        return np.array([dx_dt, dy_dt, dtheta_dt, dwd_dt, dwg_dt, dthetad_dt, dthetag_dt])



    #dans notre cas du robot a deux roues nous actualisons uniquement la vitesse angulaire z et le x 
    def callback_cmd(self,msg):        
        self.cmd.linear.x = msg.linear.x
        self.cmd.angular.z = msg.angular.z




def main(args=None):
    rclpy.init(args=args)
    node=mod_act()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()