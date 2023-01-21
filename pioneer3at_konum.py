#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Twist
from math import atan2
from tf import transformations
import math
from sensor_msgs.msg import NavSatFix, Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int16, String
from math import sqrt
import time

p3at_gps_konum = NavSatFix()
hedef = NavSatFix()
xy_point = Point()

baslangic_point = Point()
baslangic_point.x = 3.0
baslangic_point.y = 3.0

cart_x = 0.0
cart_y = 0.0

komut = String()
hedef_uzaklik = 0
donus_aci = 0
p3at_hiz = 0.0

def Komut_Callback(data):
    global komut
    komut = data.data



def pionerr3at_gps_callback(msg) :
    global p3at_gps_konum
    p3at_gps_konum.latitude = msg.latitude
    p3at_gps_konum.longitude = msg.longitude
    p3at_gps_konum.altitude = msg.altitude
    #print(p3at_gps_konum)

def pionerr3at_konum_callback(msg) :
    global xy_point, donus_aci
    xy_point.x = msg.pose.pose.position.x
    xy_point.y = msg.pose.pose.position.y

    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w )
        

    euler = transformations.euler_from_quaternion(quaternion)
    donus_aci   = euler[2]

def Pioneer_Hiz_callback(hiz_komut):
    global p3at_hiz
    if(hiz_komut.data != ""):
        p3at_hiz = float(hiz_komut.data)
    else:
        p3at_hiz = 0.0    


def ENLEM_BOYLAM_to_XY(latitude, longitude):
     #referans: https://en.wikipedia.org/wiki/Earth_radius, WGS84'e gore alinmistir. 
    global cart_x, cart_y
    origin_latitude  = 49.8602455603    #49.8602499617
    origin_longitude = 8.68707686586    #8.6870957027
    
    WORLD_POLAR_M = 6356752.3142   # DUNYA_KUTUP_YARICAP
    WORLD_EQUATORIAL_M = 6378137.0 #DUNYA_EKVATORAL_YARICAP

    eccentricity = math.acos(WORLD_POLAR_M/WORLD_EQUATORIAL_M) # dis_merkezlilik
    n_prime = 1/(math.sqrt(1 - math.pow(math.sin(math.radians(float(latitude))),2.0)*math.pow(math.sin(eccentricity), 2.0)))
    m = WORLD_EQUATORIAL_M * math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)
    n = WORLD_EQUATORIAL_M * n_prime

    difflongitude = float(longitude) - float(origin_longitude)
    difflatitude  = float(latitude)  - float(origin_latitude)


    surfdistLongitude = math.pi /180.0 * math.cos(math.radians(float(latitude))) * n
    surfdistLatitude  = math.pi/180.00 * m
   
    cart_y = difflongitude * surfdistLongitude
    cart_x = difflatitude  * surfdistLatitude
    cart_y *= -1
    return cart_x,cart_y



def Hedef_GPS_Callback(msg):
    global hedef
    hedef.latitude = msg.latitude
    hedef.longitude = msg.longitude

#################################################################################################
def hedef_xy():
    a = [0, 0]
    a = ENLEM_BOYLAM_to_XY(hedef.latitude, hedef.longitude)
    #print(a)
    return a
# KOORDINATLARI VERILEN HEDEF NOKTANIN X VE Y DEGERINI BULAN FONKSIYON
#################################################################################################


#################################################################################################
def hedef_acim():
    global hedef_uzaklik
    hedef_x, hedef_y = hedef_xy()
    anlik_x, anlik_y = ENLEM_BOYLAM_to_XY(p3at_gps_konum.latitude, p3at_gps_konum.longitude)
    fark_x           = hedef_x - anlik_x
    fark_y           = hedef_y - anlik_y
    hedf_aci         = math.atan2(fark_y,fark_x)

    hedef_uzaklik = sqrt(pow((hedef_x - anlik_x), 2) + pow((hedef_y - anlik_y), 2))
    return hedf_aci
# PIOONER IN VERILEN HEDEF ICIN YONELDIGI ACIYI BULAN FONKSIYON
###################################################################################################

def hedefe_git() :
    x, y = ENLEM_BOYLAM_to_XY(p3at_gps_konum.latitude, p3at_gps_konum.longitude)                           # Pioonerin anlik x ve y konumu
    msg  = Twist()
    h_x, h_y = hedef_xy() 
    hedef_aci = hedef_acim()

    if hedef_aci - donus_aci > 0.1 :
        msg.angular.z = 0.4
    elif hedef_aci - donus_aci < -0.1 :
        msg.angular.z = - 0.4
    else :
        msg.angular.z = 0.0
        msg.linear.x  = p3at_hiz    #0.5     

    if(hedef_uzaklik < 2.0) :
        msg.linear.x = 0.3
    else :
        pass


    if(hedef_uzaklik < 0.1) :
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        print("Hedefe vardi") 
            
    else :
        pass
    
    return msg


if __name__ == '__main__':
    rospy.init_node("pioneer3at_konum", anonymous = True)
    rospy.Subscriber("/gps/fix", NavSatFix , pionerr3at_gps_callback )
    pub1 = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size = 1)
    rospy.Subscriber("/robot2/odom", Odometry , pionerr3at_konum_callback )
    
    
    rospy.Subscriber('/pioneer_hiz_komut', String, Pioneer_Hiz_callback)
    

    pub = rospy.Publisher("/pioneer_location", NavSatFix, queue_size = 1)
    rospy.Subscriber('/komut_p3at', String, Komut_Callback)
    rospy.Subscriber('/komut_gps_p3at', NavSatFix, Hedef_GPS_Callback)

    pub_p3_yaw  = rospy.Publisher('/pioneer_yaw', String, queue_size = 1)

    msg = Twist()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown() :
    
            
        ENLEM_BOYLAM_to_XY(p3at_gps_konum.latitude, p3at_gps_konum.longitude)
        
        print("P3AT KONUMU PUBLISH EDILIYOR")
        
        if(komut == "Hedef GPS'e Git"):
            msg = hedefe_git()
        
        else:
            pass
        
        
        str_yaw = str((donus_aci * 180)/ math.pi)

        pub.publish(p3at_gps_konum)
        pub1.publish(msg)

        pub_p3_yaw.publish(str_yaw)
       
        rate.sleep()
    #rospy.spin()