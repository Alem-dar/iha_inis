#!/usr/bin/env python

#   KUTUPHANELER
#################################################################################################

import rospy
import sensor_msgs.msg
import time
import math
import numpy as np

from sensor_msgs.msg                import LaserScan
from geometry_msgs.msg              import PoseStamped, Point, Twist
from math                           import atan2
from tf                             import transformations
from sensor_msgs.msg                import NavSatFix, Imu, Image
from std_msgs.msg                   import Float64, Int16, String

from math                           import sqrt
from visualization_msgs.msg         import Marker
from ar_track_alvar_msgs.msg        import AlvarMarkers

from nav_msgs.msg                   import Odometry

#################################################################################################
#   GLOBAL DEGISKENLER
pozisyon        = NavSatFix()
hedef           = NavSatFix()
piooner_hedef   = NavSatFix()

hedef_uzaklik     = 0.0
marker_donus_aci  = 0.0
donus_aci         = 0.0
irtifa            = 4.0
takip_mesafe      = 0.0
p3at_yaw          = 0.0
hector_hiz        = 0.0
ar_marker         = []

lat0  = 49.8602456847                             # 0'a 0 noktasinin latitude longitude degerleri
lon0  = 8.68707640992      


hedef_marker  = Point()
pozisyonum    = Point()
p3at_hiz      = Twist()
komut         = String()
hector_bilgi  = String()
ar_bilgi      = String()

ar_ok          = False
break_ok       = False
hedef_marker_x = False
arazi_bilgi = False


rx= 0
ry= 0
count_xy = -1
noktalar_x = []
noktalar_y = []
i_rx = 0
i_ry = 0
tarama_lat = 100.0
tarama_lon = 100.0
h_x = 0.0
h_y = 0.0

deneme_xy = Point() 
#################################################################################################
def Hector_Irtifa_Callback(hec_irtifa):
    global irtifa
    if(hec_irtifa.data != ""):
        irtifa = float(hec_irtifa.data)
    else:
        pass
# HECTOR'UN YUKSEKLIGI ICIN ARAYUZDEN VERILEN IRTIFAYI ALAN FONKSIYON
#################################################################################################

def komut_callback(msg):
    global komut
    komut = msg.data

# ARAYUZDEN VERILEN KOMUTU ALAN FONKSIYON
#################################################################################################

def Takip_Mesafe_Callback(mesafe):
    global takip_mesafe
    if(mesafe.data == ""):
        takip_mesafe = 0.0
    else:

        takip_mesafe = float(mesafe.data)
        #print(mesafe)

# TAKIP MESAFESI ICIN VERILEN DEGERI ALAN FONKSIYON
#################################################################################################

def pozisyon_callback(msg):
    global pozisyonum
    pozisyonum = msg.pose.position
# Z EKSENI YANI YUKSEKLIK BULAN FONKSIYON
#################################################################################################

def arayuz_gps_callback(msg) :
    global hedef
    hedef.latitude  =  msg.latitude
    hedef.longitude = msg.longitude
  
# ARAYUZDEN HEDEFE GIT KOMUTU ILE HECTOR'A VERILEN NOKTANIN KOORDINATLARI
#################################################################################################

def p3at_gps_callback(msg) :
    global piooner_hedef
    piooner_hedef.latitude  =  msg.latitude
    piooner_hedef.longitude =  msg.longitude

# PIOONER LATITUDE LONGITUDE ANLIK DEGERLERINI VEREN FONKSIYON
#################################################################################################
def Hector_Hiz_Callback(hec_hiz):
    global hector_hiz 
    if(hec_hiz.data != ""):
        hector_hiz = float(hec_hiz.data)
    else:
        if(p3at_hiz.linear.x == 0):
            hector_hiz = 0.5
        
        else :
            hector_hiz = p3at_hiz.linear.x + 0.3
#################################################################################################
def Hector_Tarama_Callback(msg):
    global tarama_lat
    global tarama_lon
    tarama_lat = msg.latitude
    tarama_lon = msg.longitude


def Pioneer_Yaw_Callback(msg):
    global p3at_yaw

    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w )
        

    euler = transformations.euler_from_quaternion(quaternion)
    p3at_yaw   = euler[2]
    

# SENSORDEN PIOONER'IN YAW DEGERINI HESAPLAYAN FONKSIYON
#################################################################################################




#################################################################################################
def gps_callback(msg) : 
    global pozisyon
    pozisyon.latitude  = msg.latitude
    pozisyon.longitude = msg.longitude
    pozisyon.altitude  = msg.altitude
    
# HECTOR LATITUDE LONGITUDE ANLIK DEGERLERINI VEREN FONKSIYON
#################################################################################################


    
#################################################################################################
def gps_callback1(msg) :
    global donus_aci

    quaternion = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ]

    euler     = transformations.euler_from_quaternion(quaternion)
    donus_aci = euler[2]
    
# SENSORDEN HECTOR'UN YAW DEGERINI HESAPLAYAN FONKSIYON
#################################################################################################

def hedef_xy(h_latitude, h_longitude):
    a = [0, 0]
    a = get_local_coord(h_latitude, h_longitude)
    
    return a
# KOORDINATLARI VERILEN HEDEF NOKTANIN X VE Y DEGERINI BULAN FONKSIYON
#################################################################################################


#################################################################################################
def get_local_coord(lat, lon):
    WORLD_POLAR_M       = 6356752.3142
    WORLD_EQUATORIAL_M  = 6378137.0
    origin_lat          = 49.8602456847  
    origin_lon          = 8.68707640992   
    eccentricity        = math.acos(WORLD_POLAR_M/WORLD_EQUATORIAL_M)
    
    n_prime = 1/(math.sqrt(1 - math.pow(math.sin(math.radians(float(lat))),2.0)*math.pow(math.sin(eccentricity), 2.0)))
    
    m       = WORLD_EQUATORIAL_M * math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)
    
    n       = WORLD_EQUATORIAL_M * n_prime
    
    difflon = float(lon) - float(origin_lon)
    difflat = float(lat) - float(origin_lat)
    

    surfdistLon = math.pi /180.0 * math.cos(math.radians(float(lat))) * n
    surfdistLat = math.pi/180.00 * m
    
    y = -(difflon * surfdistLon)
    x =  difflat * surfdistLat
    return x,y 
# ANLIK OLARAK BILINEN LATITUDE LONGITUDE DEGERLERINI X VE Y YE DONUSTUREN FONKSIYON
#################################################################################################


#################################################################################################
def mdeglon(lon1):
    lon0rad = math.radians(lon1)
    return (111415.13 *math.cos(lon0rad) - 94.55 * math.cos(3.0 *lon0rad) - 0.12 * math.cos(5.0 * lon0rad))

def mdeglat(lat1):
    lat0rad = math.radians(lat1)
    return (111132.09 - 566.05 *math.cos(2.0*lat0rad) + 1.20 * math.cos(4.0 *lat0rad) - 0.002 * math.cos(6.0 * lat0rad))

def xy2lanlon(x1,y1,lat1,lon1):
    global lat0, lon0                                                                    # x = 0 y = 0 Latitude Longitude degerini gosteren global degiskenler
    y1          = y1 * (-1)
    longitude1  = y1 / mdeglon(lat1) + lon1
    latitude1   = x1/ mdeglat(lat1) + lat1
    return latitude1, longitude1
# X VE Y YI LATITUDE LONGITUDE DONUSTUREN FONKSIYONLAR
#################################################################################################



#################################################################################################
def hedef_acim(h_latitude, h_longitude):
    global hedef_uzaklik
    hedef_x, hedef_y = hedef_xy(h_latitude, h_longitude)
    anlik_x, anlik_y = get_local_coord(pozisyon.latitude, pozisyon.longitude)
    fark_x           = hedef_x - anlik_x
    fark_y           = hedef_y - anlik_y
    hedf_aci         = math.atan2(fark_y,fark_x)

    hedef_uzaklik = sqrt(pow((hedef_x - anlik_x), 2) + pow((hedef_y - anlik_y), 2))
    return hedf_aci
# QUADCOPTERIN VERILEN HEDEF ICIN YONELDIGI ACIYI BULAN FONKSIYON
###################################################################################################

def hareket_algila(msg):
    global p3at_hiz
    p3at_hiz = msg
    #print(p3at_hiz.linear.x)

def marker(msg) :
    global ar_marker, marker_donus_aci, hedef_marker
    ar_marker = msg.markers 
    if(msg.markers != []) :
        hedef_marker = msg.markers[0].pose.pose.position
       
        
        quaternion = [
            msg.markers[0].pose.pose.orientation.x,
            msg.markers[0].pose.pose.orientation.y,
            msg.markers[0].pose.pose.orientation.z,
            msg.markers[0].pose.pose.orientation.w
        ]
        
        euler            = transformations.euler_from_quaternion(quaternion)
        marker_donus_aci = euler[2]
       
        
    else :
        pass




def hedefe_git(h_latitude, h_longitude) :
    global hector_bilgi
    global arazi_bilgi
    global h_x, h_y
    x, y = get_local_coord(pozisyon.latitude, pozisyon.longitude)                           # Hectorun anlik x ve y konumu
    msg  = Twist()
    
    hassasiyet_konum = 0.2
    h_x, h_y = hedef_xy(h_latitude, h_longitude)                                                                   # Hedef konumun x ve y degeri bulundu ve yerel degiskenlere atandi
    
            
    hedef_aci = hedef_acim(h_latitude, h_longitude)
    
    if pozisyonum.z < irtifa : 
        msg.linear.z = 0.8
        hector_bilgi = "Hector {} m irtifaya yukseliyor.".format(irtifa)
        if pozisyonum.z > 1 :
            if hedef_aci - donus_aci > 0.1 :
                msg.angular.z = 0.4
            elif hedef_aci - donus_aci < -0.1 :
                msg.angular.z = - 0.4
            else :
                msg.angular.z = 0.0

                if(komut == "AR PLATFORMUNA INIS" or komut == "ARACI TAKIP ET") :
                    msg.linear.x  = p3at_hiz.linear.x + 0.5     #0.3 tu
                    hector_bilgi = "Hector hedefe ilerliyor."
                else :
                    msg.linear.x  = hector_hiz    #p3at_hiz.linear.x + 0.5     #0.3 tu
                    hector_bilgi = "Hector hedefe ilerliyor."
        else :
            hector_bilgi = "Hedefe {} m kaldi.".format(hedef_uzaklik)
            if(hedef_uzaklik <= 0.15) :
                msg.linear.x = p3at_hiz.linear.x
            else :
                pass

    elif pozisyonum.z > irtifa + 0.01 :
        msg.linear.z = 0
        msg.linear.x = p3at_hiz.linear.x + 0.3 
        hector_bilgi = "Hector {} m yukseklige ulasti.".format(irtifa)

        if pozisyonum.z < irtifa + 1.0 :
            if hedef_aci - donus_aci > 0.1 :
                msg.angular.z = 0.4
            elif hedef_aci - donus_aci < - 0.1 :
                msg.angular.z = - 0.4
            else :
                msg.angular.z = 0.0
                
                if(komut == "AR PLATFORMUNA INIS" or komut == "ARACI TAKIP ET") :
                    msg.linear.x  = p3at_hiz.linear.x + 0.5     #0.3 tu
                    hector_bilgi = "Hector hedefe ilerliyor."
                else :
                    msg.linear.x  = hector_hiz    #p3at_hiz.linear.x + 0.5     #0.3 tu
                    hector_bilgi = "Hector hedefe ilerliyor."
        else :
            hector_bilgi = "Hedefe {} m kaldi.".format(hedef_uzaklik)
    
        
    
    else :
        pass 


    
    if(p3at_hiz.linear.x != 0) :
        
        if (h_x - hassasiyet_konum < x and x < h_x + hassasiyet_konum) and (h_y - hassasiyet_konum < y and y < h_y + hassasiyet_konum) :
            msg.linear.x = p3at_hiz.linear.x 
            msg.linear.z = -0.6       
            hector_bilgi = "Hector inise geciyor."    
        else :
            pass

    else :
        if (h_x - hassasiyet_konum < x and x < h_x + hassasiyet_konum) and (h_y - hassasiyet_konum < y and y < h_y + hassasiyet_konum) :
            msg.linear.x = 0.0
            msg.linear.z = 0.0       
            print("Buraya girmeeee!!!")
        else :
            pass
    
   

    
    
    #print("Hedefe Git hiz: " , msg.linear.x)
    return msg






def marker_tespit() :
    global ar_ok, break_ok, ar_bilgi
    msg  = Twist()
 


    if(ar_ok != True):
        ar_bilgi = "AR TAG Algilandi."
        if(pozisyonum.z > 2):
            msg.linear.z = -0.5
        else:
            msg.linear.z = -0.2

        if(pozisyonum.z > 1.9 and pozisyonum.z < 2.1):
            if(hedef_marker.z > -0.14):
                ar_bilgi = "AR TAG ustune hareket ediliyor."
                msg.linear.x = 0.6
                msg.linear.z = 0.0

            elif(hedef_marker.z < -0.2):
                ar_bilgi = "AR TAG geride kaldi."
                msg.linear.z = 0.0
                msg.linear.x = -0.2
            else:
                ar_bilgi = "Hector AR TAG ustunde."
                ar_ok = True
               # msg.linear.x = 0.0
        else:
            pass
    
    else:
        msg.linear.x = 0.0

        if(pozisyonum.z > 1.0):
            ar_bilgi = "AR TAG ustune iniliyor."
            msg.linear.z = -0.6
            break_ok = False
        else:
            ar_bilgi = "AR TAG'e inis saglandi."
            break_ok = True    
    
    return msg










def hareket_takib() :
    hector_hiz1 = Twist()

    pozisyon_kontrol =(takip_mesafe * 2.12) / 0.8
    hector_hiz1.linear.x = p3at_hiz.linear.x

    if p3at_yaw - donus_aci > 0.2:
        hector_hiz1.angular.z = 0.6

    elif p3at_yaw - donus_aci > 0.1:
        hector_hiz1.angular.z = 0.3

    elif p3at_yaw - donus_aci < -0.2:
        hector_hiz1.angular.z = -0.6

    elif p3at_yaw - donus_aci < -0.1:
        hector_hiz1.angular.z = -0.3

    else:
        hector_hiz1.angular.z = 0.0

        if hedef_marker.y > 0.2:
            hector_hiz1.linear.y = 0.4
        elif hedef_marker.y > 0.1:
            hector_hiz1.linear.y = 0.2

        elif hedef_marker.y < -0.2:
            hector_hiz1.linear.y = -0.4

        elif hedef_marker.y < -0.1:
            hector_hiz1.linear.y = -0.2
        else:
            hector_hiz1.linear.y = 0.0

    if(takip_mesafe == 0.0):
        if(hedef_marker.z > -0.15):
            hector_hiz1.linear.x = p3at_hiz.linear.x + 0.3
        
        elif(hedef_marker.z > -0.2):
            hector_hiz1.linear.x = p3at_hiz.linear.x - 0.2
        
        else:
            hector_hiz1.linear.x = p3at_hiz.linear.x

    else:

        if(hedef_marker.z > pozisyon_kontrol + 0.2):
            hector_hiz1.linear.x = (p3at_hiz.linear.x + 0.3)

        elif(hedef_marker.z >= pozisyon_kontrol):
            hector_hiz1.linear.x = p3at_hiz.linear.x
        
        elif(hedef_marker.z > pozisyon_kontrol - 0.3):
            hector_hiz1.linear.x = 0.0
    
    return hector_hiz1



def hareketli_inis():
    global ar_ok,break_ok,hedef_marker_x
    speed = Twist()
  

    if p3at_yaw - donus_aci > 0.2:
        speed.angular.z = 0.6

    elif p3at_yaw - donus_aci > 0.1:
        speed.angular.z = 0.3



    elif p3at_yaw - donus_aci < -0.2:
        speed.angular.z = -0.6

    elif p3at_yaw - donus_aci < -0.1:
        speed.angular.z = -0.3


    else:
        speed.angular.z = 0.0

    if(ar_ok != True):         
        if hedef_marker.z > 0.0:
            speed.linear.x = p3at_hiz.linear.x + 0.5
        elif(hedef_marker.z > -0.15):
            speed.linear.x = p3at_hiz.linear.x + 0.4

            if(pozisyonum.z > 2.0):
                speed.linear.z = -0.4
            else:
                speed.linear.z = 0.0
        elif(hedef_marker.z < - 0.17):    
            speed.linear.x = p3at_hiz.linear.x + 0.1

        elif(hedef_marker.z < -0.2):
            speed.linear.x = 0.1
        
        else:
            speed.linear.x = p3at_hiz.linear.x + 0.1
            hedef_marker_x = True

        
        if hedef_marker.y > 0.2:
            speed.linear.y = 0.5
        elif hedef_marker.y > 0.1:
            speed.linear.y = 0.2

        elif hedef_marker.y < -0.2:
            speed.linear.y = -0.5

        elif hedef_marker.y < -0.1:
            speed.linear.y = -0.2
        else:
            speed.linear.y = 0.0
            #pos_marker_y = True

        
        if(hedef_marker_x == True):        
            ar_ok = True
        else:
            ar_ok = False

    else:

        speed.linear.x = p3at_hiz.linear.x 

        if(pozisyonum.z > 1.0):
            speed.linear.z = -0.5
            break_ok = False
        else:
            break_ok = True  
        
    return speed

    

def Noktalari_Bul(hedef_lat,hedef_lon):
    global rx,ry,count_xy  
    global noktalar_x
    global noktalar_y  
    global artis
    rx,ry = get_local_coord(pozisyon.latitude,pozisyon.longitude) 
    if rx >= 0:
        if(int(rx + 0.5) == int(rx)):
            rx = math.floor(rx)
        else:
            rx = math.ceil(rx)
    else:
        if(int(rx - 0.5) == int(rx)):
            rx = math.ceil(rx)
        else:
            rx = math.floor(rx)


    if ry > 0:

        if(int(ry + 0.5) == int(ry)):
            ry = math.floor(ry)
        else:
            ry = math.ceil(ry)
    else:
        if(int(rx - 0.5) == int(rx)):
            rx = math.ceil(rx)
        else:
            rx = math.floor(rx)
    
    

    if rx % 2 == 0:
        if rx >= 0:
            rx = rx + 0.0000001
        else:
            rx = rx - 0.0000001
    else:
        pass

    if ry % 2 == 0:
        if ry > 0:
            ry = ry + 0.0000001
        else:
            ry = ry - 0.0000001
    else:
        pass

    son_x, son_y = get_local_coord(tarama_lat,tarama_lon)
    if(int(son_x + 0.5) == int(son_x)):
        son_x = math.floor(son_x)
    else:
        son_x = math.ceil(son_x)
    
    if(int(son_y + 0.5) == int(son_y)):
        son_y = math.floor(son_y)
    else:
        son_y = math.ceil(son_y)


    if count_xy != 1:
        if(rx < son_x):
            artis = 1
        elif(rx > son_x):
            artis = -1
        else:
            artis = 0
        
        count_xy = 1
    else:
        pass
    
    if son_x % 2 == 0:
        if son_x > 0:
            son_x = son_x + 0.0000001
        else:
            son_x = son_x - 0.0000001
    else:
        pass

    if son_y % 2 == 0:
        if son_y > 0:
            son_y = son_y + 0.0000001
        else:
            son_y = son_y - 0.0000001
    else:
        pass



    while(int(rx) != int(son_x)):

        noktalar_x.append(rx)
        noktalar_x.append(rx)
        rx = rx + artis
        

    if(int(rx) == int(son_x)):

        noktalar_x.append(rx)
        noktalar_x.append(rx)


            
    noktalar_y.append(ry)
    uzunluk = len(noktalar_x)


    for i in range (int(uzunluk / 2)):
        if(i != (uzunluk / 2) - 1):
            if i % 2 == 0:
                noktalar_y.append(son_y)
                noktalar_y.append(son_y)
            else:
                noktalar_y.append(ry)
                noktalar_y.append(ry)
        else:
            noktalar_y.append(son_y)

    if rx < 0:
        noktalar_x.pop()
        noktalar_x.pop()
        noktalar_y.pop()
        noktalar_y.pop()
        noktalar_y.pop()
        noktalar_y.append(son_y)
    else:
        pass

    return noktalar_x,noktalar_y






if __name__ == '__main__':
    rospy.init_node("gps_koordinat", anonymous = True)
    rospy.Subscriber("/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/raw_imu", Imu, gps_callback1)
    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped , pozisyon_callback )
    rospy.Subscriber("/pioneer_location", NavSatFix, p3at_gps_callback)                                 # P3AT GPS bilgisi icin olusturdugum topic. Baska bir node uzerinden yolluyorum.
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers, marker)
    rospy.Subscriber("/robot2/cmd_vel", Twist, hareket_algila)
    rospy.Subscriber('/komut_gps_git', NavSatFix, arayuz_gps_callback)
    rospy.Subscriber('/komut', String, komut_callback)
    rospy.Subscriber('/hector_irtifa',String, Hector_Irtifa_Callback)
    rospy.Subscriber('/hector_artag_takip_mesafe', String, Takip_Mesafe_Callback)
    rospy.Subscriber('/robot2/odom', Odometry, Pioneer_Yaw_Callback)
    rospy.Subscriber('/hector_hiz_komut', String, Hector_Hiz_Callback)
    rospy.Subscriber('/hector_ortam_tarama', NavSatFix, Hector_Tarama_Callback)

    pub                 = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    pub_hector_bilgi    = rospy.Publisher('/hector_bilgi', String, queue_size = 1)
    pub_hector_ar_bilgi = rospy.Publisher('/hector_ar_bilgi', String, queue_size = 1)
    pub_yaw             = rospy.Publisher('/hector_yaw', String, queue_size = 1)

    rate = rospy.Rate(100)
    msg  = Twist()
    hassasiyet_konum = 0.2
    global i
    i = 0
    while not rospy.is_shutdown() :
        print(komut)
        if(komut == "Hedef GPS'e Git"):
            msg = hedefe_git(hedef.latitude, hedef.longitude)

        elif(komut == "AR PLATFORMUNA INIS"):
            arazi_bilgi = False
            if(p3at_hiz.linear.x == 0.0):
                if(hedef_uzaklik > 2.0):
                    if(ar_marker == []):
                        msg = hedefe_git(piooner_hedef.latitude,piooner_hedef.longitude)
                    else:
                        msg = hedefe_git(piooner_hedef.latitude,piooner_hedef.longitude)

                    
                else:

                    if(ar_marker == []):
                        msg = hedefe_git(piooner_hedef.latitude,piooner_hedef.longitude)

                    elif(ar_marker != []):

                        msg = marker_tespit()
                        if(break_ok == True):
                            break               
                            
                        else:
                            pass

                    else:
                        pass

            
            else :
                
                if(hedef_uzaklik > 2.0):
                    if(ar_marker == []):
                        msg = hedefe_git(piooner_hedef.latitude,piooner_hedef.longitude)
                    else:
                        msg = hedefe_git(piooner_hedef.latitude,piooner_hedef.longitude)

                else :

                    if(ar_marker == []):
                        msg = hedefe_git(piooner_hedef.latitude,piooner_hedef.longitude)


                    elif(ar_marker != []):

                        msg = hareketli_inis()
                        if(break_ok == True):
                            break               
                            
                        else:
                            pass
                    else:
                        pass
        
            #print(hedef_uzaklik)
        elif(komut == "ARACI TAKIP ET"):
            arazi_bilgi = False 
            if(ar_marker == []):
                msg = hedefe_git(piooner_hedef.latitude,piooner_hedef.longitude)
          
            elif(ar_marker != []):
                msg = hareket_takib()

            else :
                pass
            
            



        elif(komut == "ORTAM TARA"):
            noktalar_dizisi_x,noktalar_dizisi_y =  Noktalari_Bul(tarama_lat,tarama_lon)

            h_x = noktalar_dizisi_x[i]
            h_y = noktalar_dizisi_y[i]
            print(noktalar_dizisi_x[i]," ",noktalar_dizisi_y[i])
            
            a, b = xy2lanlon(h_x,h_y,lat0,lon0)
            cart_x,cart_y = get_local_coord(pozisyon.latitude,pozisyon.longitude)
            gps_to_x = cart_x
            gps_to_y = cart_y
            hedef_aci = hedef_acim(a, b)
            hedef_mesafesi1 = math.sqrt(pow(noktalar_dizisi_x[i] - gps_to_x, 2) + pow(noktalar_dizisi_y[i] - gps_to_y, 2))
    
            if hedef_mesafesi1 > 0.2:
                
                if pozisyonum.z < irtifa : 
                    msg.linear.z = 0.8
                    
                    if pozisyonum.z > 1 :
                        if hedef_aci - donus_aci > 0.1 :
                            msg.angular.z = 0.4
                        elif hedef_aci - donus_aci < -0.1 :
                            msg.angular.z = - 0.4
                        else :
                            msg.angular.z = 0.0
                            msg.linear.x = hector_hiz
                
            
                else:
                    msg.linear.z = 0
                    
                    if pozisyonum.z < irtifa + 1.0 :
                        if hedef_aci - donus_aci > 0.1 :
                            msg.angular.z = 0.4
                        elif hedef_aci - donus_aci < - 0.1 :
                            msg.angular.z = - 0.4
                        else :
                            msg.angular.z = 0.0
                            msg.linear.x = hector_hiz 
                            

                    else:
                        pass
                
            else:
                msg.linear.x = 0.0
                if(i != (len(noktalar_dizisi_x)) - 1) :
                    i = i + 1
                else:
                    i = (len(noktalar_dizisi_x)) - 1
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0


        else :
            pass
        
        

        str_yaw = str((donus_aci * 180)/ math.pi)
        #print(hector_bilgi)
        pub.publish(msg)
        pub_hector_bilgi.publish(hector_bilgi)
        pub_hector_ar_bilgi.publish(ar_bilgi)
        pub_yaw.publish(str_yaw)
        rate.sleep()

   



