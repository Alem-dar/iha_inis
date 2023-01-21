# Enis Ceri
# Hector Quadrotor ve P3AT GUI
# -*- coding: utf-8 -*-




import sys
import rospy
import threading
import math
import datetime
import cv2


from PyQt5             import QtWidgets
from PyQt5.QtCore      import QTimer
from PyQt5             import QtCore
from PyQt5             import QtGui
from PyQt5.QtGui       import *
from PyQt5.QtCore      import *
from PyQt5.QtGui       import QPainter,QBrush,QPen
from PyQt5.QtWidgets   import QComboBox

from std_msgs.msg      import String 
from sensor_msgs.msg   import NavSatFix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import Image
from PyQt5.QtGui       import QIcon, QPixmap
from cv_bridge         import CvBridge




#GLOBAL DEGISKENLER
hector_lat = 1.0
hector_lon = 2.0
hector_alt = 3.0
p3at_lat   = 1.0
p3at_lon   = 2.0
p3at_alt   = 3.0
origin_latitude  = 49.8602461528 # GAZEBO (0,0) koordinatlari
origin_longitude = 8.68707626336  
cart_x = 0.0
cart_y = 0.0
p3at_x = 0.0
p3at_y = 0.0
hector_yukseklik = 0.0
p3at_z = 0.0
nav_goal = NavSatFix()
nav_goal_p3 = NavSatFix()
tarama_GPS = NavSatFix()
komut = String()
komut_p3 = String()
p3_hiz_komut = String()
hec_bilgi = String()
ar_bilgi  = String()
irtifa = String()
takip  = String()
bridge = CvBridge()
p3at_yaw = String()
hec_yaw  = String()
p3_hiz_bilgi = String()
hec_hiz_komut = String()
hec_hiz_bilgi = String()
combo_box_index = -1


def ENLEM_BOYLAM_to_XY(latitude, longitude):
    # referans: https://en.wikipedia.org/wiki/Earth_radius, WGS84'e gore alinmistir. 
    global origin_latitude
    global origin_longitude
    global cart_x, cart_y
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
   

def mdeglon(lon0):
    lon0rad = math.radians(lon0)
    return (111415.13 *math.cos(lon0rad) - 94.55 * math.cos(3.0 *lon0rad) - 0.12 * math.cos(5.0 * lon0rad))

def mdeglat(lat0):
    lat0rad = math.radians(lat0)
    return (111132.09 - 566.05 *math.cos(2.0*lat0rad) + 1.20 * math.cos(4.0 *lat0rad) - 0.002 * math.cos(6.0 * lat0rad))

def xy2lanlon(x,y,lat0,lon0):
    y = y * (-1)
    global lat
    global lon
    lon = y / mdeglon(lat0) + lon0
    lat = x/ mdeglat(lat0) + lat0
    return lat, lon

def Hector_Pos_Callback(gps):
    global hector_lat
    global hector_lon
    global hector_alt
    hector_lat = gps.latitude
    hector_lon = gps.longitude
    hector_alt = gps.altitude

def Hector_Z_Callback(odom):
    global hector_yukseklik
    hector_yukseklik = odom.pose.position.z

def Pioneer_Pos_Callback(gps):
    global p3at_lat
    global p3at_lon
    global p3at_alt
    p3at_lat = gps.latitude
    p3at_lon = gps.longitude
    p3at_alt = gps.altitude

def Pioneer_Z_Callback(odom):
    global p3at_z
    p3at_z = odom.pose.pose.position.z

def Pioneer_Yaw_callback(yaw):
    global p3at_yaw
    p3at_yaw  = yaw.data

def Pioner_Hiz_Callback(hiz):
    global p3_hiz_bilgi
    p3_hiz_bilgi = str(hiz.linear.x)

def Hector_Hiz_Callback(hiz):
    global hec_hiz_bilgi
    hec_hiz_bilgi = str(hiz.linear.x)

def Hector_Bilgi(info):
    global hec_bilgi
    hec_bilgi = info.data

def Hector_AR_Bilgi(info_AR):
    global ar_bilgi
    ar_bilgi = info_AR.data

def Hector_Yaw_callback(yaw):
    global hec_yaw
    hec_yaw  = yaw.data


def hec_cam_callback(cam):
    global hec_cur_pix 
    hec_cur_pix = cam

class Window(QtWidgets.QWidget):
    def __init__(self):
        super(Window,self).__init__()

        self.init_ui()


    def init_ui(self):
     
        self.setFixedHeight(700)
        self.setFixedWidth(1200)


        self.hector_title         = QtWidgets.QLabel("HECTOR QUADROTOR",self)
        self.hector_konum_title   = QtWidgets.QLabel("GPS Konum",self)
        self.label_hec_lat        = QtWidgets.QLabel("Latitude    :",self)
        self.label_hec_lon        = QtWidgets.QLabel("Longitude :",self)
        self.label_hec_alt        = QtWidgets.QLabel("Altitude      :",self)
        self.gazebo_xyz_title     = QtWidgets.QLabel("Gazebo XYZ",self)
        self.label_hec_x          = QtWidgets.QLabel("Hector X :",self)
        self.label_hec_y          = QtWidgets.QLabel("Hector Y :",self)
        self.label_hec_z          = QtWidgets.QLabel("Hector Z :",self)
        self.label_komut_title    = QtWidgets.QLabel("KOMUTLAR",self)

        self.label_hec_lat_val    = QtWidgets.QLabel(self)
        self.label_hec_lon_val    = QtWidgets.QLabel(self)
        self.label_hec_alt_val    = QtWidgets.QLabel(self)

        self.label_hec_x_val      = QtWidgets.QLabel(self)
        self.label_hec_y_val      = QtWidgets.QLabel(self)
        self.label_hec_z_val      = QtWidgets.QLabel(self)

        self.label_le_enlem       = QtWidgets.QLabel("Latitude",self)
        self.label_le_boylam      = QtWidgets.QLabel("Longitude",self)
        self.label_le_x           = QtWidgets.QLabel("X",self)
        self.label_le_Y           = QtWidgets.QLabel("Y",self)
        self.label_le_hec_hiz     = QtWidgets.QLabel("HIZ",self)
        self.label_le_p3_hiz      = QtWidgets.QLabel("HIZ",self)

        self.label_p3at_title     = QtWidgets.QLabel("PIONEER KARA ARACI",self)
        self.label_p3at_konum     = QtWidgets.QLabel("GPS Konum",self)
        self.label_p3at_lat       = QtWidgets.QLabel("Latitude    :",self)
        self.label_p3at_lon       = QtWidgets.QLabel("Longitude :",self)
        self.label_p3at_alt       = QtWidgets.QLabel("Altitude     :",self)
        self.label_p3at_gazebo    = QtWidgets.QLabel("Gazebo XYZ",self)
        self.label_p3at_x         = QtWidgets.QLabel("P3AT X :",self)
        self.label_p3at_y         = QtWidgets.QLabel("P3AT Y :",self)
        self.label_p3at_z         = QtWidgets.QLabel("P3AT Z :",self)
        self.label_p3at_lat_val   = QtWidgets.QLabel(self)
        self.label_p3at_lon_val   = QtWidgets.QLabel(self)
        self.label_p3at_alt_val   = QtWidgets.QLabel(self)
        self.label_p3at_x_val     = QtWidgets.QLabel(self)
        self.label_p3at_y_val     = QtWidgets.QLabel(self)
        self.label_p3at_z_val     = QtWidgets.QLabel(self)

        self.label_p3at_komut     = QtWidgets.QLabel("KOMUTLAR",self)
        self.label_le_p3at_enlem  = QtWidgets.QLabel("Latitude",self)
        self.label_le_p3at_boylam = QtWidgets.QLabel("Longitude",self)
        self.label_le_p3at_x      = QtWidgets.QLabel("X",self)
        self.label_le_p3at_y      = QtWidgets.QLabel("Y",self)
        

        self.label_takip_mesafe   = QtWidgets.QLabel("Takip Mesafesi (m)",self)
        self.label_irtifa         = QtWidgets.QLabel("IRTIFA (m)",self)
        self.label_hector_bilgi   = QtWidgets.QLabel("BILGI",self)
        self.label_bilgi_callback = QtWidgets.QLabel(self)
        self.label_hec_cam        = QtWidgets.QLabel("HECTOR KAMERA",self)
        self.frame_cam            = QtWidgets.QLabel(self)
        self.label_p3yaw          = QtWidgets.QLabel(self)
        self.label_hec_yaw        = QtWidgets.QLabel(self)

        self.label_p3_hiz_bilgi   = QtWidgets.QLabel(self)
        self.label_hec_hiz_bilgi  = QtWidgets.QLabel(self)
        self.label_ar_bilgi       = QtWidgets.QLabel(self)
        self.label_tarama_lat     = QtWidgets.QLabel("Latitude",self)
        self.label_tarama_lon     = QtWidgets.QLabel("Longitude",self)
        self.label_komut_bilgisi  = QtWidgets.QLabel("KOMUT YOK",self)


        self.le_hedef_lat         = QtWidgets.QLineEdit(self)
        self.le_hedef_lon         = QtWidgets.QLineEdit(self)
        self.le_x                 = QtWidgets.QLineEdit(self)
        self.le_y                 = QtWidgets.QLineEdit(self)

        self.le_p3at_lat          = QtWidgets.QLineEdit(self)
        self.le_p3at_lon          = QtWidgets.QLineEdit(self)
        self.le_p3at_x            = QtWidgets.QLineEdit(self)
        self.le_p3at_y            = QtWidgets.QLineEdit(self)
        self.le_p3at_hiz          = QtWidgets.QLineEdit(self)
        self.le_hec_hiz           = QtWidgets.QLineEdit(self)

        self.le_takip             = QtWidgets.QLineEdit(self)
        self.le_irtifa            = QtWidgets.QLineEdit(self)

        self.le_tarama_lat        = QtWidgets.QLineEdit(self)
        self.le_tarama_lon        = QtWidgets.QLineEdit(self)


        self.pbt_hedefe_git       = QtWidgets.QPushButton("Hedefe Git",self)
        self.pbt_xy_to_EB         = QtWidgets.QPushButton("XY -> GPS",self)

        self.pbt_hedef_p3at       = QtWidgets.QPushButton("Hedefe Git",self)
        self.pbt_xy_to_EB_p3at    = QtWidgets.QPushButton("XY -> GPS",self)
        self.pbt_platform_inis    = QtWidgets.QPushButton("P3AT INIS",self)
        self.pbt_launch           = QtWidgets.QPushButton("ORTAMI BASLAT",self)
        self.pbt_takip            = QtWidgets.QPushButton("P3AT TAKIP",self)
        self.pbt_cam_new_W        = QtWidgets.QPushButton("Yeni Pencerede Ac",self)
        self.combo_box_cam        = QComboBox(self)
        self.pbt_tarama           = QtWidgets.QPushButton("ORTAM TARAMA",self)


        #self.hector_title.setStyleSheet("QLabel { background-color : red; color : blue }") RENK
        # RENK AYARLARI
        self.hector_title       .setStyleSheet("QLabel {color : blue}")
        self.hector_konum_title .setStyleSheet("QLabel {color : red}")
        self.gazebo_xyz_title   .setStyleSheet("QLabel {color : red}")
        self.label_komut_title  .setStyleSheet("QLabel {color : red}")
        self.label_p3at_title   .setStyleSheet("QLabel {color : blue}")

        self.pbt_hedefe_git     .setStyleSheet("background-color  : cyan")
        self.label_p3at_konum   .setStyleSheet("QLabel {color     : red}")
        self.label_p3at_gazebo  .setStyleSheet("QLabel {color     : red}")
        self.label_p3at_komut   .setStyleSheet("QLabel {color     : red}")
        self.pbt_hedef_p3at     .setStyleSheet(("background-color : cyan"))
        self.pbt_platform_inis  .setStyleSheet(("background-color : red; color : white"))
        self.pbt_launch         .setStyleSheet(("background-color : orange"))
        self.pbt_takip          .setStyleSheet(("background-color : green; color : white"))
        self.label_hector_bilgi .setStyleSheet(("background-color : green; color : white"))
        self.label_hec_cam      .setStyleSheet("QLabel {color     : blue}")
        self.pbt_cam_new_W      .setStyleSheet(("background-color : blue; color : white"))
        self.pbt_tarama         .setStyleSheet(("background-color : gray; color : white"))
        self.label_komut_bilgisi.setStyleSheet(("background-color : red; color : white"))

        # FONT AYARLARI
        self.BOLDFONT = QtGui.QFont()
        self.BOLDFONT.setBold(True)

        self.hector_title       .setFont(QFont("Times",16))
        self.hector_konum_title .setFont(QFont("Times",14))
        self.gazebo_xyz_title   .setFont(QFont("Times",14))
        self.label_komut_title  .setFont(QFont("Times",14))
        self.label_p3at_title   .setFont(QFont("Times",16))
        self.label_p3at_konum   .setFont(QFont("Times",14))
        self.label_p3at_gazebo  .setFont(QFont("Times",14))
        self.label_p3at_komut   .setFont(QFont("Times",14))
        self.label_hec_cam      .setFont(QFont("Times",16))

        #Kalin FONT
        self.label_hec_lat      .setFont (self.BOLDFONT)
        self.label_hec_lon      .setFont (self.BOLDFONT)
        self.label_hec_alt      .setFont (self.BOLDFONT)
        self.label_hec_x        .setFont (self.BOLDFONT)
        self.label_hec_y        .setFont (self.BOLDFONT) 
        self.label_hec_z        .setFont (self.BOLDFONT)
        self.pbt_hedefe_git     .setFont (self.BOLDFONT)
        self.pbt_xy_to_EB       .setFont (self.BOLDFONT)
        self.pbt_hedef_p3at     .setFont (self.BOLDFONT)
        self.pbt_xy_to_EB_p3at  .setFont (self.BOLDFONT)
        self.label_p3at_lat     .setFont (self.BOLDFONT)
        self.label_p3at_lon     .setFont (self.BOLDFONT)
        self.label_p3at_alt     .setFont (self.BOLDFONT)
        self.label_p3at_x       .setFont (self.BOLDFONT)
        self.label_p3at_y       .setFont (self.BOLDFONT)
        self.label_p3at_z       .setFont (self.BOLDFONT)
        self.pbt_platform_inis  .setFont (self.BOLDFONT)
        self.label_takip_mesafe .setFont (self.BOLDFONT)
        self.pbt_takip          .setFont (self.BOLDFONT)
        self.label_irtifa       .setFont (self.BOLDFONT)
        self.label_hector_bilgi .setFont (self.BOLDFONT)
        self.label_komut_bilgisi.setFont (self.BOLDFONT)


        # SET FIXED WIDTH
        self.label_hec_lat_val  .setFixedWidth(150)
        self.label_hec_lon_val  .setFixedWidth(150)
        self.label_hec_alt_val  .setFixedWidth(150)
        self.label_hec_x_val    .setFixedWidth(150)
        self.label_hec_y_val    .setFixedWidth(150)
        self.label_hec_z_val    .setFixedWidth(150)
        self.label_p3at_lat_val .setFixedWidth(150)
        self.label_p3at_lon_val .setFixedWidth(150)
        self.label_p3at_alt_val .setFixedWidth(150)
        self.label_p3at_x_val   .setFixedWidth(150)
        self.label_p3at_y_val   .setFixedWidth(150)
        self.label_p3at_z_val   .setFixedWidth(150)


        self.le_hedef_lat       .setFixedWidth(130)
        self.le_hedef_lon       .setFixedWidth(130)
        self.le_x               .setFixedWidth(60)
        self.le_y               .setFixedWidth(60)

        self.le_p3at_lat        .setFixedWidth(130)
        self.le_p3at_lon        .setFixedWidth(130)
        self.le_p3at_x          .setFixedWidth(60)
        self.le_p3at_y          .setFixedWidth(60)
        self.le_p3at_hiz        .setFixedWidth(60)
        self.le_hec_hiz         .setFixedWidth(60)

        self.pbt_hedefe_git     .setFixedWidth(270)
        self.pbt_xy_to_EB       .setFixedWidth(130)
        self.pbt_hedef_p3at     .setFixedWidth(270)
        self.pbt_xy_to_EB_p3at  .setFixedWidth(130)
        self.pbt_platform_inis  .setFixedWidth(270)

        self.le_takip           .setFixedWidth(90)
        self.pbt_takip          .setFixedWidth(270)
        self.le_irtifa          .setFixedWidth(90)
        self.label_bilgi_callback.setFixedWidth(300)
        self.pbt_launch         .setFixedWidth(120)
        self.label_hec_hiz_bilgi.setFixedWidth(300) 
        self.label_hec_hiz_bilgi.setFixedWidth(300) 
        self.pbt_cam_new_W      .setFixedWidth(270)
        self.combo_box_cam      .setFixedWidth(270)
        self.pbt_tarama         .setFixedWidth(270)
        self.le_tarama_lat      .setFixedWidth(130)
        self.le_tarama_lon      .setFixedWidth(130)
        self.label_komut_bilgisi.setFixedWidth(150)

        
        # MOVE

        self.hector_title         .move(80,20)
        self.hector_konum_title   .move(30,50)
        self.label_hec_lat        .move(30,80)
        self.label_hec_lon        .move(30,110)
        self.label_hec_alt        .move(30,140)

        self.label_hec_lat_val    .move(130,80)
        self.label_hec_lon_val    .move(130,110)
        self.label_hec_alt_val    .move(130,140)

        

        self.gazebo_xyz_title     .move(260,50)
        self.label_hec_x          .move(260,80)
        self.label_hec_y          .move(260,110)
        self.label_hec_z          .move(260,140)
        
        self.label_hec_x_val      .move(340,80)
        self.label_hec_y_val      .move(340,110)
        self.label_hec_z_val      .move(340,140)

        self.label_komut_title    .move(180,180)
        self.label_le_enlem       .move(55,210)
        self.label_le_boylam      .move(190,210)
        self.label_le_x           .move(325,210)
        self.label_le_Y           .move(400,210)
        self.label_le_hec_hiz     .move(475,210)

        self.le_hedef_lat         .move(30,240)
        self.le_hedef_lon         .move(170,240)
        self.le_x                 .move(310,240)
        self.le_y                 .move(380,240)
        self.le_hec_hiz           .move(450,240)


        self.pbt_hedefe_git       .move(30,270)
        self.pbt_xy_to_EB         .move(310,270)

        self.label_p3at_title     .move(650,20)
        self.label_p3at_konum     .move(600,50)
        self.label_p3at_lat       .move(600,80)
        self.label_p3at_lon       .move(600,110)
        self.label_p3at_alt       .move(600,140)
        self.label_p3at_gazebo    .move(830,50)
        self.label_p3at_x         .move(830,80)
        self.label_p3at_y         .move(830,110)
        self.label_p3at_z         .move(830,140)
        self.label_p3at_lat_val   .move(700,80)
        self.label_p3at_lon_val   .move(700,110)
        self.label_p3at_alt_val   .move(700,140)
        self.label_p3at_x_val     .move(900,80)
        self.label_p3at_y_val     .move(900,110)
        self.label_p3at_z_val     .move(900,140)
        self.label_p3at_komut     .move(750,180)
        self.label_le_p3at_enlem  .move(625,210)
        self.label_le_p3at_boylam .move(760,210)
        self.label_le_p3at_x      .move(905,210)
        self.label_le_p3at_y      .move(970,210)
        self.label_le_p3_hiz      .move(1035,210)

        self.le_p3at_lat          .move(600,240)
        self.le_p3at_lon          .move(740,240)
        self.le_p3at_x            .move(880,240)
        self.le_p3at_y            .move(950,240)
        self.le_p3at_hiz          .move(1020,240)

        self.pbt_hedef_p3at       .move(600,270)
        self.pbt_xy_to_EB_p3at    .move(880,270)
        self.pbt_platform_inis    .move(30,300)
        self.pbt_launch           .move(480,20)


        self.label_takip_mesafe   .move(320,330)
        self.le_takip             .move(340,360)
        self.pbt_takip            .move(30,360)
        self.label_irtifa         .move(490,330)
        self.le_irtifa            .move(480,360)
        self.label_hector_bilgi   .move(30,410)
        self.label_bilgi_callback .move(30,440)
        self.label_p3_hiz_bilgi   .move(30,520)
        self.label_hec_hiz_bilgi  .move(30,500)
        self.label_ar_bilgi       .move(30,520)

        self.label_p3yaw          .move(30,460)
        self.label_hec_yaw        .move(30,480)
        self.label_hec_cam        .move(650,370)
        self.frame_cam            .move(600,420)
        self.pbt_cam_new_W        .move(900,400)
        self.combo_box_cam        .move(900,450)
        self.pbt_tarama           .move(900,500)
        self.le_tarama_lat        .move(900,560)
        self.le_tarama_lon        .move(1040,560)
        self.label_tarama_lat     .move(930,530)
        self.label_tarama_lon     .move(1065,530)
        self.label_komut_bilgisi  .move(30,560)

        cart_x,cart_y  = ENLEM_BOYLAM_to_XY(hector_lat,hector_lon)
        p3at_x, p3at_y = ENLEM_BOYLAM_to_XY(p3at_lat,p3at_lon)
        
        #SET TEXT
        self.label_hec_lat_val.setText(str(hector_lat))
        self.label_hec_lon_val.setText(str(hector_lon))
        self.label_hec_alt_val.setText(str(hector_alt))
        
        self.label_hec_x_val.setText(str(cart_x))
        self.label_hec_y_val.setText(str(cart_y))
        self.label_hec_z_val.setText(str(hector_yukseklik))

        self.label_p3at_lat_val.setText(str(p3at_lat))
        self.label_p3at_lon_val.setText(str(p3at_lon))
        self.label_p3at_alt_val.setText(str(p3at_alt))

        self.label_p3at_x_val.setText(str(p3at_x))
        self.label_p3at_y_val.setText(str(p3at_y))
        self.label_p3at_z_val.setText(str(p3at_z))
        self.label_bilgi_callback.setText(str(hec_bilgi))
        self.label_p3yaw.setText("Pioneer kara aracinin anlik acisi : {} derece".format(p3at_yaw)) 
        self.label_hec_yaw.setText("Hector quadrotor anlik acisi : {} derece".format(hec_yaw)) 
        self.label_p3_hiz_bilgi.setText("Pioneer anlik hiz : {} ".format(p3_hiz_bilgi)) 
        self.label_hec_hiz_bilgi.setText("Hector anlik hiz : {} ".format(hec_hiz_bilgi)) 


        self.pbt_xy_to_EB.clicked.connect(self.click_xy_to_gps)
        self.pbt_hedefe_git.clicked.connect(self.click_hedefe_git)
        self.pbt_xy_to_EB_p3at.clicked.connect(self.click_xy_to_gps_p3at)
        self.pbt_hedef_p3at.clicked.connect(self.click_hedef_p3at)
        # self.pbt_launch.clicked.connect(self.click_launch)
        self.pbt_platform_inis.clicked.connect(self.click_hedefe_git)
        self.pbt_takip.clicked.connect(self.click_hedefe_git)

        self.pbt_cam_new_W.clicked.connect(self.click_cam_new)
        self.pbt_tarama.clicked.connect(self.click_hedefe_git)


        """
        Hector camera on gui........................................
        """
        self.kayit  = "/home/abdulhamid/hector_quadrotor_tutorial/src/hector_quadrotor/hector_quadrotor_demo/launch/aa.jpg"
        self.width  = 250
        self.height = 250

        self.hector_camera()
        self.pixmap = QPixmap(self.kayit)
        self.frame_cam.setPixmap(self.pixmap)
        self.resize(self.pixmap.width(),self.pixmap.height())

        self.dialog = Cam_New_Window()
        self.combo_box_cam.addItem("640X480")




        self.timer = QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.readSerial)
        self.timer.start()


        self.show()


    def click_cam_new(self):
        global combo_box_index
        combo_box_index = self.combo_box_cam.currentIndex()


        self.dialog.show()


    def hector_camera(self):
        self.goruntu = Image()
        self.goruntu = bridge.imgmsg_to_cv2(hec_cur_pix)
        self.goruntu = cv2.resize(self.goruntu,(250,250))
        self.kayit  = "/home/abdulhamid/hector_quadrotor_tutorial/src/hector_quadrotor/hector_quadrotor_demo/launch/aa.jpg"
        cv2.imwrite(self.kayit,self.goruntu)



    # def click(self):
    #     sender = self.sender()

    #     if sender.text() == "Clear":
    #         self.line_edit.clear()

    #     elif sender.text() == "Update":
    #         self.readSerial()  

    def paintEvent(self,e):
        painter = QPainter(self)
        painter2 = QPainter(self)


        painter.setPen(QPen(Qt.black,2,Qt.SolidLine))
        painter2.setPen(QPen(Qt.black,2,Qt.SolidLine))



        painter.drawRect(20,50,480,120)
        painter2.drawRect(580,50,480,120)



    def click_xy_to_gps(self):
        
        self.float_x = float(self.le_x.text())
        self.float_y = float(self.le_y.text())
        if(self.float_x == 0):
            self.float_x = 0.001
        else:
            pass
            
        if(self.float_y == 0):
            self.float_y = 0.001

        lat,lon = xy2lanlon(self.float_x,self.float_y,origin_latitude,origin_longitude)
        self.le_hedef_lat.setText(str(lat))
        self.le_hedef_lon.setText(str(lon))

    def click_xy_to_gps_p3at(self):
        
        self.float_p3_x = float(self.le_p3at_x.text())
        self.float_p3_y = float(self.le_p3at_y.text())
        
        if(self.float_p3_x == 0):
            self.float_p3_x = 0.001
        else:
            pass

        if(self.float_p3_y == 0):
            self.float_p3_y = 0.001
        else:
            pass

        lat,lon = xy2lanlon(self.float_p3_x,self.float_p3_y,origin_latitude,origin_longitude)
        self.le_p3at_lat.setText(str(lat))
        self.le_p3at_lon.setText(str(lon))

    def click_hedefe_git(self):
        sender = self.sender()
        global nav_goal
        global komut
        global irtifa
        global takip
        global hec_hiz_komut
        global tarama_GPS

        if(sender.text() == "Hedefe Git"):
            komut = "Hedef GPS'e Git"
            hec_hiz_komut = self.le_hec_hiz.text()



            try:
                float_irtifa = float(self.le_irtifa.text())
                if(float_irtifa >= 3.0):
                    irtifa = self.le_irtifa.text()
                    pub_irtifa.publish(irtifa)
                else:
                    irtifa = "3.0"
                    pub_irtifa.publish(irtifa)

                
            except ValueError:
                irtifa = "3.0"
                pub_irtifa.publish(irtifa)

            nav_goal.latitude  = float(self.le_hedef_lat.text())
            nav_goal.longitude = float(self.le_hedef_lon.text())
            pub_gps_goal.publish(nav_goal)
            pub_komut.publish(komut)
            pub_hec_hiz.publish(hec_hiz_komut)
            self.label_komut_bilgisi.setText("HEDEF GPS'e ILERLE")
            
        elif(sender.text() == "P3AT INIS"):
            
            try:
                float_irtifa = float(self.le_irtifa.text())
                if(float_irtifa >= 3.0):
                    irtifa = self.le_irtifa.text()
                    pub_irtifa.publish(irtifa)
                else:
                    irtifa = "3.0"
                    pub_irtifa.publish(irtifa)

                
            except ValueError:
                irtifa = "3.0"
                pub_irtifa.publish(irtifa)


            
            komut = "AR PLATFORMUNA INIS"
            pub_komut.publish(komut)
            self.label_komut_bilgisi.setText("AR TAG'e IN")

        elif(sender.text() == "P3AT TAKIP"):

            try:
                float_irtifa = float(self.le_irtifa.text())
                if(float_irtifa >= 3.0):
                    irtifa = self.le_irtifa.text()
                    pub_irtifa.publish(irtifa)
                else:
                    irtifa = "3.0"
                    pub_irtifa.publish(irtifa)


                
                
            except ValueError:
                irtifa = "3.0"
                pub_irtifa.publish(irtifa)   

            try:
                float_takip_mesafe = float(self.le_takip.text())
                if(float_takip_mesafe > float_irtifa - 2.0):
                    float_takip_mesafe = float_irtifa - 2.0
                    takip = str(float_takip_mesafe)
                    pub_takip_mesafe.publish(takip)
                else:
                    takip = self.le_takip.text()
                    pub_takip_mesafe.publish(takip)
                
            except:
                takip = "0.0"
                pub_takip_mesafe.publish(takip)      



            komut = "ARACI TAKIP ET"
            pub_komut.publish(komut)
            self.label_komut_bilgisi.setText("ARACI TAKIP ET")

        elif(sender.text() == "ORTAM TARAMA"):

            komut = "ORTAM TARA"
            tarama_GPS.latitude  = float(self.le_tarama_lat.text())
            tarama_GPS.longitude = float(self.le_tarama_lon.text())
            pub_komut.publish(komut)
            pub_tarama_GPS.publish(tarama_GPS)
            self.label_komut_bilgisi.setText("ORTAMI KESFET")



        else:
            pass
            
        

   
    def click_hedef_p3at(self):
        global nav_goal_p3
        global komut_p3
        global p3_hiz_komut
        komut_p3 = "Hedef GPS'e Git"
        nav_goal_p3.latitude = float(self.le_p3at_lat.text())
        nav_goal_p3.longitude = float(self.le_p3at_lon.text())
        p3_hiz_komut  = self.le_p3at_hiz.text()
        pub_p3_gps.publish(nav_goal_p3)
        pub_komut_p3.publish(komut_p3)
        pub_p3_hiz.publish(p3_hiz_komut)
    
    # def click_launch(self):
    #     ROS_PROGRAM = QProcess(self)
    #     program = "roslaunch hector_quadrotor_demo outdoor.launch "
    #     ROS_PROGRAM.start(program)
        
  
  

    
    def readSerial(self):

        self.update_label()
        # send = self.line_edit.text()
        # pub.publish(send)

    
    def update_label(self):
        cart_x,cart_y = ENLEM_BOYLAM_to_XY(hector_lat,hector_lon)
        p3at_x,p3at_y = ENLEM_BOYLAM_to_XY(p3at_lat,p3at_lon)


        self.label_hec_lat_val.setText(str(hector_lat))
        self.label_hec_lon_val.setText(str(hector_lon))
        self.label_hec_alt_val.setText(str(hector_alt))
        self.label_hec_x_val.setText(str(cart_x))
        self.label_hec_y_val.setText(str(cart_y))
        self.label_hec_z_val.setText(str(hector_yukseklik))

        self.label_p3at_lat_val.setText(str(p3at_lat))
        self.label_p3at_lon_val.setText(str(p3at_lon))
        self.label_p3at_alt_val.setText(str(p3at_alt))

        self.label_p3at_x_val.setText(str(p3at_x))
        self.label_p3at_y_val.setText(str(p3at_y))
        self.label_p3at_z_val.setText(str(p3at_z))
        self.label_bilgi_callback.setText(str(hec_bilgi))





        

        self.label_p3yaw.setText("Pioneer kara aracinin anlik acisi : {} derece".format(p3at_yaw)) 
        self.label_hec_yaw.setText("Hector quadrotor anlik acisi : {} derece".format(hec_yaw)) 
        self.label_p3_hiz_bilgi.setText("Pioneer anlik hiz : {} ".format(p3_hiz_bilgi)) 
        self.label_hec_hiz_bilgi.setText("Hector anlik hiz : {} ".format(hec_hiz_bilgi)) 

        pub_gps_goal.publish(nav_goal)
        pub_komut.publish(komut)
        pub_p3_gps.publish(nav_goal_p3)
        pub_komut_p3.publish(komut_p3)
        pub_irtifa.publish(irtifa)
        pub_takip_mesafe.publish(takip)
        pub_p3_hiz.publish(p3_hiz_komut)
        pub_hec_hiz.publish(hec_hiz_komut)
        pub_tarama_GPS.publish(tarama_GPS)

        self.hector_camera()
        self.pixmap = QPixmap(self.kayit)
        self.frame_cam.setPixmap(self.pixmap)
        self.resize(self.pixmap.width(),self.pixmap.height())


class Cam_New_Window(QtWidgets.QWidget):
    def __init__(self):
        super(Cam_New_Window,self).__init__()
        self.frame_cam            = QtWidgets.QLabel(self)
        self.frame_cam.move(10,10)
        self.kayit  = "/home/abdulhamid/hector_quadrotor_tutorial/src/hector_quadrotor/hector_quadrotor_demo/launch/aa.jpg"
        self.cam_width = 640
        self.cam_height = 480
      
        self.timer = QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.update)
        self.timer.start()





        self.hector_camera()

        self.pixmap = QPixmap(self.kayit)
        self.frame_cam.setPixmap(self.pixmap)
        self.resize(self.pixmap.width(),self.pixmap.height())



    def hector_camera(self):
        self.goruntu = Image()
        self.goruntu = bridge.imgmsg_to_cv2(hec_cur_pix)
      
        self.goruntu = cv2.resize(self.goruntu,(self.cam_width,self.cam_height))
        self.kayit  = "/home/abdulhamid/hector_quadrotor_tutorial/src/hector_quadrotor/hector_quadrotor_demo/launch/aa.jpg"
        cv2.imwrite(self.kayit,self.goruntu)

    def update(self):
        self.hector_camera()
        self.pixmap = QPixmap(self.kayit)
        self.frame_cam.setPixmap(self.pixmap)
        self.resize(self.pixmap.width(),self.pixmap.height())


    
        

if __name__ == '__main__':

    while not rospy.is_shutdown():


        rospy.Subscriber("/fix", NavSatFix, Hector_Pos_Callback)
        rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, Hector_Z_Callback)
        rospy.Subscriber('/robot2/odom', Odometry, Pioneer_Z_Callback)
        rospy.Subscriber('/gps/fix', NavSatFix, Pioneer_Pos_Callback)
        rospy.Subscriber('/hector_bilgi', String, Hector_Bilgi)
        rospy.Subscriber("/downward_cam/camera/image", Image, hec_cam_callback)
        rospy.Subscriber("/pioneer_yaw", String, Pioneer_Yaw_callback)
        rospy.Subscriber("/hector_yaw",  String, Hector_Yaw_callback)
        rospy.Subscriber("/robot2/cmd_vel",  Twist, Pioner_Hiz_Callback)
        rospy.Subscriber("/cmd_vel",  Twist, Hector_Hiz_Callback)
        rospy.Subscriber("/hector_ar_bilgi",  String, Hector_AR_Bilgi)



        rospy.init_node('ara_yuz', anonymous=True)
        pub = rospy.Publisher('enis_gonderiyor', String, queue_size=10)
    
        pub_gps_goal     = rospy.Publisher("/komut_gps_git",NavSatFix,queue_size = 1)
        pub_komut        = rospy.Publisher("/komut", String, queue_size = 1)
        pub_p3_gps       = rospy.Publisher("/komut_gps_p3at",NavSatFix,queue_size = 1)
        pub_komut_p3     = rospy.Publisher("/komut_p3at",String, queue_size = 1)
        pub_irtifa       = rospy.Publisher("/hector_irtifa",String, queue_size = 1)
        pub_takip_mesafe = rospy.Publisher("/hector_artag_takip_mesafe",String, queue_size = 1)
        pub_p3_hiz       = rospy.Publisher("/pioneer_hiz_komut",String, queue_size = 1)
        pub_hec_hiz      = rospy.Publisher("/hector_hiz_komut",String, queue_size = 1)
        pub_tarama_GPS   = rospy.Publisher("/hector_ortam_tarama",NavSatFix, queue_size = 1)

        app = QtWidgets.QApplication(sys.argv)
        
        w = Window()  

                                                                                                                                                
        thr = threading.Thread(target=Window)
        thr.start()
        



        sys.exit(app.exec_())


