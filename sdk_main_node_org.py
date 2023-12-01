#!/usr/bin/env python3
import json
import logging
import time
from time import gmtime, strftime
import rospy
import subprocess
import argparse
import psutil
import os, sys
from std_msgs.msg import String
from terra_utm_sdk.msg import TelemetryData
from terra_utm_sdk.msg import UTMMessage
from terra_utm_sdk.msg import UTMMessageRes

# opencv
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
#sys.path.remove('/root/ros_catkin_ws/install_isolated/lib/python2.7/dist-packages')
import cv2 
from datetime import datetime as dt
import numpy as np
from model import Model
import numpy

import threading
import datetime


manufacturer = rospy.get_param('manufacturer')
serial_id = rospy.get_param('serial_id')
clientId = rospy.get_param('thing_name')
pub_topic = manufacturer + '/' + serial_id + '/drone'
print(pub_topic)
parser = argparse.ArgumentParser(description='Display WLAN signal strength.')
parser.add_argument(dest='interface', nargs='?', default='wlan0', help='wlan interface (default: wlan0)')
args = parser.parse_args([])


global release_version
global RELEASE_VERSION_202203
RELEASE_VERSION_202203 = "202203"   #202203実証実験分
global RELEASE_VERSION_202204
RELEASE_VERSION_202204 = "202204"   #202204ソース整理時

# 動作バージョン指定
release_version = RELEASE_VERSION_202203
#release_version = RELEASE_VERSION_202204



global objectdetection_target_model
global OBJECTDETECTION_TARGET_MODEL_RESNET50
OBJECTDETECTION_TARGET_MODEL_RESNET50 = "resnet50"      #resnet50_ssd_model
global OBJECTDETECTION_TARGET_MODEL_INSULATOR
OBJECTDETECTION_TARGET_MODEL_INSULATOR = "insulator"    #insulator(がいし)

# 物体検知対象モデル指定
objectdetection_target_model = OBJECTDETECTION_TARGET_MODEL_INSULATOR
#objectdetection_target_model = OBJECTDETECTION_TARGET_MODEL_RESNET50

global target_model_class_indexes

#insulator classes
class_insulator = 0         
class_insulator660_t = 0    #0に66kvの耐張が入っている前提
class_insulator220_t = 1
class_insulator220_k = 2
class_insulator500_t = 3
class_insulator500_k = 4
class_insulator_t = 5       #5に220kv、500kvの耐張が入っている前提
class_insulator_k = 6       #6に220kv、500kvの懸垂が張っている前提

#input
global input_mode
global INPUT_MODE_GSTREAMER_UDP
INPUT_MODE_GSTREAMER_UDP = "gstreamer_udp"  #GstreamerでUDP入力(Matrice300のカメラから取得)
global INPUT_MODE_GSTREAMER_UDP_STR
INPUT_MODE_GSTREAMER_UDP_STR = 'udpsrc port=50002 buffer-size=2129920 caps=\"application/x-rtp,payload=\(int\)96,encoding-name=\(string\)H264,width=1280,height=720\"  ! rtph264depay ! avdec_h264 !  videoscale ! video/x-raw,width=1280,height=720 ! videoconvert ! appsink sync=false' #20220328 jishou OK
#INPUT_MODE_GSTREAMER_UDP_STR = 'udpsrc address=\"192.168.10.176\" port=50002 buffer-size=2129920 caps=\"application/x-rtp,payload=\(int\)96,encoding-name=\(string\)H264,width=1280,height=720\"  ! rtph264depay ! avdec_h264 !  videoscale ! video/x-raw,width=1280,height=720 ! videoconvert ! appsink sync=false' #20220328 jishou OK
# INPUT_MODE_GSTREAMER_UDP_STR = 'udpsrc port=50002 buffer-size=2129920 caps=\"application/x-rtp,payload=\(int\)96,encoding-name=\(string\)H264,width=1280,height=720\"  ! rtph264depay ! avdec_h264 !  videoscale ! video/x-raw,width=1280,height=720 ! videoconvert ! appsink sync=false' #20220328 jishou OK

global INPUT_MODE_GSTREAMER_URI
INPUT_MODE_GSTREAMER_URI = "gstreamer_uri"  #Gstreamerでストリーミングサーバの映像を取得
global INPUT_MODE_GSTREAMER_URI_STR
INPUT_MODE_GSTREAMER_URI_STR = 'souphttpsrc location=http://192.168.10.185:8090/test.mjpg !  jpegdec ! videoconvert ! appsink sync=false' #another server 
# INPUT_MODE_GSTREAMER_URI_STR = 'souphttpsrc location=http://127.0.0.1:8090/test.mjpg !  jpegdec ! videoconvert ! appsink sync=false' #remotecontroller2 
# INPUT_MODE_GSTREAMER_URI_STR = 'souphttpsrc location=http://192.168.10.185:8090/test.mjpg !  jpegdec ! videoconvert ! appsink sync=false' #another server 

global INPUT_MODE_FILE
INPUT_MODE_FILE = "file"                    #動画や画像ファイルから指定

global INPUT_MODE_FILE_FILE_PATH
#INPUT_MODE_FILE_FILE_PATH = '/mnt/hgfs/td_objectdetection/No.37/DJI_20220315153828_0001_W.MP4'        
INPUT_MODE_FILE_FILE_PATH = '/home/terra/No.37/DJI_20220315153828_0001_W.MP4'        

# 映像入力モード指定
input_mode = INPUT_MODE_GSTREAMER_UDP
#input_mode = INPUT_MODE_FILE

#model
if  objectdetection_target_model == OBJECTDETECTION_TARGET_MODEL_RESNET50:
    # resnet50 モデル
    MODEL_PATH = '/home/terra/models/resnet50_ssd_model/resnet50_ssd_model' #resnet50(人や物を対象にしてテストする用)

    # クラスと表示色 resnet50 用
    CLASSES = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']
    COLORS = [(0,0,175),(175,0,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0),(0,175,0)]

    # 対象のクラス
    class_resnet50_human = 14
    target_model_class_indexes = [class_resnet50_human]

else :
    # inculator モデル
    MODEL_PATH = '/home/terra/models/deployed_model/deploy_model_algo_1'    #20220314_02

    # クラスと表示色 insulator 用
    CLASSES = ['insulator66_t','insulator220_t','insulator220_k','insulator500_t','insulator500_k','insulator_t', 'insulator_k'] #20220314
    COLORS = [(0,0,175),(175,0,0),(0,175,0),(0,0,75),(75,0,0),(0,75,0),(0,0,105)]

    # 対象のクラス
    #target_model_class_indexes = [class_insulator660_t,class_insulator_t, class_insulator_k]
    target_model_class_indexes = [class_insulator660_t,class_insulator_t]


#OUTPUT_PATH = './output' # 画像を出力するフォルダ
OUTPUT_PATH = '/home/terra/screenshot/output' # 画像を出力するフォルダ
#INTERVAL = 24 # フレーム再生間隔 #for manifold2c
if release_version == RELEASE_VERSION_202203:
    INTERVAL = 8 # フレーム再生間隔 #for gpdwin3 202203実証時
elif release_version == RELEASE_VERSION_202204 :
    INTERVAL = 4 # フレーム再生間隔 #for gpdwin3 test20220401
else :
    INTERVAL = 8 # フレーム再生間隔 #for gpdwin3 202203実証時

# CLASSDATA_NAME = ["confidence","detection_centerX","detection_centerY","boundingbox_width","boundingbox_height","detection_square"]


#中心厳格判定モード(バウンディングボックスが中央の視覚より小さい場合は中心判定を厳しくする)
global flg_target_strict_judgement
flg_target_strict_judgement = True      #OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOMのときはFalseにする
# flg_target_strict_judgement = False     


#add
global previous_waypoint_index 
global current_waypoint_index 
global flg_objectdetection_test
global flg_objectdetection_test2
previous_waypoint_index = 0
current_waypoint_index = 0
flg_objectdetection_test = True     #物体検知処理を起動するための一回だけ使用するフラグ(要改善)
flg_objectdetection_test2 = False   #自動で物体検知起動フラグ

global relatedX_old
global relatedY_old

global camera_width     #カメラ幅
global camera_height    #ガメラ高さ
global maxCornerX       #初期ジンバル最大角度X
global maxCornerY       #初期ジンバル最大角度Y
global gimbalCornerX    #ジンバル角度X
global gimbalCornerY    #ジンバル角度Y
global relatedX_old
global relatedY_old
global cornerbias
global targetSquareWidth
global targetSquareHeight
global corner_add_value_X   #test20220307
global corner_add_value_Y   #test20220307

maxCornerX = 7        #初期ジンバル最大角度X    #202201の実証時の値
maxCornerX = 12       #初期ジンバル最大角度X    #test20220307 
maxCornerY = 7        #初期ジンバル最大角度Y    #202201の実証時の値

corner_add_value_X = maxCornerX/2           #test20220307
corner_add_value_Y = maxCornerY/2           #test20220307

targetSquareWidth = 60  #初期中央四角(横)
targetSquareHeight = 60 #初期中央四角(縦)
targetSquareWidth = 120  #初期中央四角(横)          20211015
targetSquareHeight = 120 #初期中央四角(縦)          20211015

global zoomValueCurrent
zoomValueCurrent = 2

global confidence_bias
confidence_bias = 0.3   #認識率閾値

#addstart 20220310
global confidence_bias_taicho
global confidence_bias_kensui
global confidence_bias_taicho_wide_to_zoom 
confidence_bias_taicho = 0.3
confidence_bias_kensui = 0.2
confidence_bias_taicho_wide_to_zoom = 0.1
#addend 20220310
global confidence_bias_show #add 20220330
confidence_bias_show = 0.1  #add 20220330
 
global  sampling_count  #サンプリング数
global  group_bias      #グループ閾値
global  zoom_bias       #ズーム終了閾値

# sampling_count = 3      #サンプリング数 #for manifold2c
if release_version == RELEASE_VERSION_202203:
    sampling_count = 5      #サンプリング数 #for gpdwin3  202203実証時
elif release_version == RELEASE_VERSION_202204:
    sampling_count = 10      #サンプリング数 #for gpdwin3 test20220401
else :
    sampling_count = 5      #サンプリング数 #for gpdwin3  202203実証時

# group_bias = 30         #グループ閾値
group_bias = 100         #グループ閾値  20211015 #202203実証時

# zoom_bias = 0.8        #ズーム終了閾値 #202203実証時
zoom_bias = 1.1        #ズーム終了閾値 20220414ズームの割合で終了させないために1より大きい値を設定。

global wait_zoom
global wait_gimbal
global wait_camera_change

wait_zoom = 3           
wait_gimbal = 3 #20220304
wait_camera_change = 1

global objectdetection_mode #center,area,confidence
global OBJECTDETECTION_MODE_DEFAULT
OBJECTDETECTION_MODE_DEFAULT = "area"
OBJECTDETECTION_MODE_DEFAULT = "center"

objectdetection_mode = OBJECTDETECTION_MODE_DEFAULT

global objectdetection_tower_mode
global objectdetection_tower_mode_taicho
global objectdetection_tower_mode_kensui
global OBJECTDETECTION_TOWER_MODE_TAICHO_220KV
global OBJECTDETECTION_TOWER_MODE_KENSUI_500KV
OBJECTDETECTION_TOWER_MODE_TAICHO_220KV = "taicho_220kv"
OBJECTDETECTION_TOWER_MODE_KENSUI_500KV = "kensui_500kv"
OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM = "taicho_wide_to_zoom"
#objectdetection_tower_mode_taicho = OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM
objectdetection_tower_mode_taicho = OBJECTDETECTION_TOWER_MODE_TAICHO_220KV
objectdetection_tower_mode_kensui = OBJECTDETECTION_TOWER_MODE_KENSUI_500KV
objectdetection_tower_mode = objectdetection_tower_mode_taicho  #起動時のデフォルト設定(DJI Onobard SDKから来るテレメトリで変わることがある)


global objectdetection_status #test20211128
global objectdetection_status_old #test20211128
global OBJECTDETECTION_STATUS_START
global OBJECTDETECTION_STATUS_NO_ACTION
OBJECTDETECTION_STATUS_START = "objectdetection_start"
OBJECTDETECTION_STATUS_NO_ACTION = "no_action"
objectdetection_status_old = OBJECTDETECTION_STATUS_NO_ACTION


global gimbal_pitch
global gimbal_roll
global gimbal_yaw
global gimbal_mode
global gimbal_mode_old
global GIMBAL_MODE_FREE
GIMBAL_MODE_FREE = 0
global GIMBAL_MODE_FOLLOW
GIMBAL_MODE_FOLLOW = 2
gimbal_mode_old = -1

global gimbal_pitch_saved
global gimbal_roll_saved
global gimbal_yaw_saved  

global mission_status               #test20211124
global MISSION_STATUS_PAUSE         #test20211124
MISSION_STATUS_PAUSE = 4            #test20211124
global MISSION_STATUS_FLYING         #test20211125
MISSION_STATUS_FLYING = 3            #test20211125

global centerX
global centerY

#ズーム倍率
global ZoomValuex2
global ZoomValuex4
global ZoomValuex5
global ZoomValuex6
global ZoomValuex8
global ZoomValuex10
global ZoomValuex12
global ZoomValuex16
global ZoomValuex20      
ZoomValuex2 = 2
ZoomValuex4 = 4
ZoomValuex5 = 5
ZoomValuex6 = 6
ZoomValuex8 = 8
ZoomValuex10 = 10
ZoomValuex12 = 12
ZoomValuex16 = 16
ZoomValuex20 = 20      
global ZoomValues
ZoomValues = [ZoomValuex2,ZoomValuex4,ZoomValuex5,ZoomValuex6,ZoomValuex8,ZoomValuex10,ZoomValuex12,ZoomValuex16,ZoomValuex20]


global currentCameraLens
global CAMERALENS_WIDE
CAMERALENS_WIDE = "wide"
global CAMERALENS_ZOOM
CAMERALENS_ZOOM = "zoom"
global CAMERALENS_DUMMY_PAUSE
CAMERALENS_DUMMY_PAUSE = "pause"
currentCameraLens = CAMERALENS_WIDE

global OSDK_CAMERA_SOURCE_H20T_WIDE
global OSDK_CAMERA_SOURCE_H20T_ZOOM
global OSDK_CAMERA_SOURCE_H20T_IR
OSDK_CAMERA_SOURCE_H20T_WIDE = 1
OSDK_CAMERA_SOURCE_H20T_ZOOM = 2
OSDK_CAMERA_SOURCE_H20T_IR = 3


global current_aircraft_waypoint    #test20211128
current_aircraft_waypoint = -1      #test20211128
 
global flg_gimbal_variable_waiting_time
flg_gimbal_variable_waiting_time = False #202203の実証時はFalseと同等で実行
# flg_gimbal_variable_waiting_time = True 

class TerraUTMSDKMainNode():

    def __init__(self):
        self.watchdog_sub = rospy.Subscriber('watchdog', String, self.watchdogCallback)
        self.utm_pub = rospy.Publisher('utm_message', UTMMessage, queue_size=10)
        self.utm_sub = rospy.Subscriber('utm_message_res', UTMMessageRes, self.utmMessageResCallback)
        self.telemetry_sub = rospy.Subscriber('telemetry', TelemetryData, self.telemetryCallback)
        self.start_time = 0


    def utmMessageResCallback(self, data):
        try:
            j = json.dumps({"type":data.type,"cmd":data.cmd,"result":data.result,"flight_id":data.flight_id})
            print(j)
        except Exception as e:
            _, _, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print("utmMessageResCallback: Json dump error -> {}, file:{} line: {}".format(e, fname, exc_tb.tb_lineno))


    def telemetryCallback(self, data):
        
        global gimbal_pitch
        global gimbal_roll
        global gimbal_yaw
        global gimbal_mode
        global gimbal_mode_old
        global GIMBAL_MODE_FREE
        global GIMBAL_MODE_FOLLOW
        global flg_objectdetection_test2
        global objectdetection_mode
        global OBJECTDETECTION_MODE_DEFAULT   

        global gimbal_pitch_saved
        global gimbal_roll_saved
        global gimbal_yaw_saved

        global ZoomValuex2

        global mission_status       #test20211124
        global MISSION_STATUS_PAUSE #test20211124
        global MISSION_STATUS_FLYING         #test20211125

        global objectdetection_status #test20211128
        global objectdetection_status_old #test20211128
        global OBJECTDETECTION_STATUS_START
        global OBJECTDETECTION_STATUS_NO_ACTION

        global current_aircraft_waypoint    #test20211128


        global objectdetection_tower_mode   #test20220202
        global OBJECTDETECTION_TOWER_MODE_TAICHO_220KV #test20220202
        global OBJECTDETECTION_TOWER_MODE_KENSUI_500KV #test02020202
        global OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM #test20220330

        global confidence_bias          #20220314
        global confidence_bias_taicho   #20220314
        global confidence_bias_kensui   #20220314
        global confidence_bias_show     #20220330

        try:
            gimbal_pitch = data.gimbal_pitch
            gimbal_roll = data.gimbal_roll
            gimbal_yaw = data.gimbal_yaw
            gimbal_mode = data.gimbal_mode

            mission_status = data.mission_status #test20211125

            objectdetection_status = data.objectdetection_status #test20211128

            current_aircraft_waypoint = data.waypoint   #test20211128

            #test20220202
            if data.objectdetection_mode == 0:
                # objectdetection_tower_mode = OBJECTDETECTION_TOWER_MODE_TAICHO_220KV
                objectdetection_tower_mode = objectdetection_tower_mode_taicho
                confidence_bias = confidence_bias_taicho    #20220314
            else:
                # objectdetection_tower_mode = OBJECTDETECTION_TOWER_MODE_KENSUI_500KV
                objectdetection_tower_mode = objectdetection_tower_mode_kensui
                confidence_bias = confidence_bias_kensui    #20220314
            #test20220202

            try:
                gimbal_pitch_saved
            except NameError:
                print("error test")
                gimbal_pitch_saved = gimbal_pitch

            try:
                gimbal_yaw_saved
            except NameError:
                print("error test")
                gimbal_yaw_saved = gimbal_yaw

            # print("gimbal_pitch:{},gimbal_roll:{},gimbal_yaw:{},gimbal_mode:{},gimbal_mode_old:{}".format(gimbal_pitch,gimbal_roll,gimbal_yaw,gimbal_mode,gimbal_mode_old))
            print("gimbal_pitch:{},gimbal_roll:{},gimbal_yaw:{},gimbal_mode:{},gimbal_mode_old:{},mission_status:{},MISSION_STATUS_PAUSE:{}".format(gimbal_pitch,gimbal_roll,gimbal_yaw,gimbal_mode,gimbal_mode_old,mission_status,MISSION_STATUS_PAUSE))
            print("objectdetection_status:{}".format(objectdetection_status))   #test20211128
            if ((objectdetection_status != objectdetection_status_old)): #test20211128 ジンバルモード切替なし
                print("モード切替！")
                if((objectdetection_status == OBJECTDETECTION_STATUS_START)): #test20211128 ジンバルモード切替なし
                    #物体検知処理有効
                    flg_objectdetection_test2 = True
                    objectdetection_mode = OBJECTDETECTION_MODE_DEFAULT
                    #ジンバル角度保存
                    gimbal_pitch_saved = gimbal_pitch
                    gimbal_roll_saved = gimbal_roll
                    gimbal_yaw_saved = gimbal_yaw

                    print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gimbal_yaw_saved, gimbal_pitch_saved))
                    print("物体検知開始！")
                    pass
                elif((objectdetection_status == OBJECTDETECTION_STATUS_NO_ACTION)) : #test20211128 ジンバルモード切替なし
                    #物体検知処理無効
                    print("gimbalcornerReset")
                    self.gimbalcornerReset()
                    # currentCameraLens = CAMERALENS_WIDE
                    self.cameraControlZoom(ZoomValuex2)
                    print("objectdetection off")
                    flg_objectdetection_test2 = False
                    #ジンバル角度復元
                    print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gimbal_yaw_saved, gimbal_pitch_saved))
                    self.gimbalControlYawTiltLargeAbsolute(gimbal_yaw_saved, gimbal_pitch_saved)

                    print("==========物体検知終了==========")
            gimbal_mode_old = gimbal_mode
            objectdetection_status_old = objectdetection_status #test20211128
        except Exception as e:
            print("telemetry error:{}".format(e))
            pass

        return


    def watchdogCallback(self, data):
        # payload = data
        pass


    def start(self):
        result = True                       #change 20220302
        if result != False :
           trhead1 =  threading.Thread(target=self.objectdetection0)
           trhead1.start()

        return result


    def on_click(event, x, y, flags, frame):
        if event == cv2.EVENT_LBUTTONUP:
            tdatetime = dt.now()
            date_str = tdatetime.strftime('%Y%m%d_%H%M%S')
            filename = "{}/{}.jpg".format(OUTPUT_PATH, date_str)
            cv2.imwrite(filename, frame)
            print("Saved. {}".format(filename))


    def onMouseClick(self,event,x,y,flags,param):
        global flg_objectdetection_test2

        global gimbal_pitch
        global gimbal_roll
        global gimbal_yaw

        global gimbal_pitch_saved
        global gimbal_roll_saved
        global gimbal_yaw_saved

        global camera_width     #カメラ幅
        global camera_height    #カメラ高さ

        global currentCameraLens
        global CAMERALENS_WIDE
        global CAMERALENS_ZOOM
        global CAMERALENS_DUMMY_PAUSE

        global ZoomValuex2

        global targetSquareWidth
        global targetSquareHeight


        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)
            print("click camera_width:{},camera_height:{}x:{}, y:{}".format(camera_width,camera_height,x,y))

            if x >= camera_width // 2 - targetSquareWidth // 2 and x <= camera_width // 2 + targetSquareWidth // 2 and y >= camera_height // 2 - targetSquareHeight // 2 and y <= camera_height // 2 + targetSquareHeight // 2 :
                #中心付近をクリックすると物体検知処理無効
                print("gimbalcornerReset")
                self.gimbalcornerReset()
                # currentCameraLens = CAMERALENS_WIDE
                print("objectdetection off")
                flg_objectdetection_test2 = False
            elif x >= 0 and x <= 60 and y >= 0 and y <= 60 :
                #左上部分をクリックすると物体検知処理有効
                print("objectdetection on")
                flg_objectdetection_test2 = True

                #ジンバル角度保存
                gimbal_pitch_saved = gimbal_pitch
                gimbal_roll_saved = gimbal_roll
                gimbal_yaw_saved = gimbal_yaw
            elif x >= 0 and x <= 120 and y >= camera_height - 60 and y <= camera_height :
                #レンズ切り替え
                if (currentCameraLens == CAMERALENS_DUMMY_PAUSE) :
                    #処理再開
                    print("objectdetection continue(change lens(Pause->Zoom))")
                    currentCameraLens = CAMERALENS_ZOOM
                elif currentCameraLens == CAMERALENS_WIDE :
                    #処理一時停止
                    print("objectdetection pause(change lens(Wide->Pause))")
                    currentCameraLens = CAMERALENS_DUMMY_PAUSE
                elif currentCameraLens == CAMERALENS_ZOOM :
                    #処理一時停止
                    print("objectdetection pause(change lens(Wide->Pause))")
                    currentCameraLens = CAMERALENS_DUMMY_PAUSE
            elif x >= camera_width - 120 and x <= camera_width and y >= camera_height - 60 and y <= camera_height :
                    print("objectdetection wide(change lens(all->Wide))")
                    currentCameraLens = CAMERALENS_WIDE

                    #パラメータ初期化
                    self.gimbalcornerReset()
                    #ズーム初期値
                    self.cameraControlZoom(ZoomValuex2)
            #addstart 20220330
            elif x >= 0 and x <= 60 and y >= 80 and y <= 120 :
                #カメラ切替
                if currentCameraLens == CAMERALENS_WIDE:
                    currentCameraLens = CAMERALENS_ZOOM
                    self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_ZOOM)
                else:
                    currentCameraLens = CAMERALENS_WIDE
                    self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_WIDE)

            elif x >= 0 and x <= 60 and y >= 120 and y <= 160 :
                #x2
                self.cameraControlZoom(ZoomValuex2)
            elif x >= 0 and x <= 60 and y >= 160 and y <= 200 :
                #x4
                self.cameraControlZoom(ZoomValuex4)
            elif x >= 0 and x <= 60 and y >= 200 and y <= 240 :
                #x5
                self.cameraControlZoom(ZoomValuex5)
            elif x >= 0 and x <= 60 and y >= 240 and y <= 280 :
                #x6
                self.cameraControlZoom(ZoomValuex6)
            elif x >= 0 and x <= 60 and y >= 280 and y <= 320 :
                #x8
                self.cameraControlZoom(ZoomValuex8)
            elif x >= 0 and x <= 60 and y >= 320 and y <= 360 :
                #x10
                self.cameraControlZoom(ZoomValuex10)
            elif x >= 0 and x <= 60 and y >= 360 and y <= 400 :
                #x12
                self.cameraControlZoom(ZoomValuex12)
            elif x >= 0 and x <= 60 and y >= 400 and y <= 440 :
                #x16
                self.cameraControlZoom(ZoomValuex16)
            elif x >= 0 and x <= 60 and y >= 440 and y <= 480 :
                #x20
                self.cameraControlZoom(ZoomValuex20)
            #addend 202203330


            else : 
                self.gimbalControlYawTiltLarge(x,y)

    def gimbalcornerReset(self):
        global gimbalCornerX
        global gimbalCornerY
        global maxCornerX
        global maxCornerY
        global zoomValueCurrent

        global currentCameraLens
        global CAMERALENS_WIDE
        global CAMERALENS_ZOOM
        global CAMERALENS_DUMMY_PAUSE
        
        gimbalCornerX = maxCornerX
        gimbalCornerY = maxCornerY
        zoomValueCurrent = 2                #ズーム初期化

        currentCameraLens = CAMERALENS_WIDE #レンズはwideで初期化
    
    def gimbalcornerResetOnlyCorner(self):
        global gimbalCornerX
        global gimbalCornerY
        global maxCornerX
        global maxCornerY
        global zoomValueCurrent

        global currentCameraLens
        global CAMERALENS_WIDE
        global CAMERALENS_ZOOM
        global CAMERALENS_DUMMY_PAUSE
        
        gimbalCornerX = maxCornerX
        gimbalCornerY = maxCornerY    

    def gimbalControlYawTiltLarge(self,x,y):
        global relatedX_old
        global relatedY_old

        global camera_width     #カメラ幅
        global camera_height    #ガメラ高さ
        global maxCornerX       #初期ジンバル最大角度X
        global maxCornerY       #初期ジンバル最大角度Y
        global gimbalCornerX    #ジンバル角度X
        global gimbalCornerY    #ジンバル角度Y
        global relatedX_old
        global relatedY_old
        global cornerbias
        global corner_add_value_X   #test20220307
        global corner_add_value_Y   #test20220307
        global zoomValueCurrent

        relatedX = x
        relatedY = y 

        #  //X軸移動
        if relatedX < centerX:
            diffCenterX = centerX - relatedX

            if relatedX_old != -1:
                if relatedX_old < centerX :
                    gimbalCornerX = gimbalCornerX + corner_add_value_X  #test20220307
                elif relatedX_old > centerX :
                    gimbalCornerX = gimbalCornerX - corner_add_value_X  #test20220307
                  
            yaw = -1 * gimbalCornerX * (diffCenterX / centerX)
            yaw = yaw * cornerbias / zoomValueCurrent
            
        elif relatedX > centerX :
            diffCenterX = relatedX - centerX

            if relatedX_old != -1 :
                if relatedX_old < centerX :
                    gimbalCornerX = gimbalCornerX - corner_add_value_X  #test20220307
                elif relatedX_old > centerX :
                    gimbalCornerX = gimbalCornerX + corner_add_value_X  #test20220307

            yaw = gimbalCornerX * (diffCenterX / centerX)
            yaw = yaw * cornerbias / zoomValueCurrent

        else:
            yaw = 0 #//センター線上
        
        #  //Y軸移動
        if relatedY < centerY :
            diffCenterY = centerY - relatedY

            if relatedY_old != -1 :
                if relatedY_old < centerY :
                    gimbalCornerY = gimbalCornerY + corner_add_value_Y  #test20220307
                elif relatedY_old > centerY :
                    gimbalCornerY = gimbalCornerY - corner_add_value_Y  #test20220307
            tilt = gimbalCornerY * (diffCenterY / centerY)
            tilt = tilt * cornerbias / zoomValueCurrent
        elif relatedY > centerY :
            diffCenterY = relatedY - centerY

            if relatedY_old != -1 :
                if relatedY_old < centerY :
                    gimbalCornerY = gimbalCornerY - corner_add_value_Y  #test20220307
                elif relatedY_old > centerY :
                    gimbalCornerY = gimbalCornerY + corner_add_value_Y  #test20220307
            tilt = -1 * gimbalCornerY * (diffCenterY / centerY)
            tilt = tilt * cornerbias / zoomValueCurrent
        else:
            tilt = 0            #//センター線上

        print("yaw:{},tilt:{}".format(yaw,tilt))


        diffX = relatedX_old - relatedX
        diffY = relatedY_old - relatedY

        relatedX_old = relatedX
        relatedY_old = relatedY          


        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 1,"action_type": 2,"action_value": str(yaw) +","+ str(tilt)}
        )
        print(j)

        #addstart20220302
        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)
        #addend20220202

        return yaw, tilt #test 20220414

    def gimbalControlYawTiltLargeAbsolute(self,yaw,tilt):
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 1,"action_type": 4,"action_value": str(yaw) +","+ str(tilt)}
        )
        print(j)

        #addstart20220302
        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)
        #addend20220202


    def cameraControlZoom(self,zoomvalue):
        global zoomValueCurrent
        zoomValueCurrent = zoomvalue
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 2,"action_type": 0,"action_value": str(zoomvalue)}
        )
        print(j)

        #addstart20220302
        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)
        #addend20220202        

    def cameraControlFocus(self,focusmode,x,y):
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 2,"action_type": 1,"action_value": str(focusmode) + "," + str(x) + "," + str(y)}
        )
        print(j)

        #addstart20220302
        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)
        #addend20220202

    def cameraControlChangeLens(self,lensvalue):
        global lensValueCurrent
        lensValueCurrent = lensvalue
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 2,"action_type": 2,"action_value": str(lensvalue)}
        )
        print(j)

        #addstart20220302
        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)
        #addend20220202

    def droneControlAction(self,actionvalue):
        global actionValueCurrent
        actionValueCurrent = actionvalue
        j = json.dumps(
            {"cmd": "fp_resume", "type": "ctrl_drone", "flight_id": "dummy"}
        )
        print(j)

        #addstart20220302
        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        # utmMessage.target_type = jd["target_type"]
        # utmMessage.action_type = jd["action_type"]
        # utmMessage.action_value = jd["action_value"]

        utmMessage.flight_id = jd["flight_id"]

        self.utm_pub.publish(utmMessage)
        #addend20220202


    def generate_sampling_group(self,input_result,input_prob,height, width, mode_flag,input_objectdetection_data):
        try :

            global confidence_bias
            global group_bias

            # モデル設定
            # self.__names = names
            # self.__colors = colors
            self.__shape = 512
            self.__h_magnification = self.__shape/height
            self.__w_magnification = self.__shape/width

            print("len(input_objectdetection_data):{}".format(len(input_objectdetection_data)))
            if (len(input_objectdetection_data) > 0 ):
                objectdetection_data = input_objectdetection_data
            else :
                objectdetection_data = []

            print('=====result start=====')
            print("magnification H:{} W:{}".format(1/self.__h_magnification, 1/self.__w_magnification))
            print("input_result:{}".format(input_result))
            print("input_prob:{}".format(input_prob))
            for det in input_prob:
                if(det[0]==-1):
                    continue
                #クラス
                index = int(det[0])

                #識別率
                confidence = det[1]
                x1 = int(det[2] * self.__shape * 1/self.__w_magnification)
                y1 = int(det[3] * self.__shape * 1/self.__h_magnification)
                x2 = int(det[4] * self.__shape * 1/self.__w_magnification)
                y2 = int(det[5] * self.__shape * 1/self.__h_magnification) 

                #中心算出
                detection_centerX = int((x1 + x2)/2)
                detection_centerY = int((y1 + y2)/2)

                #面積算出
                detection_square = int((x2 - x1) * (y2 - y1))

                #縦横の長さ算出
                boundingbox_width = int(x2 - x1)
                boundingbox_height = int(y2 -y1)

                #認識率判定
                if confidence > confidence_bias :
                    print("type(input_prob):{}".format(type(input_prob)))
                    print("type(det):{}".format(type(det)))
                    print("[class:{}], confidence:{:.1f} x1:{}, y1:{}, x2:{}, y2:{},boundingbox_width:{},boundingbox_height:{}".format(CLASSES[index], confidence, x1, y1, x2, y2,boundingbox_width,boundingbox_height))

                    #グループ化
                    print("objectdetection_data:{}".format(objectdetection_data))
                    print("len(objectdetection_data):{}".format(len(objectdetection_data)))
                    if len(objectdetection_data) > 0 :
                        for classdata in objectdetection_data :
                            print("classdata:{}".format(classdata))
                            print("index:{},\n abs(detection_centerX - classdata[2]):{},\n abs(detection_centerY - classdata[3]):{}".format(index, abs(detection_centerX - classdata[2]),abs(detection_centerY - classdata[3])))
                            #クラス比較と中心でグループ化
                            if (classdata[0] == index and  abs(detection_centerX - classdata[2]) <= group_bias and  abs(detection_centerY - classdata[3]) <= group_bias):
                                print("==========更新===========")
                                #グループとみなせる場合は更新
                                classdata[1] = (classdata[1] + confidence)//2           #認識率
                                classdata[2] = (classdata[2] + detection_centerX)//2    #中心の横位置
                                classdata[3] = (classdata[3] + detection_centerY)//2    #中心の縦位置
                                classdata[4] = (classdata[4] + boundingbox_width)//2    #横の長さ
                                classdata[5] = (classdata[5] + boundingbox_height)//2   #縦の長さ
                                classdata[6] = (classdata[6] + detection_square)//2     #面積
                                classdata[7] = classdata[7] + 1                         #グループ内の数
                                break
                        else:
                            #グループとみなせるものがない場合は追加
                            print("==========追加===========")
                            objectdetection_data_new = np.array([[index,confidence,detection_centerX,detection_centerY,boundingbox_width,boundingbox_height,detection_square,1]])
                            print("objectdetection_data_new: np.array[{}]".format(objectdetection_data_new))
                            objectdetection_data = np.append(objectdetection_data,objectdetection_data_new,axis=0)
                            print("apended objectdetection_data:{}".format(objectdetection_data))
                            print("apended len(objectdetection_data):{}".format(len(objectdetection_data)))                        
                    else :
                        print("==========新規===========")
                        objectdetection_data = np.array([[index,confidence,detection_centerX,detection_centerY,boundingbox_width,boundingbox_height,detection_square,1]])
                        print("objectdetection_data: np.array[{}]".format(objectdetection_data))
            print('=====result end=====')
            return(objectdetection_data)
        except Exception as e:
            print("generate_sampling_group exception:{}".format(e))


    def imshow_while_waiting(self, wait_time, cap, frameName):
        counter_zoom = 0
        wait_zoom_tmp = wait_time
        _cap = cap
        time_zoom = dt.now() + datetime.timedelta(seconds=wait_zoom_tmp)
        print("start:{}".format(dt.now()))
        print("time_zooom:{}".format(time_zoom))
        while dt.now() < time_zoom:
            if counter_zoom % (INTERVAL/2) == 0:
                try:
                    # _, frame = cap.read()
                    _, frame = _cap.read()
                    # cv2.imshow('frame', frame)
                    # cv2.imshow(frameName, frame)

                    label = "Remaining waiting time:" + str(time_zoom - dt.now())                                                   
                    frame = cv2.putText(frame,label,(0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),1, cv2.LINE_AA)
                    cv2.imshow(frameName, frame)

                    # qキーで終了
                    if cv2.waitKey(1) & 0xFF == ord('a'):
                        break                                                            
                except:
                    print("error VideoCapture")
                    pass
            counter_zoom += 1
        print("end:{}".format(dt.now()))


    def objectdetection0(self):
        #faker
        global camera_width     #カメラ幅
        global camera_height    #ガメラ高さ
        global maxCornerX       #初期ジンバル最大角度X
        global maxCornerY       #初期ジンバル最大角度Y
        global gimbalCornerX    #ジンバル角度X
        global gimbalCornerY    #ジンバル角度Y
        global relatedX_old
        global relatedY_old
        global cornerbias
        global targetSquareWidth
        global targetSquareHeight

        global centerX
        global centerY

        global zoomValueCurrent
        global ZoomValues

        global lensValueCurrent

        global actionvalue


        global flg_objectdetection_test2
        global objectdetection_mode
        global OBJECTDETECTION_MODE_DEFAULT

        global sampling_count
        global group_bias
        global zoom_bias

        global target_model_class_index
        global target_model_class_indexes #add 20220310

        global gimbal_pitch
        global gimbal_roll
        global gimbal_yaw

        global gimbal_yaw_saved
        global gimbal_pitch_saved

        global gimbal_mode
        global gimbal_mode_old

        global currentCameraLens
        global CAMERALENS_WIDE
        global CAMERALENS_ZOOM
        global CAMERALENS_DUMMY_PAUSE

        global OSDK_CAMERA_SOURCE_H20T_WIDE
        global OSDK_CAMERA_SOURCE_H20T_ZOOM
        global OSDK_CAMERA_SOURCE_H20T_IR

        global objectdetection_tower_mode                   #add 20211126
        global OBJECTDETECTION_TOWER_MODE_TAICHO_220KV      #add 20211126
        global OBJECTDETECTION_TOWER_MODE_KENSUI_500KV      #add 20211126

        global current_aircraft_waypoint    #test20211128
        # current_aircraft_waypoint = -1  #test20211128

        global flg_objectdetection_test2_prev #test20220401
        flg_objectdetection_test2_prev = False  #test20220401

        global confidence_bias  #test20220405
        global confidence_bias_taicho #test20220407
        global confidence_bias_kensui #test20220407
        global confidence_bias_taicho_wide_to_zoom 

        global flg_gimbal_variable_waiting_time

        global objectdetection_target_model

        global input_mode
        global INPUT_MODE_GSTREAMER_UDP
        global INPUT_MODE_GSTREAMER_UDP_STR
        global INPUT_MODE_GSTREAMER_URI
        global INPUT_MODE_GSTREAMER_URI_STR
        global INPUT_MODE_FILE
        global INPUT_MODE_FILE_FILE_PATH

        while True :
            try :
                print("start cap")
                if input_mode == INPUT_MODE_FILE:
                    print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_FILE,INPUT_MODE_FILE_FILE_PATH))
                    cap = cv2.VideoCapture(INPUT_MODE_FILE_FILE_PATH)                               #ファイルから
                elif input_mode == INPUT_MODE_GSTREAMER_URI:
                    print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_GSTREAMER_UDP,INPUT_MODE_GSTREAMER_URI_STR))
                    cap = cv2.VideoCapture(INPUT_MODE_GSTREAMER_URI_STR,cv2.CAP_GSTREAMER)      #gstreamer_uriから
                else :
                    print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_GSTREAMER_UDP,INPUT_MODE_GSTREAMER_UDP_STR))
                    cap = cv2.VideoCapture(INPUT_MODE_GSTREAMER_UDP_STR,cv2.CAP_GSTREAMER)      #gstreamer_udpから
                print("end cap")
                if not cap.isOpened():
                    sys.exit()
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = cap.get(cv2.CAP_PROP_FPS)
                print("fps:{}　width:{}　height:{}".format(fps, width, height))
                break
            except :
                print("error VideoCapture")
                pass

        print("cap test")
        #モデル読み込み
        model = Model(CLASSES, COLORS, height, width, MODEL_PATH)


        #描画設定
        camera_width = int(width)     #カメラ幅
        camera_height = int(height)    #カメラ高さ
        print("camera_width:{},camera_height:{}".format(camera_width,camera_height))
        relatedX_old = -1
        relatedY_old = -1

        centerX = camera_width // 2   #中心(横)
        centerY = camera_height // 2  #中心(縦)
        gimbalCornerX = maxCornerX
        gimbalCornerY = maxCornerY
        cornerbias = 2

        #サンプリング設定
        generate_sampling_group_data = []
        
        #vmwareフルスクリーン用
        ##cv2.namedWindow('frame', cv2.WND_PROP_AUTOSIZE)
        ##cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN , cv2.WINDOW_NORMAL)

        #vmware最大画面用
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)


        cv2.setMouseCallback('frame',self.onMouseClick)


        counter = 0
        # while True:
        while not rospy.is_shutdown():

            while True:
        #         cap.set(cv2.CAP_PROP_POS_FRAMES, 5)
                # カメラ画像取得
                try :
                    if not cap.isOpened():
                        # cap = cv2.VideoCapture(GST_STR,cv2.CAP_GSTREAMER) #test20210823
                        if input_mode == INPUT_MODE_FILE:
                            print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_FILE,INPUT_MODE_FILE_FILE_PATH))
                            cap = cv2.VideoCapture(INPUT_MODE_FILE_FILE_PATH)                               #ファイルから
                        elif input_mode == INPUT_MODE_GSTREAMER_URI:
                            print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_GSTREAMER_UDP,INPUT_MODE_GSTREAMER_URI_STR))
                            cap = cv2.VideoCapture(INPUT_MODE_GSTREAMER_URI_STR,cv2.CAP_GSTREAMER)      #gstreamer_uriから
                        else :
                            print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_GSTREAMER_UDP,INPUT_MODE_GSTREAMER_UDP_STR))
                            cap = cv2.VideoCapture(INPUT_MODE_GSTREAMER_UDP_STR,cv2.CAP_GSTREAMER)      #gstreamer_udpから
                        continue
                    _, frame = cap.read()
                    break
                # except Exception as e :
                #     print("error cap.read:{}".format(e))
                #     continue
                except :
                    print("error cap.read")
                    while True :
                        try :
                            print("start cap")
                            # cap = cv2.VideoCapture(file_path)
                            # cap = cv2.VideoCapture(GST_STR,cv2.CAP_GSTREAMER)
                            if input_mode == INPUT_MODE_FILE:
                                print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_FILE,INPUT_MODE_FILE_FILE_PATH))
                                cap = cv2.VideoCapture(INPUT_MODE_FILE_FILE_PATH)                               #ファイルから
                            elif input_mode == INPUT_MODE_GSTREAMER_URI:
                                print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_GSTREAMER_UDP,INPUT_MODE_GSTREAMER_URI_STR))
                                cap = cv2.VideoCapture(INPUT_MODE_GSTREAMER_URI_STR,cv2.CAP_GSTREAMER)      #gstreamer_uriから
                            else :
                                print("INPUT_MODE:{}, VALUE:{}".format(INPUT_MODE_GSTREAMER_UDP,INPUT_MODE_GSTREAMER_UDP_STR))
                                cap = cv2.VideoCapture(INPUT_MODE_GSTREAMER_UDP_STR,cv2.CAP_GSTREAMER)      #gstreamer_udpから
                            print("end cap")
                            if not cap.isOpened():
                                sys.exit()
                            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                        #     width = 512
                        #     height = 512
                            fps = cap.get(cv2.CAP_PROP_FPS)
                            print("fps:{}　width:{}　height:{}".format(fps, width, height))
                            break
                        except :
                            print("error VideoCapture")
                            pass
                    continue

            if(frame is None):
                try :
                    # cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    # Gstreamerで配信しているのが止まった時にエラーになるためここの処理はコメントアウトする
                    pass
                except Exception as e :
                    print("error cap:{}".format(e))
                continue

            if (flg_objectdetection_test2 == True ) :
                
                #addstart 20220401
                if flg_objectdetection_test2_prev == False :
                    #物体検知開始
                    print("▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼物体検知開始({},INTERVAL:{},sampling_count:{})▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼".format(dt.now(),INTERVAL,sampling_count))

                    if objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM:
                            print("物体開始時ズーム初期化(2倍)")
                            self.cameraControlZoom(ZoomValuex2)
                            print("物体開始時レンズ切替(ワイド)")
                            self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_WIDE)                        
                            currentCameraLens = CAMERALENS_WIDE

                    flg_objectdetection_test2_prev = True
                    continue 
                    
                #addend 20220401

                if ( counter % INTERVAL == 0):
                    #addstart 20211126 
                    # モード判定
                    if objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_TAICHO_220KV:
                        if currentCameraLens == CAMERALENS_WIDE:
                            print("物体開始時ズーム初期化(2倍)")
                            self.cameraControlZoom(ZoomValuex2)
                            currentCameraLens = CAMERALENS_ZOOM
                            #wide -> zoomに切り替え
                            print("物体開始時レンズ切替(ワイド→ズーム)")
                            self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_ZOOM)
                            print("########################wide->zoom wait {}s#######################".format(wait_gimbal))    #test20220304
                            # #カメラ・ジンバル制御待機(画面描画)
                            self.imshow_while_waiting(wait_time=(wait_camera_change),cap=cap,frameName="frame")   #add 20220414
                            generate_sampling_group_data = []   #サンプリングデータ初期化   #20220304
                            self.gimbalcornerResetOnlyCorner()
                            continue
                    #addend 20211126
                    #addstart 20220310
                    elif objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_KENSUI_500KV:
                        if currentCameraLens == CAMERALENS_WIDE:
                            print("物体開始時ズーム初期化(2倍)")
                            self.cameraControlZoom(ZoomValuex2)
                            currentCameraLens = CAMERALENS_ZOOM
                            #wide -> zoomに切り替え
                            print("物体開始時レンズ切替(ワイド→ズーム)")
                            self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_ZOOM)
                            print("########################wide->zoom wait {}s#######################".format(wait_gimbal))    #test20220304
                            #カメラ・ジンバル制御待機(画面描画)
                            self.imshow_while_waiting(wait_time=(wait_camera_change),cap=cap,frameName="frame")   #add 20220414
                            generate_sampling_group_data = []   #サンプリングデータ初期化 
                            self.gimbalcornerResetOnlyCorner()
                            continue
                    #addend 20220310
                    #addstart 20220330
                    elif objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM:
                        if currentCameraLens == CAMERALENS_ZOOM:
                            #ズームのとき
                            targetSquareWidth = 140
                            targetSquareHeight = 120
                            confidence_bias_taicho = 0.3        #test20220407
                            confidence_bias = confidence_bias_taicho  #test20220407

                        else :
                            #ワイドのとき
                            targetSquareWidth = 240
                            targetSquareHeight = 240

                            targetSquareWidth = 360
                            targetSquareHeight = 300

                            targetSquareWidth = 420
                            targetSquareHeight = 360

                            confidence_bias_taicho = confidence_bias_taicho_wide_to_zoom        #test20220407
                            confidence_bias = confidence_bias_taicho  #test20220407


                    #addend 20220330


                    # 保存用
                    save_image = frame.copy()
                    (result, disp_frame, prob) = model.inference(frame)
                    print("{}".format(result))

                    #各種描画
                    #十字
                    print("line camera_width:{},camera_height:{}".format(camera_width,camera_height))
                    # print("currnet gimbal mode:{}".format(gimbal_mode)) //test20211123
                    cv2.line(disp_frame, (0, centerY), (camera_width, centerY), (175, 175, 175), thickness=1)
                    cv2.line(disp_frame, (centerX, 0), (centerX, camera_height), (175, 175, 175), thickness=1)
                    #ターゲット
                    cv2.rectangle(disp_frame,(centerX - targetSquareWidth // 2, centerY - targetSquareHeight // 2),(centerX + targetSquareWidth // 2,centerY + targetSquareHeight // 2),(175,175,175),1)

                    #モード表示
                    label = objectdetection_mode
                    disp_frame = cv2.putText(disp_frame,label,(0, camera_height-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
                    disp_frame = cv2.putText(disp_frame,label,(0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)    #test20220404

                    #レンズモード表示
                    label = currentCameraLens
                    print("label:{}".format(label)) #test20220330
                    if label == CAMERALENS_DUMMY_PAUSE : 
                        disp_frame = cv2.putText(disp_frame,label,(0, camera_height-10-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
                        disp_frame = cv2.putText(disp_frame,label,(0, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)    #test20220404
                    else :
                        print("test20220330")
                        disp_frame = cv2.putText(disp_frame,label,(0, camera_height-10-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
                        disp_frame = cv2.putText(disp_frame,label,(0, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)    #test20220404

                    # #test20211128
                    # #waypoint表示 
                    print("current_aircraft_waypoint:{}".format(current_aircraft_waypoint)) #test20220330
                    label = "WP:" + str(current_aircraft_waypoint)
                    disp_frame = cv2.putText(disp_frame,label,(0, camera_height-10-20-35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
                    disp_frame = cv2.putText(disp_frame,label,(0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)    #test20220404
                    # #test20211128

                    #test20220310
                    #検知時のパラメータ表示 
                    label = "Model:" + objectdetection_target_model + ", Mode:" + str(objectdetection_tower_mode) + ", conf:" + str(confidence_bias) + ", conf_show:" + str(confidence_bias_show) + ", camera:" + str(currentCameraLens) + ", zoom:" + str(zoomValueCurrent)
                    disp_frame = cv2.putText(disp_frame,label,(0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),1, cv2.LINE_AA)
                    #test20220310

                    #左上の枠
                    cv2.rectangle(disp_frame,(0, 0),(60,60),(175,175,175),1)

                    #左下の枠
                    cv2.rectangle(disp_frame,(0, camera_height - 60),(120,camera_height),(175,175,175),1)

                    #右下の枠
                    cv2.rectangle(disp_frame,(camera_width - 120, camera_height - 60),(camera_width,camera_height),(175,175,175),1)

                    # 画像表示
                    cv2.imshow('frame', disp_frame)

                    #レンズがポーズのときは次に行く
                    if currentCameraLens == CAMERALENS_DUMMY_PAUSE :
                        #サンプリング初期化
                        generate_sampling_group_data = []   #サンプリングデータ初期化

                        counter += 1
                        continue

                    #物体検知サンプリング
                    generate_sampling_group_data = self.generate_sampling_group(result,prob,height, width, "add", generate_sampling_group_data)
                    print("len(generate_sampling_group_data):{}".format(len(generate_sampling_group_data)))
                    if len(generate_sampling_group_data) > 0 :
                        print("test")
                        flg_control_gimbal = False
                        max_detection_square = 0
                        min_detection_distance = width    #保留
                        max_confidence_bias = 0
                        target_index_max_detection_square = -1
                        target_index_min_detection_distance = -1
                        target_index_max_confidence_bias = 0
                        for i, predet in enumerate(generate_sampling_group_data) :
                            #サンプリング判定用集計
                            if(int(predet[0]) in target_model_class_indexes):   #test20220310
                                if (int(predet[7]) >= sampling_count):
                                    flg_control_gimbal = True
                                else:
                                    pass
                            else:
                                continue

                            if (max_detection_square < predet[6]):
                                #最大面積算出
                                max_detection_square = predet[6]  #最大面積
                                target_index_max_detection_square = i
                                print("max_detection_square:{}".format(max_detection_square))
                                # print("max_target_class:{}".format(target_class))

                            tmp_detection_distance = numpy.linalg.norm(numpy.array([predet[2], predet[3]]) -  numpy.array([centerX, centerY]))
                            print("i:{},numpy.array([predet[2], predet[3]]):{},numpy.array([centerX, centerY]):{},tmp_detection_distance:{}".format(i,numpy.array([predet[2], predet[3]]),numpy.array([centerX, centerY]),tmp_detection_distance))
                            if (min_detection_distance > numpy.linalg.norm(numpy.array([predet[2], predet[3]]) -  numpy.array([centerX, centerY]))):
                                #中心からの最小距離
                                min_detection_distance = numpy.linalg.norm(numpy.array([predet[2], predet[3]]) -  numpy.array([centerX, centerY]))  #最小距離
                                target_index_min_detection_distance = i
                                print("min_detection_distance:{},numpy.array([predet[2], predet[3]]):{}".format(min_detection_distance,numpy.array([predet[2], predet[3]])))
                                # print("max_target_class:{}".format(target_class))

                            if (max_confidence_bias < predet[1]):
                                #最大認識率
                                max_confidence_bias = predet[1]  #最大認識率
                                target_index_max_confidence_bias = i
                                print("max_confidence_bias:{}".format(max_confidence_bias))
                                # print("max_target_class:{}".format(target_class))

                        #集計結果
                        print("i:{},max_detection_square:{}".format(target_index_max_detection_square,max_detection_square))
                        print("i:{},min_detection_distance:{}".format(target_index_min_detection_distance,min_detection_distance))
                        print("i:{},max_confidence_bias:{}".format(target_index_max_confidence_bias,max_confidence_bias))


                        for i, det in enumerate(generate_sampling_group_data) :
                            print("test2")
                            print(det)
                            print("det[0]:{}".format(det[0]))
                            print("det[7]:{}".format(det[7]))
                            target_detected_index = ""
                            if objectdetection_mode == "center" :           #中心優先
                                print("中心優先")
                                target_detected_index = target_index_min_detection_distance
                            elif  objectdetection_mode == "area" :          #面積優先
                                print("面積優先")
                                target_detected_index = target_index_max_detection_square
                            elif  objectdetection_mode == "confidence" :    #認識率優先
                                print("認識率優先")
                                target_detected_index = target_index_max_confidence_bias
                            else :                                          #面積優先
                                print("面積優先")
                                target_detected_index = target_index_max_detection_square
                            print("mode:{},selected_index:{}".format(objectdetection_mode,target_detected_index))
                
                            #物体検知
                            if (int(det[0]) in target_model_class_indexes and i == target_detected_index and flg_control_gimbal == True):   #test20220310
                                print("det[2]:{}".format(det[2]))
                                print("centerX - targetSquareWidth // 2:{}".format(centerX - targetSquareWidth // 2))
                                print("centerX + targetSquareWidth // 2:{}".format(centerX + targetSquareWidth // 2))
                                print("det[3]:{}".format(det[3]))
                                print("centerY - targetSquareWidth // 2:{}".format(centerY - targetSquareHeight // 2))
                                print("centerY + targetSquareWidth // 2:{}".format(centerY + targetSquareHeight // 2))
                                #検知後の値を判断
                                tmpzoom = zoomValueCurrent // 2     #テストズーム時に中心判定緩和

                                #changestart 20220401
                                # #中心厳格判定(バウンディングボックスが中央の視覚より小さい場合は中心判定を厳しくする)
                                if det[4] <  targetSquareWidth and ((currentCameraLens == CAMERALENS_ZOOM and objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM) or (flg_target_strict_judgement == True)) :
                                    tmp_targetSquareWidth = targetSquareWidth // 2
                                else:
                                    tmp_targetSquareWidth = targetSquareWidth

                                if det[5] <  targetSquareHeight and ((currentCameraLens == CAMERALENS_ZOOM and objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM) or (flg_target_strict_judgement == True)) :
                                    tmp_targetSquareHeight = targetSquareHeight // 2
                                else:
                                    tmp_targetSquareHeight = targetSquareHeight

                                #中心判定
                                if ((det[2] > centerX - (tmp_targetSquareWidth // 2 * tmpzoom) and det[2] < centerX + (tmp_targetSquareWidth //2 * tmpzoom)  \
                                    and det[3] > centerY - (tmp_targetSquareHeight // 2 * tmpzoom) and det[3] < centerY + (tmp_targetSquareHeight // 2) * tmpzoom) or \
                                    (objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_KENSUI_500KV and gimbal_pitch > 28 and det[2] > centerX - (tmp_targetSquareWidth // 2 * tmpzoom) and det[2] < centerX + (tmp_targetSquareWidth //2 * tmpzoom)) \
                                    ): #test20211210 change #20220310 add

                                #changeend 20220401
                                    print("中心！")
                                    # if ( currentCameraLens == CAMERALENS_WIDE):
                                    if ( currentCameraLens == CAMERALENS_WIDE and objectdetection_tower_mode != OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM):

                                        #サンプリング初期化
                                        generate_sampling_group_data = []   #サンプリングデータ初期化

                                        #パラメータ初期化
                                        self.gimbalcornerReset()
                                        #ズーム初期値
                                        self.cameraControlZoom(ZoomValuex2)
                                        # #レンズ切り替えまで他の処理はさせない
                                        # currentCameraLens = CAMERALENS_DUMMY_PAUSE
                                        #レンズ切り替えまで他の処理はさせない
                                        currentCameraLens = CAMERALENS_ZOOM
                                        #wide -> zoomに切り替え
                                        self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_ZOOM)
                                        print("########################wide->zoom wait {}s#######################".format(wait_gimbal))    #test20220304
                                        #カメラ・ジンバル制御待機(画面描画)
                                        self.imshow_while_waiting(wait_time=(wait_camera_change),cap=cap,frameName="frame")   #add 20220414

                                        counter += 1
                                        continue

                                        
                                    #ズーム後の高さおよび幅を確認
                                    if (det[4] >= camera_width * zoom_bias or det[5] >= camera_height * zoom_bias):
                                        #検知終了

                                        #パラメータ初期化
                                        self.gimbalcornerReset()
                                        #ズーム初期値
                                        self.cameraControlZoom(ZoomValuex2)
                                        #ジンバル復元
                                        print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gimbal_yaw_saved, gimbal_pitch_saved))
                                        self.gimbalControlYawTiltLargeAbsolute(gimbal_yaw_saved, gimbal_pitch_saved)
                                        #zoom -> wideに切り替え
                                        self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_WIDE)
                                        # rospy.sleep(7)
                                        #カメラ・ジンバル制御待機(画面描画)
                                        self.imshow_while_waiting(wait_time=(7),cap=cap,frameName="frame")   #add 20220414

                                        generate_sampling_group_data = []   #サンプリングデータ初期化
                                        gimbal_mode_old = 2 #test 要確認 20211126
                                        # print("gimbal_mode:{},gimbal_mode_old:{}".format(gimbal_mode,gimbal_mode_old)) //test20211123
                                        flg_objectdetection_test2 = False
                                        print("検知終了！！({})".format(objectdetection_mode))

                                        #フライトプラン再開
                                        self.droneControlAction("fp_resume")
                                        

                                    else :
                                        cv2.circle(disp_frame, (int(det[2]), int(det[3])),10, (0, 255, 255), thickness=-1) #add 20211014
                                        cv2.imshow('frame', disp_frame)                                                     #add 20211014
                                        tdatetime = dt.now()
                                        date_str = tdatetime.strftime('%Y%m%d_%H%M%S')
                                        filename = "{}/{}.jpg".format(OUTPUT_PATH, date_str)
                                        cv2.imwrite(filename, frame)
                                        print("Saved. {}".format(filename))

                                        #addstart 20211126
                                        #物体検知のモード判定
                                        if objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_TAICHO_220KV:
                                            self.cameraControlZoom(ZoomValuex12) #12倍
                                            self.cameraControlFocus(2,0.5,0.5)  #AFC、中心
                                            generate_sampling_group_data = []   #サンプリングデータ初期化
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(wait_zoom + 2),cap=cap,frameName="frame")   #add 20220414

                                            #add 20220117
                                            #パラメータ初期化
                                            self.gimbalcornerReset()
                                            # currentCameraLens = CAMERALENS_WIDE
                                            #ズーム初期値
                                            self.cameraControlZoom(ZoomValuex2)
                                            #ジンバル復元
                                            print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gimbal_yaw_saved, gimbal_pitch_saved))
                                            self.gimbalControlYawTiltLargeAbsolute(gimbal_yaw_saved, gimbal_pitch_saved)
                                            #zoom -> wideに切り替え
                                            self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_WIDE)
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(wait_gimbal),cap=cap,frameName="frame")   #add 20220414

                                            generate_sampling_group_data = []   #サンプリングデータ初期化
                                            # print("gimbal_mode:{},gimbal_mode_old:{}".format(gimbal_mode,gimbal_mode_old)) //test20211123
                                            flg_objectdetection_test2 = False
                                            print("220kv耐張検知終了！！({})".format(objectdetection_mode))

                                            #フライトプラン再開
                                            self.droneControlAction("fp_resume")
                                        #addstart 20220310
                                        elif objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_KENSUI_500KV:
                                            self.cameraControlZoom(ZoomValuex12) #12倍
                                            self.cameraControlFocus(2,0.5,0.5)  #AFC、中心
                                            generate_sampling_group_data = []   #サンプリングデータ初期化
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(wait_zoom + 2),cap=cap,frameName="frame")   #add 20220414

                                            #add 20220117
                                            #パラメータ初期化
                                            self.gimbalcornerReset()
                                            # currentCameraLens = CAMERALENS_WIDE
                                            #ズーム初期値
                                            self.cameraControlZoom(ZoomValuex2)
                                            #ジンバル復元
                                            print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gimbal_yaw_saved, gimbal_pitch_saved))
                                            self.gimbalControlYawTiltLargeAbsolute(gimbal_yaw_saved, gimbal_pitch_saved)
                                            #zoom -> wideに切り替え
                                            self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_WIDE)
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(wait_gimbal),cap=cap,frameName="frame")   #add 20220414

                                            generate_sampling_group_data = []   #サンプリングデータ初期化
                                            flg_objectdetection_test2 = False
                                            print("500kv懸垂検知終了！！({})".format(objectdetection_mode))

                                            #フライトプラン再開
                                            self.droneControlAction("fp_resume")

                                        #addend 20220310
                                        #addstart 20220330
                                        elif objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE_TAICHO_WIDE_TO_ZOOM:
                                            if currentCameraLens == CAMERALENS_WIDE:
                                                #ジンバル制御
                                                print("ジンバル制御！({})".format(objectdetection_mode))
                                                cv2.circle(disp_frame, (int(det[2]), int(det[3])),10, (0, 255, 255), thickness=-1) #add 20211014
                                                cv2.imshow('frame', disp_frame)                                                     #add 20211014
                                                print("cornerbias:{},gimbalCornerX:{},gimbalCornerY:{}".format(cornerbias,gimbalCornerX,gimbalCornerY))  #test20220304
                                                tdatetime = dt.now()
                                                date_str = tdatetime.strftime('%Y%m%d_%H%M%S')
                                                filename = "{}/{}.jpg".format(OUTPUT_PATH, date_str)
                                                cv2.imwrite(filename, frame)
                                                print("Saved. {}".format(filename))

                                                self.gimbalControlYawTiltLarge(det[2],det[3])
                                                # generate_sampling_group_data = []   #サンプリングデータ初期化

                                                #wide -> zoomに切り替え
                                                currentCameraLens = CAMERALENS_ZOOM
                                                print("ズームモード時ズーム初期化(2倍)")
                                                self.cameraControlZoom(ZoomValuex2)
                                                # #カメラ・ジンバル制御待機(画面描画)
                                                self.imshow_while_waiting(wait_time=(wait_zoom),cap=cap,frameName="frame")   #add 20220401
                                                self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_ZOOM)
                                                # rospy.sleep(wait_gimbal)
                                                
                                                generate_sampling_group_data = []   #サンプリングデータ初期化

                                                gimbalCornerX = maxCornerX
                                                gimbalCornerY = maxCornerY

                                                continue    #test20220415

                                            else:
                                                self.cameraControlZoom(ZoomValuex12) #12倍？ #107
                                                self.cameraControlFocus(2,0.5,0.5)  #AFC、中心
                                                generate_sampling_group_data = []   #サンプリングデータ初期化

                                                #カメラ・ジンバル制御待機(画面描画)
                                                self.imshow_while_waiting(wait_time=(wait_gimbal + 2),cap=cap,frameName="frame")   #add 20220331

                                                #パラメータ初期化
                                                self.gimbalcornerReset()
                                                #ズーム初期値
                                                self.cameraControlZoom(ZoomValuex2)
                                                #ジンバル復元
                                                print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gimbal_yaw_saved, gimbal_pitch_saved))
                                                self.gimbalControlYawTiltLargeAbsolute(gimbal_yaw_saved, gimbal_pitch_saved)
                                                #zoom -> wideに切り替え
                                                currentCameraLens = CAMERALENS_WIDE
                                                self.cameraControlChangeLens(OSDK_CAMERA_SOURCE_H20T_WIDE)

                                                #カメラ・ジンバル制御待機(画面描画)
                                                self.imshow_while_waiting(wait_time=wait_gimbal,cap=cap,frameName="frame")   #add 20220331

                                                generate_sampling_group_data = []   #サンプリングデータ初期化
                                                flg_objectdetection_test2 = False
                                                flg_objectdetection_test2_prev = False  #test20220404
                                                print("耐張(wide)検知終了！！({})".format(objectdetection_mode))
                                                #物体検知終了
                                                print("▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲物体検知終了({},INTERVAL:{},sampling_count:{})▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲".format(dt.now(),INTERVAL,sampling_count))

                                                #フライトプラン再開
                                                self.droneControlAction("fp_resume")
                                        #addstart 20220330
                                        else: 
                                        #addend 202111126
                                            #物体検知モード指定以外の場合                                                          
                                            #中心に近い場合はジンバル制御を停止しズームする
                                            if (zoomValueCurrent < ZoomValues[0]):
                                                self.cameraControlZoom(ZoomValues[0])
                                            elif (zoomValueCurrent < ZoomValues[1]):
                                                self.cameraControlZoom(ZoomValues[1])
                                            elif (zoomValueCurrent < ZoomValues[2]):
                                                self.cameraControlZoom(ZoomValues[2])
                                            elif (zoomValueCurrent < ZoomValues[3]):
                                                self.cameraControlZoom(ZoomValues[3])
                                            elif (zoomValueCurrent < ZoomValues[4]):
                                                self.cameraControlZoom(ZoomValues[4])
                                            elif (zoomValueCurrent < ZoomValues[5]):
                                                self.cameraControlZoom(ZoomValues[5])
                                            elif (zoomValueCurrent < ZoomValues[6]):
                                                self.cameraControlZoom(ZoomValues[6])
                                            elif (zoomValueCurrent < ZoomValues[7]):
                                                self.cameraControlZoom(ZoomValues[7])
                                            elif (zoomValueCurrent < ZoomValues[8]):    #add 20220330
                                                self.cameraControlZoom(ZoomValues[8])   #add 20220330
                                            else :
                                                self.cameraControlZoom(ZoomValues[0])
                                            #ズーム後フォーカス処理
                                            self.cameraControlFocus(2,0.5,0.5)  #AFC、中心
                                            generate_sampling_group_data = []   #サンプリングデータ初期化
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(wait_zoom),cap=cap,frameName="frame")   #add 20220414

                    
                                else :
                                    #ジンバル制御
                                    print("ジンバル制御！({})".format(objectdetection_mode))
                                    cv2.circle(disp_frame, (int(det[2]), int(det[3])),10, (0, 255, 255), thickness=-1) #add 20211014
                                    cv2.imshow('frame', disp_frame)                                                     #add 20211014
                                    print("cornerbias:{},gimbalCornerX:{},gimbalCornerY:{}".format(cornerbias,gimbalCornerX,gimbalCornerY))  #test20220304

                                    tdatetime = dt.now()
                                    date_str = tdatetime.strftime('%Y%m%d_%H%M%S')
                                    filename = "{}/{}.jpg".format(OUTPUT_PATH, date_str)
                                    cv2.imwrite(filename, frame)
                                    print("Saved. {}".format(filename))

                                    yaw,tilt = self.gimbalControlYawTiltLarge(det[2],det[3])
                                    generate_sampling_group_data = []   #サンプリングデータ初期化
                                    tmp_wait_gimbal = wait_gimbal
                                    if flg_gimbal_variable_waiting_time == True:
                                        #ジンバル制御時の待ち時間自動調整
                                        tmp_wait_gimbal = 4
                                        if abs(yaw) >= 60 or abs(tilt) >= abs(60):
                                            tmp_wait_gimbal = 4
                                        elif abs(yaw) > abs(tilt) and 60 > abs(yaw):
                                            tmp_wait_gimbal = 2 *  abs(yaw) / 60 + 2
                                        elif abs(tilt) > abs(yaw) and 60 > abs(tilt):
                                            tmp_wait_gimbal = 2 *  abs(tilt) / 60 + 2
                                    else :
                                        #ジンバル制御時の待ち時間固定
                                        tmp_wait_gimbal = wait_gimbal
                                    print("ジンバル制御waittime:{}".format(tmp_wait_gimbal))
                                    rospy.sleep(tmp_wait_gimbal)
                                    

            else :
                #サンプリング初期化
                generate_sampling_group_data = []   #サンプリングデータ初期化

                disp_frame = frame
                #各種描画
                #十字
                cv2.line(disp_frame, (0, centerY), (camera_width, centerY), (175, 175, 175), thickness=1)
                cv2.line(disp_frame, (centerX, 0), (centerX, camera_height), (175, 175, 175), thickness=1)
                #ターゲット
                cv2.rectangle(disp_frame,(centerX - targetSquareWidth // 2, centerY - targetSquareHeight // 2),(centerX + targetSquareWidth // 2,centerY + targetSquareHeight // 2),(175,175,175),1)

                #左上の枠
                cv2.rectangle(disp_frame,(0, 0),(60,60),(175,175,175),1)

                #addstart 20220330
                rectangl_width = 60
                rectangl_height = 60
                rectangl_height = 40    #test20220401

                #カメラ切替
                start_y = 80
                currnet_start_y = start_y
                cv2.rectangle(disp_frame,(0, currnet_start_y),(rectangl_width,currnet_start_y + rectangl_height),(175,175,175),1)
                label = "<>"
                currnet_start_y += rectangl_height
                disp_frame = cv2.putText(disp_frame,label,(0, currnet_start_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, cv2.LINE_AA)

                #ズーム用枠
                start_y = 120
                currnet_start_y = start_y
                for test in ZoomValues:
                    cv2.rectangle(disp_frame,(0, currnet_start_y),(rectangl_width,currnet_start_y + rectangl_height),(175,175,175),1)
                    label = "x" + str(test)
                    currnet_start_y += rectangl_height
                    disp_frame = cv2.putText(disp_frame,label,(0, currnet_start_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
                #addend 20220330

                #左下の枠
                cv2.rectangle(disp_frame,(0, camera_height - 60),(120,camera_height),(175,175,175),1)

                #右下の枠
                cv2.rectangle(disp_frame,(camera_width - 120, camera_height - 60),(camera_width,camera_height),(175,175,175),1)

                # #test20211128
                # #waypoint表示 
                label = "WP:" + str(current_aircraft_waypoint)
                disp_frame = cv2.putText(disp_frame,label,(0, camera_height-10-20-35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
                disp_frame = cv2.putText(disp_frame,label,(0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)    #test20220404
                # #test20211128


                # 画像表示
                cv2.imshow('frame', disp_frame)

                #フラグ保持
                flg_objectdetection_test2_prev = False  #test20220401


            # qキーで終了
            #if cv2.waitKey(1) & 0xFF == 2490368:
            #    break
            wk = cv2.waitKey(1) 
            if wk==13:
                break
            elif wk==87:
                os.system("echo 'terra1234'|sudo -Ss;sudo /sbin/reboot")
            elif wk==97: #a
                 cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
                 cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN) #�~C~U�~C��~B��~B��~C��~C��~C��~A| �~A�VNC�~A��~O~O�~T��~A~L�~[��~V��~A~U�~B~L�~A��~A~D
            elif wk==119: #w
                cv2.namedWindow('frame', cv2.WND_PROP_AUTOSIZE)
                cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN , cv2.WINDOW_NORMAL)
            elif wk==-1:
                pass
            else:
                print("wk:{}".format(wk))

            counter += 1

        cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        cv2.waitKey(1)
        cv2.waitKey(1)
        cv2.waitKey(1)
        self.watchdog_sub.unregister()
        self.utm_pub.unregister()
        self.utm_sub.unregister()
        self.telemetry_sub.unregister()
        rospy.signal_shutdown('objectdetection')
        subprocess.run('echo "terra1234" | sudo -Ss;/home/terra/kill_all_rosnode.sh', shell=True)
       

if __name__ == '__main__':
    rospy.init_node('objectdetection', anonymous=True)
    try:
        if TerraUTMSDKMainNode().start():
            print("start spin()")
            rospy.spin()
            print("end spin()")
        else:
            print('TerraUTMSDKMainNode failed, please check launch file !!!')
    except rospy.ROSInterruptException:
        pass
