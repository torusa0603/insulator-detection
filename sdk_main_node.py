#!/usr/bin/env python3
import json
from time import gmtime, strftime
import rospy
import subprocess
import argparse
import os, sys
from std_msgs.msg import String
from terra_utm_sdk.msg import TelemetryData
from terra_utm_sdk.msg import UTMMessage
from terra_utm_sdk.msg import UTMMessageRes

import time

from object_files import define_value as DEFINE
from object_files.camera_control import CameraControl
from object_files.gimbal_control import GimbalControl
from object_files.data_from_telemetry import DataFromTelemetry
from object_files.instruction_from_click import InstructionFromClick
from object_files.gimbal_instruction_from_detection import GimbalInstructionFromDetection
from object_files.save_prev_data import SavePrevData

# opencv
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 
from datetime import datetime as dt
import numpy as np
from model import Model
import threading
import datetime
from collections import deque

import multiprocessing
import multiprocessing.sharedctypes
from multiprocessing import Value
import math
import datetime

manufacturer = rospy.get_param('manufacturer')
serial_id = rospy.get_param('serial_id')
clientId = rospy.get_param('thing_name')
pub_topic = manufacturer + '/' + serial_id + '/drone'
print(pub_topic)
parser = argparse.ArgumentParser(description='Display WLAN signal strength.')
parser.add_argument(dest='interface', nargs='?', default='wlan0', help='wlan interface (default: wlan0)')
args = parser.parse_args([])

gbl_camera_control = CameraControl()
gbl_gimbal_control = GimbalControl()
gbl_data_from_telemetry = DataFromTelemetry()
gbl_instruction_from_click = InstructionFromClick()
gbl_gimbal_instruction_from_detection = GimbalInstructionFromDetection()
gbl_save_prev_data = SavePrevData()


# if self.objectdetection_tower_mode == OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom:
#             self.flg_target_strict_judgement = False

class Coodinate():
    def __init__(self, x:float, y:float):
        self.x = int(x)
        self.y = int(y)

class WDH():
    def __init__(self, width:float, depth:float, height:float) -> None:
        self.width = int(width)
        self.depth = int(depth)
        self.height = int(height)

class ConstSettingValue():
    def __init__(
        self, target_model_class_indexes,  input_mode:str, 
        interval:int, objectdetection_target_model:str, model_path:str, output_path:str, 
        maxCornerX:int, maxCornerY:int, confidence_biases: dict, 
        sampling_count: int, group_bias:int, zoom_bias:float, wait_time: dict,
        cornerbias: int, flg_gimbal_variable_waiting_time: bool,
        flg_target_strict_judgement: bool,
        ):
        self.target_model_class_indexes = target_model_class_indexes
        self.input_mode = input_mode
        self.interval = interval
        self.objectdetection_target_model = objectdetection_target_model
        self.model_path = model_path
        self.output_path = output_path
        self.maxCornerX = maxCornerX                    #初期ジンバル最大角度X 
        self.maxCornerY = maxCornerY                    #初期ジンバル最大角度Y
        self.corner_add_value_X = self.maxCornerX/2
        self.corner_add_value_Y = self.maxCornerY/2     #初期中央四角(縦) 
        self.confidence_biases = confidence_biases          #認識率閾値
        self.sampling_count = sampling_count
        self.group_bias = group_bias
        self.zoom_bias = zoom_bias
        self.wait_time = wait_time
        self.cornerbias = cornerbias
        self.flg_gimbal_variable_waiting_time = flg_gimbal_variable_waiting_time
        self.flg_target_strict_judgement = flg_target_strict_judgement


class TerraUTMSDKMainNode():

    def __init__(self):
        self.watchdog_sub = rospy.Subscriber('watchdog', String, self.watchdogCallback)
        self.utm_pub = rospy.Publisher('utm_message', UTMMessage, queue_size=10)
        self.utm_sub = rospy.Subscriber('utm_message_res', UTMMessageRes, self.utmMessageResCallback)
        self.telemetry_sub = rospy.Subscriber('telemetry', TelemetryData, self.telemetryCallback)
        self.start_time = 0
        self.const_setting_value = ConstSettingValue(
            target_model_class_indexes=[DEFINE.CLASS_INSULATOR.t_660, DEFINE.CLASS_INSULATOR.t_, DEFINE.CLASS_INSULATOR.t_500], 
            input_mode = DEFINE.INPUT_MODE_GSTREAMER.udp,
            interval = DEFINE.INTERVAL.version_202203,
            objectdetection_target_model = DEFINE.OBJECTDETECTION_TARGET_MODEL.insulator,
            model_path = DEFINE.PATH.model,
            output_path = DEFINE.PATH.output,
            maxCornerX = 12, 
            maxCornerY = 7, 
            confidence_biases = {"default":0.45, "taicho": 0.3, "taicho_wide_to_zoom": 0.1, "show":0.3},
            sampling_count = 5,    
            group_bias = 100,       
            zoom_bias = 1.1,   
            wait_time={"zoom":3, "gimbal":4, "camera_change":1},
            cornerbias = 2,
            flg_gimbal_variable_waiting_time = False,
            flg_target_strict_judgement = True
            )

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def utmMessageResCallback(self, data):
        try:
            j = json.dumps({"type": data.type, "cmd": data.cmd, "result": data.result, "flight_id": data.flight_id})
            print(j)
        except Exception as e:
            _, _, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print("utmMessageResCallback: Json dump error -> {}, file:{} line: {}".format(e, fname, exc_tb.tb_lineno))

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def telemetryCallback(self, data):
        global gbl_gimbal_control
        global gbl_data_from_telemetry
        global gbl_save_prev_data

        try:
            gbl_gimbal_control.pitch = data.gimbal_pitch
            gbl_gimbal_control.roll = data.gimbal_roll
            gbl_gimbal_control.yaw = data.gimbal_yaw
            gbl_gimbal_control.mode = data.gimbal_mode
            gbl_data_from_telemetry.current_aircraft_waypoint = data.waypoint
            mission_status = data.mission_status
            objectdetection_status = data.objectdetection_status

            if data.objectdetection_mode == 0:
                gbl_data_from_telemetry.objectdetection_tower_mode = DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom
                gbl_data_from_telemetry.confidence_bias = self.const_setting_value.confidence_biases["taicho_wide_to_zoom"]

            print("gimbal_pitch: {}, gimbal_roll: {}, gimbal_yaw: {}, gimbal_mode: {}, gimbal_mode_old: {}, mission_status: {}, MISSION_STATUS_PAUSE: {}, objectdetection_status: {}".format(gbl_gimbal_control.pitch,gbl_gimbal_control.roll,gbl_gimbal_control.yaw,gbl_gimbal_control.mode,gbl_gimbal_control.mode_old,mission_status,DEFINE.MISSION_STATUS.pause,objectdetection_status))

            if (objectdetection_status != gbl_save_prev_data.objectdetection_status_old):
                print("モード切替！")
                if(objectdetection_status == DEFINE.OBJECTDETECTION_STATUS.start): # OD開始
                    gbl_data_from_telemetry.objectdetection_mode = DEFINE.OBJECTDETECTION_MODE.default_center
                    self.manualStartObjectDetection()
                elif(objectdetection_status == DEFINE.OBJECTDETECTION_STATUS.no_action) : # OD無効
                    #物体検知処理無効
                    gbl_data_from_telemetry.flg_objectdetection_auto = False
                    #ジンバル角度復元
                    self.gimbalControlYawPitch(yaw=(gbl_gimbal_control.yaw - gbl_gimbal_control.yaw_saved), pitch=(gbl_gimbal_control.pitch - gbl_gimbal_control.pitch_saved))
                    self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                    self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                    print("==========物体検知終了==========")
            gbl_gimbal_control.mode_old = gbl_gimbal_control.mode
            gbl_save_prev_data.objectdetection_status_old = objectdetection_status
        except Exception as e:
            print("telemetry error:{}".format(e))
            pass
        return

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def watchdogCallback(self, data):
        pass

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def start(self):
        trhead_detect =  threading.Thread(target=self.objectdetection, daemon=True, args=())
        trhead_detect.start()
        trhead_show = threading.Thread(target=self.showThread, daemon=True, args=())
        trhead_show.start()

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def manualStartObjectDetection(self):
        global gbl_data_from_telemetry
        global gbl_gimbal_control

        gbl_data_from_telemetry.flg_objectdetection_auto = True
        #　ジンバル角度保存
        gbl_gimbal_control.pitch_saved = gbl_gimbal_control.pitch
        gbl_gimbal_control.roll_saved  = gbl_gimbal_control.roll
        gbl_gimbal_control.yaw_saved   = gbl_gimbal_control.yaw

    def onMouseClick(self, event, x, y, flags, param):
        def clickZoomValue(x:int,y:int,y_inc:int):
            is_x_area = (x >= 0 and x <= 60)
            is_y_area = (y >= 120 + 40*(y_inc - 1) and y <= 160 + 40*(y_inc - 1))
            return (is_x_area and is_y_area)

        global gbl_data_from_telemetry
        global gbl_camera_control
        global gbl_instruction_from_click

        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)
            if x >= gbl_camera_control.width // 2 - gbl_instruction_from_click.targetSquareWidth // 2 and x <= gbl_camera_control.width // 2 + gbl_instruction_from_click.targetSquareWidth // 2 and y >= gbl_camera_control.height // 2 - gbl_instruction_from_click.targetSquareHeight // 2 and y <= gbl_camera_control.height // 2 + gbl_instruction_from_click.targetSquareHeight // 2 :
                #中心付近をクリックすると物体検知処理無効
                print("objectdetection off")
                gbl_data_from_telemetry.flg_objectdetection_auto = False
            elif x >= 0 and x <= 60 and y >= 0 and y <= 60 :
                #左上部分をクリックすると物体検知処理有効
                print("objectdetection on")
                self.manualStartObjectDetection()
            elif x >= 0 and x <= 120 and y >= gbl_camera_control.height - 60 and y <= gbl_camera_control.height :
                #レンズ切り替え
                if (gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.dummy_pause) :
                    #処理再開
                    print("objectdetection continue(change lens(Pause->Zoom))")
                    gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.zoom
                elif gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide :
                    #処理一時停止
                    print("objectdetection pause(change lens(Wide->Pause))")
                    gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.dummy_pause
                elif gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.zoom :
                    #処理一時停止
                    print("objectdetection pause(change lens(Wide->Pause))")
                    gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.dummy_pause
            elif x >= gbl_camera_control.width - 120 and x <= gbl_camera_control.width and y >= gbl_camera_control.height - 60 and y <= gbl_camera_control.height :
                    print("objectdetection wide(change lens(all->Wide))")
                    self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                    #ズーム初期値
                    self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
            elif x >= 0 and x <= 60 and y >= 80 and y <= 120 :
                #カメラ切替
                # <> と表示されている部分
                if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                    gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.zoom
                    self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.zoom)
                else:
                    gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.wide
                    self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
            # ×(倍率) よりも下部分
            elif clickZoomValue(x=x,y=y,y_inc=1):
                #x2
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
            elif clickZoomValue(x=x,y=y,y_inc=2):
                #x4
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_4)
            elif clickZoomValue(x=x,y=y,y_inc=3):
                #x5
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_5)
            elif clickZoomValue(x=x,y=y,y_inc=4):
                #x6
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_6)
            elif clickZoomValue(x=x,y=y,y_inc=5):
                #x8
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_8)
            elif clickZoomValue(x=x,y=y,y_inc=6):
                #x10
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_10)
            elif clickZoomValue(x=x,y=y,y_inc=7):
                #x12
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_12)
            elif clickZoomValue(x=x,y=y,y_inc=8):
                #x16
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_16)
            elif clickZoomValue(x=x,y=y,y_inc=9):
                #x20
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_20)
            else : 
                self.gimbalControlXY(x=x, y=y, cameraType=gbl_instruction_from_click.currentCameraLens, 
                    zoomVlue=gbl_gimbal_instruction_from_detection.zoomValueCurrent, width=gbl_camera_control.width, height=gbl_camera_control.height)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def gimbalControlYawPitch(self, yaw, pitch):
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 1,"action_type": 2,"action_value": str(yaw) +","+ str(pitch)}
        )

        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)

    def gimbalControlXY(self, x:float, y:float, cameraType:DEFINE.CAMERALENS, zoomVlue:int, width:int, height:int):
        # zoom率の大きさによってfovの値が変わる
        if cameraType == DEFINE.CAMERALENS.wide:
            diagonal_fov = DEFINE.CAMERA_PARAMETER.fov
        else:
            diagonal_fov = 2 * math.degrees(math.atan(math.tan(math.radians(DEFINE.CAMERA_PARAMETER.fov / 2)) / zoomVlue))

        # fovが対角方向に対するものなんで、水平・垂直方向のfovに変更するための値を作成
        destination = Coodinate(x=x,y=y)
        diagonal_pix =  math.sqrt(width ** 2 + height ** 2)
        horizontal_fov_wide = diagonal_fov * width / diagonal_pix
        vertical_fov_wide   = diagonal_fov * height / diagonal_pix

        yaw = 0.0
        pitch = 0.0

        # 横縦方向のそれぞれ、全体に対する移動させたい座標への割合から変更させたい角度の大きさを計算
        yaw = math.degrees(math.atan(2 * (destination.x -width / 2) * math.tan(math.radians(horizontal_fov_wide / 2)) / width))
        pitch = -1 * math.degrees(math.atan(2 * (destination.y -height / 2) * math.tan(math.radians(vertical_fov_wide / 2)) / height))

        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 1,"action_type": 2,"action_value": str(yaw) +","+ str(pitch)}
        )

        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)

        return yaw, pitch

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def cameraControlZoom(self,zoomvalue):
        global gbl_gimbal_instruction_from_detection
        gbl_gimbal_instruction_from_detection.zoomValueCurrent = zoomvalue
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 2,"action_type": 0,"action_value": str(zoomvalue)}
        )
        print(j)

        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def cameraControlFocus(self,focusmode,x,y):
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 2,"action_type": 1,"action_value": str(focusmode) + "," + str(x) + "," + str(y)}
        )
        print(j)

        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def cameraControlChangeLens(self,lensvalue):
        global gbl_instruction_from_click
        # このプログラム内におけるカメラ種類を保存(OSDKから現在のカメラ種類を取得できればそれが一番良いんだけどね、、、)
        if lensvalue == DEFINE.OSDK_CAMERA_SOURCE_H20T.wide:
            gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.wide
        else:
            gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.zoom
        
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 2,"action_type": 2,"action_value": str(lensvalue)}
        )
        print(j)

        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.target_type = jd["target_type"]
        utmMessage.action_type = jd["action_type"]
        utmMessage.action_value = jd["action_value"]
        self.utm_pub.publish(utmMessage)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def droneControlActionResume(self):
        j = json.dumps(
            {"cmd": "fp_resume", "type": "ctrl_drone", "flight_id": "dummy"}
        )
        print(j)

        jd = json.loads(j)
        utmMessage = UTMMessage()
        utmMessage.type = jd["type"]
        utmMessage.cmd = jd["cmd"]
        utmMessage.flight_id = jd["flight_id"]

        self.utm_pub.publish(utmMessage)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def openCapture(self,isGstreamerUri :bool):
        if isGstreamerUri:
            cmd = DEFINE.INPUT_MODE_GSTREAMER.uri_str
        else :
            cmd = DEFINE.INPUT_MODE_GSTREAMER.udp_str
        print(F'OpenGStreamer:{cmd}')
        try :
            cap = cv2.VideoCapture(cmd, cv2.CAP_GSTREAMER)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception as e:
            print(e)
            return {'Error': True}
        if not cap.isOpened():
            print("Failed to open the camera...")
            return {'Error': True}
        return {'Error': False, 'cap': cap}

    def drawDisplayImage(self, img, flag_detection : bool, zoomValueCurrent : int):
        global gbl_camera_control
        global gbl_instruction_from_click
        centerX = gbl_camera_control.width // 2
        centerY = gbl_camera_control.height // 2
        #十字
        cv2.line(img, (0, centerY), (gbl_camera_control.width, centerY), (175, 175, 175), thickness=1)
        cv2.line(img, (centerX, 0), (centerX, gbl_camera_control.height), (175, 175, 175), thickness=1)
        #ターゲット
        cv2.rectangle(img,(centerX - gbl_instruction_from_click.targetSquareWidth // 2, centerY - gbl_instruction_from_click.targetSquareHeight // 2),(centerX + gbl_instruction_from_click.targetSquareWidth // 2, centerY + gbl_instruction_from_click.targetSquareHeight // 2),(175,175,175),1)
        #左上の枠
        cv2.rectangle(img, (0, 0),(60,60),(175,175,175),1)
        #左下の枠
        cv2.rectangle(img,(0, gbl_camera_control.height - 60),(120, gbl_camera_control.height),(175,175,175),1)
        #右下の枠
        cv2.rectangle(img,(gbl_camera_control.width - 120, gbl_camera_control.height - 60),(gbl_camera_control.width, gbl_camera_control.height),(175,175,175),1)
        #waypoint表示 
        label = "WP:" + str(gbl_data_from_telemetry.current_aircraft_waypoint)
        img = cv2.putText(img,label,(0, gbl_camera_control.height-10-20-35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
        if flag_detection:
            #モード表示
            label = gbl_data_from_telemetry.objectdetection_mode
            img = cv2.putText(img,label,(0, gbl_camera_control.height-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
            #レンズモード表示
            label = gbl_instruction_from_click.currentCameraLens
            if label == DEFINE.CAMERALENS.dummy_pause : 
                img = cv2.putText(img,label,(0, gbl_camera_control.height-10-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
            else :
                img = cv2.putText(img,label,(0, gbl_camera_control.height-10-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            #検知時のパラメータ表示 
            label = "Model:" + self.const_setting_value.objectdetection_target_model + ", Mode:" + str(gbl_data_from_telemetry.objectdetection_tower_mode) + ", conf:" + str(gbl_data_from_telemetry.confidence_bias) + ", conf_show:" + str(self.const_setting_value.confidence_biases["show"]) + ", camera:" + str(gbl_instruction_from_click.currentCameraLens) + ", zoom:" + str(zoomValueCurrent)
            img = cv2.putText(img,label,(0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),1, cv2.LINE_AA)
        else :
            rectangl_width = 60
            rectangl_height = 40 
            #カメラ切替
            start_y = 80
            currnet_start_y = start_y
            cv2.rectangle(img,(0, currnet_start_y),(rectangl_width,currnet_start_y + rectangl_height),(175,175,175),1)
            label = "<>"
            currnet_start_y += rectangl_height
            img = cv2.putText(img,label,(0, currnet_start_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, cv2.LINE_AA)
            #ズーム用枠
            start_y = 120
            currnet_start_y = start_y
            cv2.rectangle(img,(0, currnet_start_y),(rectangl_width,currnet_start_y + rectangl_height),(175,175,175),1)
            label = "x" + str(zoomValueCurrent)
            currnet_start_y += rectangl_height
            img = cv2.putText(img,label,(0, currnet_start_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
        return img

    def analyzeProbGroup(self, prob):
        def compareSize(a, b):
            if a > b:
                return [b,a]
            else:
                return [a,b]

        len_prob = len(prob)
        if (len_prob == 0):
            return {'Error': True}
        else:
            # 左上が原点とし、下方向がy+、右方向がx+
            min_y = gbl_camera_control.height
            min_y_indx = 0
            max_y = 0
            max_y_indx = 0
            min_x = gbl_camera_control.width
            max_x = 0
            sum_w = 0
            sum_h = 0
            for i in range(len_prob) :
                sum_w += prob[i][2]
                sum_h += prob[i][3]
                if min_y > prob[i][1]:
                    min_y = prob[i][1]
                    min_y_indx = i
                if max_y < prob[i][1] :
                    max_y = prob[i][1]
                    max_y_indx = i
                if min_x > prob[i][0]:
                    min_x = prob[i][0]
                if max_x < prob[i][0] :
                    max_x = prob[i][0]
            average_w = sum_w / len_prob
            average_h = sum_h / len_prob
            
            # 元とする点からの傾きを角度換算で昇順リストを作成
            a_array_base_ymin_rad = self.calculateSlopes(prob=prob, base_indx=min_y_indx)
            a_array_base_ymax_rad = self.calculateSlopes(prob=prob, base_indx=max_y_indx)
            # 角度リストから使用する角度の値を抽出
            a_base_ymin_rad, max_count_ymin = self.extractSlope(a_array_base_ymin_rad)
            a_base_ymax_rad, max_count_ymax = self.extractSlope(a_array_base_ymax_rad)
            max_count = (max_count_ymin + max_count_ymax) / 2
            print(f"max_count : {max_count}")

            # 2本の傾きの値がかけ離れすぎていたら、、、
            if (abs(a_base_ymin_rad - a_base_ymax_rad) > 3):
                return {'Error': True}

            a_synthesis_rad = (a_base_ymin_rad + a_base_ymax_rad) / 2
            
            ratio_max_all = max_count / len_prob
            print(f"ratio_max_all : {ratio_max_all}")
            if ratio_max_all < 0.3:
                # 複数本並列で碍子が並んでいる場合を想定処理
                # 高さのmaxとminで各列の端の碍子を取りたいが、うまく取れずにymaxとyminが同一列上にきてしまった場合は弾く
                dif_x = prob[min_y_indx][0] - prob[max_y_indx][0]
                dif_y = prob[min_y_indx][1] - prob[max_y_indx][1]
                if dif_x == 0:
                    return {'Error': True}
                a = dif_y / dif_x
                a_between_ymin_ymax_rad = int(math.degrees(math.atan(a)))
                if (abs(a_between_ymin_ymax_rad - a_synthesis_rad) < 3):
                     return {'Error': True}

            # y=ax+bの一次関数の係数の形にする
            a_synthesis = math.tan(math.radians(a_synthesis_rad))
            if abs(a_synthesis) > 1:
                return {'Error': True} 
            b_synthesis = ((prob[max_y_indx][1] - a_synthesis*prob[max_y_indx][0]) + (prob[min_y_indx][1] - a_synthesis*prob[min_y_indx][0]))/2
            if a_synthesis < 0:
                # 傾きが負である時は右端から始める
                x_on_line = max_x
            else :
                # 傾きが正である時は左端から始める
                x_on_line = min_x

            y_on_line = a_synthesis * x_on_line + b_synthesis
            base_coordinate = Coodinate(x=x_on_line,y=y_on_line)

            left_right = compareSize(prob[min_y_indx][0], prob[max_y_indx][0])
            top_bottom = compareSize(prob[min_y_indx][1], prob[max_y_indx][1])
            group_left_top     = Coodinate(x=left_right[0] - average_w / 2, y=top_bottom[0] - average_h / 2)
            group_right_bottom = Coodinate(x=left_right[1] + average_w / 2, y=top_bottom[1] + average_h / 2)

            return {"Error": False, "slopes": a_synthesis, "base_coordinate": base_coordinate, 
                "width" : average_w, "height" : average_h, "group_box" : {"left_top" : group_left_top,"right_bottom" : group_right_bottom}}
    

    def extractSlope(self, slopes:list):
        cunt_array = [0] * 181
        cunt = 0
        i = -1

        # 傾き値(角度)の範囲が-90〜90　であるので一度毎の度数分布表を作成する
        for sl in slopes:
            i += 1
            if i == 0:
                prev = sl
                continue
            if prev == sl:
                cunt += 1
                continue
            cunt_array[prev + 90] = cunt
            prev = sl
            cunt = 0
        mode_index = np.argmax(cunt_array)
        mode = mode_index - 90
        count_area = cunt_array[mode_index - 1] + cunt_array[mode_index] + cunt_array[mode_index + 1]
        return mode, count_area
    

    def calculateSlopes(self, prob:list, base_indx:int):
        a_tan_array = []
        for i in range(len(prob)) :
            if i == base_indx:
                continue
            else :
                dif_x = prob[i][0] - prob[base_indx][0]
                dif_y = prob[i][1] - prob[base_indx][1]
                if dif_x == 0:
                    continue
                a = dif_y / dif_x
                a_tan_array.append(int(math.degrees(math.atan(a))))
        return sorted(a_tan_array)


    def validetatePoint(self , point:Coodinate, width:int, height:int):
        if point.x < 0:
            point.x = 0
        if point.x > width:
            point.x = width
        if point.y < 0:
            point.y = 0
        if point.y > height:
            point.y = height
        return point

    def decideProbCount(self, prob_width:int, group_prob_start:Coodinate, group_prob_end:Coodinate):
        count_prob = int((group_prob_end.x - group_prob_start.x) / prob_width)
        print(f"count!!!!!!!!!!!!{count_prob}")
        count = 1
        while True:
            if count == 10:
                # 100連碍子が登場したら変更してください
                break
            if count_prob < (count * 10):
                break
            else:
                count += 1
        return (count * 10)


    def objectdetection(self):
        global gbl_gimbal_control
        global gbl_camera_control
        global gbl_data_from_telemetry
        global gbl_instruction_from_click
        global gbl_gimbal_instruction_from_detection
        global gbl_save_prev_data

        flg_objectdetection_auto_prev = False
        zoom_detect_process = False
        wide_detect_skip = False
        wide_detect_timeout = 10

        # self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
        time.sleep(1)

        ret = self.openCapture(isGstreamerUri=(self.const_setting_value.input_mode == DEFINE.INPUT_MODE_GSTREAMER.uri))
        if ret['Error']:
            sys.exit()
        cap = ret['cap']
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        print("fps:{} width:{} height:{}".format(fps, width, height))

        print("cap test")
        #モデル読み込み
        model = Model(DEFINE.DISPLAY_ATTRIBUTES.classes, DEFINE.DISPLAY_ATTRIBUTES.colors, height, width, self.const_setting_value.model_path)


        #描画設定
        gbl_camera_control.width = int(width)     #カメラ幅
        gbl_camera_control.height = int(height)    #カメラ高さ
        print("camera_width:{},camera_height:{}".format(gbl_camera_control.width,gbl_camera_control.height))

        gbl_gimbal_instruction_from_detection.centerX = gbl_camera_control.width // 2   #中心(横)
        gbl_gimbal_instruction_from_detection.centerY = gbl_camera_control.height // 2  #中心(縦)

        counter = 0
        count_fail_open_camera = 0
        limit_count_fail_open_camera = 10
        while not rospy.is_shutdown():
            if not cap.isOpened():
                if count_fail_open_camera > limit_count_fail_open_camera:
                    sys.exit()
                count_fail_open_camera += 1
                ret = self.openCapture(isGstreamerUri=(self.const_setting_value.input_mode == DEFINE.INPUT_MODE_GSTREAMER.uri))
                if ret['Error']:
                    time.sleep(0.5)
                    continue
                else:
                    cap = ret['cap']
            _, frame = cap.read()
            if(frame is None):
                try :
                    pass
                except Exception as e :
                    print("error cap:{}".format(e))
                continue
            if (gbl_data_from_telemetry.flg_objectdetection_auto == True ) :
                if flg_objectdetection_auto_prev == False :
                    #物体検知開始の初期処理
                    print("▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼物体検知開始▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼")
                    self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                    self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                    self.cameraControlFocus(2,0.5,0.5)
                    flg_objectdetection_auto_prev = True
                    sum_yaw = 0
                    sum_pitch = 0
                    dt_start_detect = datetime.datetime.now()
                    wide_detect_skip = False
                    continue 
                if (counter % self.const_setting_value.interval != 0):
                    pass
                else :
                    counter = 0
                    if wide_detect_skip:
                        if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                            self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.zoom)
                            time.sleep(1)
                            self.cameraControlFocus(2,0.5,0.5)
                            time.sleep(1)

                    threadhold = 0.0
                    if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                        threadhold = 0.3
                    else:
                        threadhold = 0.4
                    # 保存用
                    disp_frame, prob = model.inference(frame, threadhold)
                    result = self.analyzeProbGroup(prob)
                    if result["Error"] == False:
                        tem_a = result["slopes"]
                        base_coordinate = result["base_coordinate"]
                        prob_wdh = WDH(width=result["width"],height=result["height"],depth=0)
                        group_prob_start = result["group_box"]["left_top"]
                        group_prob_end   = result["group_box"]["right_bottom"]

                        disp_frame = self.drawDisplayImage(img = disp_frame, flag_detection = gbl_data_from_telemetry.flg_objectdetection_auto, 
                            zoomValueCurrent = gbl_gimbal_instruction_from_detection.zoomValueCurrent)
                        # 碍子連に沿った直線の描写
                        start_pt = Coodinate(x=base_coordinate.x, y=base_coordinate.y)
                        start_pt = self.validetatePoint(point=start_pt, width=gbl_camera_control.width, height=gbl_camera_control.height)
                        if tem_a < 0 :
                            end_pt = Coodinate(x=start_pt.x - gbl_camera_control.width/3,y=start_pt.y - tem_a*gbl_camera_control.width/3)
                        else :
                            end_pt = Coodinate(x=start_pt.x + gbl_camera_control.width/3,y=start_pt.y + tem_a*gbl_camera_control.width/3)
                        end_pt = self.validetatePoint(point=end_pt, width=gbl_camera_control.width, height=gbl_camera_control.height)
                        cv2.line(disp_frame, (start_pt.x, start_pt.y), (end_pt.x, end_pt.y), (0, 0, 255), thickness=3)
                        # 碍子連全体を囲むボックスの描写
                        group_prob_start = self.validetatePoint(point=group_prob_start, width=gbl_camera_control.width, height=gbl_camera_control.height)
                        group_prob_end = self.validetatePoint(point=group_prob_end, width=gbl_camera_control.width, height=gbl_camera_control.height)
                        cv2.rectangle(disp_frame, (group_prob_start.x, group_prob_start.y),(group_prob_end.x, group_prob_end.y),(255, 0, 0),3)
                        yaw, pitch = self.gimbalControlXY(x=base_coordinate.x, y=base_coordinate.y, cameraType=gbl_instruction_from_click.currentCameraLens, 
                            zoomVlue=gbl_gimbal_instruction_from_detection.zoomValueCurrent, width=gbl_camera_control.width, height=gbl_camera_control.height)
                        sum_yaw += yaw    
                        sum_pitch += pitch
                        if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                            self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.zoom)
                            time.sleep(1)
                            self.cameraControlFocus(2,0.5,0.5)
                            time.sleep(1)
                        else:
                            if zoom_detect_process:
                                zoom_value = int(gbl_camera_control.height / (prob_wdh.height * 2))
                                self.cameraControlZoom(zoom_value)
                                time.sleep(zoom_value / 4)
                                self.cameraControlFocus(2,0.5,0.5)
                                time.sleep(2)
                                yaw = 0
                                pitch = 0
                                i_count = 0
                                prob_count = self.decideProbCount(prob_width=prob_wdh.width, group_prob_start=group_prob_start, group_prob_end=group_prob_end)
                                prob_width_current_zoom = prob_wdh.width * (zoom_value / 2)
                                move_x = gbl_camera_control.width / 2
                                # 1回目の首振りは腕を見にいくための処理のため
                                number_of_gimbal_movements = int(prob_width_current_zoom * prob_count / move_x) + 2
                                while i_count < number_of_gimbal_movements:
                                    i_count += 1
                                    start_pt = Coodinate(x=(gbl_camera_control.width / 2), y=(gbl_camera_control.height / 2))
                                    start_pt = self.validetatePoint(point=start_pt, width=gbl_camera_control.width, height=gbl_camera_control.height)
                                    # 1回目は碍子一個分戻る(接合部を見たいから)
                                    if i_count == 1:
                                        if tem_a < 0 :
                                            end_pt = Coodinate(x=(start_pt.x + move_x), y=(start_pt.y + tem_a*move_x))
                                        else :
                                            end_pt = Coodinate(x=(start_pt.x - move_x), y=(start_pt.y - tem_a*move_x))
                                    else:
                                        if tem_a < 0 :
                                            end_pt = Coodinate(x=(start_pt.x - move_x), y=(start_pt.y - tem_a*move_x))
                                        else :
                                            end_pt = Coodinate(x=(start_pt.x + move_x), y=(start_pt.y + tem_a*move_x))
                                    yaw, pitch = self.gimbalControlXY(x=end_pt.x, y=end_pt.y, cameraType=gbl_instruction_from_click.currentCameraLens, 
                                        zoomVlue=gbl_gimbal_instruction_from_detection.zoomValueCurrent, width=gbl_camera_control.width, height=gbl_camera_control.height)
                                    sum_yaw += yaw    
                                    sum_pitch += pitch
                                    _, frame = cap.read()
                                    disp_frame = self.drawDisplayImage(img = disp_frame, flag_detection = gbl_data_from_telemetry.flg_objectdetection_auto, 
                                        zoomValueCurrent = gbl_gimbal_instruction_from_detection.zoomValueCurrent)
                                    self.disp_frame = np.copy(disp_frame)
                                zoom_detect_process = False
                                self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                                time.sleep(2)
                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                # 角度を復刻しておく
                                self.gimbalControlYawPitch(yaw = -1*sum_yaw, pitch = -1*sum_pitch)
                                time.sleep(1)
                                #フライトプラン再開
                                self.droneControlActionResume()
                            
                                gbl_data_from_telemetry.flg_objectdetection_auto = False
                                flg_objectdetection_auto_prev = False  #test20220404
                            else:
                                zoom_detect_process = True
                    else:
                        dt_now = datetime.datetime.now()
                        if (dt_now - dt_start_detect).seconds > wide_detect_timeout:
                            wide_detect_skip = True
                        disp_frame = self.drawDisplayImage(img = disp_frame, flag_detection = gbl_data_from_telemetry.flg_objectdetection_auto, 
                            zoomValueCurrent = gbl_gimbal_instruction_from_detection.zoomValueCurrent)
                    # 画像表示
                    self.disp_frame = np.copy(disp_frame)
                    #レンズがポーズのときは次に行く
                    if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.dummy_pause :
                        counter += 1
                        continue
            else :
                disp_frame = self.drawDisplayImage(img = frame, flag_detection = gbl_data_from_telemetry.flg_objectdetection_auto, 
                    zoomValueCurrent = gbl_gimbal_instruction_from_detection.zoomValueCurrent)
                # 画像表示
                self.disp_frame = disp_frame
                #フラグ保持
                flg_objectdetection_auto_prev = False
            counter += 1
        cap.release()
        self.watchdog_sub.unregister()
        self.utm_pub.unregister()
        self.utm_sub.unregister()
        self.telemetry_sub.unregister()
        rospy.signal_shutdown('objectdetection')
        subprocess.run('echo "terra1234" | sudo -Ss;/home/terra/kill_all_rosnode.sh', shell=True)


    def showThread(self):
        global gbl_data_from_telemetry
        #vmware最大画面用
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.setMouseCallback('frame',self.onMouseClick)
        # 表示用画像が流れ出すかをチェック
        while not hasattr(self, "disp_frame"):
            wk = cv2.waitKey(1000) 
        while True:
            # 画像表示
            cv2.imshow('frame', self.disp_frame)
            wk = cv2.waitKey(1) 
            if wk==13:
                break
            elif wk==87:
                os.system("echo 'terra1234'|sudo -Ss;sudo /sbin/reboot")
            elif wk==97: #a
                 cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
                 cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            elif wk==119: #w
                cv2.namedWindow('frame', cv2.WND_PROP_AUTOSIZE)
                cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN , cv2.WINDOW_NORMAL)
            elif wk==-1:
                pass
            else:
                print("wk:{}".format(wk))
        cv2.destroyAllWindows()
        for i in range(5):
            cv2.waitKey(1)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('objectdetection', anonymous=True)
    try:
        TerraUTMSDKMainNode().start()
        print("start spin()")
        rospy.spin()
        print("end spin()")
    except rospy.ROSInterruptException:
        pass
