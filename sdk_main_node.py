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
        self.x = x
        self.y = y

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

            mission_status = data.mission_status

            objectdetection_status = data.objectdetection_status

            gbl_data_from_telemetry.current_aircraft_waypoint = data.waypoint

            if data.objectdetection_mode == 0:
                gbl_data_from_telemetry.objectdetection_tower_mode = DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom
                gbl_data_from_telemetry.confidence_bias = self.const_setting_value.confidence_biases["taicho_wide_to_zoom"]

            try:
                gbl_gimbal_control.pitch_saved
            except NameError:
                print("error test")
                gbl_gimbal_control.pitch_saved = gbl_gimbal_control.pitch

            try:
                gbl_gimbal_control.yaw_saved
            except NameError:
                print("error test")
                gbl_gimbal_control.yaw_saved = gbl_gimbal_control.yaw

            print("gimbal_pitch: {}, gimbal_roll: {}, gimbal_yaw: {}, gimbal_mode: {}, gimbal_mode_old: {}, mission_status: {}, MISSION_STATUS_PAUSE: {}, objectdetection_status: {}".format(gbl_gimbal_control.pitch,gbl_gimbal_control.roll,gbl_gimbal_control.yaw,gbl_gimbal_control.mode,gbl_gimbal_control.mode_old,mission_status,DEFINE.MISSION_STATUS.pause,objectdetection_status))

            if (objectdetection_status != gbl_save_prev_data.objectdetection_status_old):
                print("モード切替！")
                if(objectdetection_status == DEFINE.OBJECTDETECTION_STATUS.start): # OD開始
                    #物体検知処理有効
                    gbl_data_from_telemetry.flg_objectdetection_auto = True
                    gbl_data_from_telemetry.objectdetection_mode = DEFINE.OBJECTDETECTION_MODE.default_center
                    #ジンバル角度保存
                    gbl_gimbal_control.pitch_saved = gbl_gimbal_control.pitch
                    gbl_gimbal_control.roll_saved  = gbl_gimbal_control.roll
                    gbl_gimbal_control.yaw_saved   = gbl_gimbal_control.yaw

                    print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved))
                    pass
                elif(objectdetection_status == DEFINE.OBJECTDETECTION_STATUS.no_action) : # OD無効
                    #物体検知処理無効
                    self.gimbalcornerReset()
                    self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                    flg_objectdetection_auto = False
                    #ジンバル角度復元
                    self.gimbalControlYawTiltLargeAbsolute(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved)

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
        trhead1 =  threading.Thread(target=self.objectdetection, daemon=True, args=())
        trhead1.start()
        trhead2 = threading.Thread(target=self.showThread, daemon=True, args=())
        trhead2.start()




#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




    def onMouseClick(self,event,x,y,flags,param):
        global gbl_data_from_telemetry
        global gbl_gimbal_control
        global gbl_camera_control
        global gbl_instruction_from_click

        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)
            # print("click camera_width:{},camera_height:{}x:{}, y:{}".format(gbl_camera_control.width,gbl_camera_control.height,x,y))

            if x >= gbl_camera_control.width // 2 - gbl_instruction_from_click.targetSquareWidth // 2 and x <= gbl_camera_control.width // 2 + gbl_instruction_from_click.targetSquareWidth // 2 and y >= gbl_camera_control.height // 2 - gbl_instruction_from_click.targetSquareHeight // 2 and y <= gbl_camera_control.height // 2 + gbl_instruction_from_click.targetSquareHeight // 2 :
                #中心付近をクリックすると物体検知処理無効
                print("gimbalcornerReset")
                self.gimbalcornerReset()
                print("objectdetection off")
                gbl_data_from_telemetry.flg_objectdetection_auto = False
            elif x >= 0 and x <= 60 and y >= 0 and y <= 60 :
                #左上部分をクリックすると物体検知処理有効
                print("objectdetection on")
                gbl_data_from_telemetry.flg_objectdetection_auto = True

                #ジンバル角度保存
                gbl_gimbal_control.pitch_saved = gbl_gimbal_control.pitch
                gbl_gimbal_control.roll_saved  = gbl_gimbal_control.roll
                gbl_gimbal_control.yaw_saved   = gbl_gimbal_control.yaw
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
                    gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.wide

                    #パラメータ初期化
                    self.gimbalcornerReset()
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
            elif x >= 0 and x <= 60 and y >= 120 and y <= 160 :
                #x2
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
            elif x >= 0 and x <= 60 and y >= 160 and y <= 200 :
                #x4
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_4)
            elif x >= 0 and x <= 60 and y >= 200 and y <= 240 :
                #x5
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_5)
            elif x >= 0 and x <= 60 and y >= 240 and y <= 280 :
                #x6
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_6)
            elif x >= 0 and x <= 60 and y >= 280 and y <= 320 :
                #x8
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_8)
            elif x >= 0 and x <= 60 and y >= 320 and y <= 360 :
                #x10
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_10)
            elif x >= 0 and x <= 60 and y >= 360 and y <= 400 :
                #x12
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_12)
            elif x >= 0 and x <= 60 and y >= 400 and y <= 440 :
                #x16
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_16)
            elif x >= 0 and x <= 60 and y >= 440 and y <= 480 :
                #x20
                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_20)
            else : 
                self.gimbalControlXY(x=x,y=y)
               




#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------





    def gimbalcornerReset(self):
        global gbl_instruction_from_click
        global gbl_gimbal_instruction_from_detection
        global gbl_instruction_from_click
        
        gbl_gimbal_instruction_from_detection.gimbalCornerX = self.const_setting_value.maxCornerX
        gbl_gimbal_instruction_from_detection.gimbalCornerY = self.const_setting_value.maxCornerY
        gbl_gimbal_instruction_from_detection.zoomValueCurrent = DEFINE.ZOOM_VALUE.Val_2                #ズーム初期化

        gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.wide #レンズはwideで初期化




#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------





    def gimbalcornerResetOnlyCorner(self):
        global gbl_gimbal_instruction_from_detection
        
        gbl_gimbal_instruction_from_detection.gimbalCornerX = self.const_setting_value.maxCornerX
        gbl_gimbal_instruction_from_detection.gimbalCornerY = self.const_setting_value.maxCornerY    




#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




    def gimbalControlYawTiltLarge(self, x, y):
        global gbl_gimbal_instruction_from_detection

        relatedX = x
        relatedY = y 

        # X軸移動
        if relatedX < gbl_gimbal_instruction_from_detection.centerX:
            diffCenterX = gbl_gimbal_instruction_from_detection.centerX - relatedX

            if gbl_gimbal_instruction_from_detection.relatedX_old != -1:
                if gbl_gimbal_instruction_from_detection.relatedX_old < gbl_gimbal_instruction_from_detection.centerX :
                    gbl_gimbal_instruction_from_detection.gimbalCornerX = gbl_gimbal_instruction_from_detection.gimbalCornerX + self.const_setting_value.corner_add_value_X  #test20220307
                elif gbl_gimbal_instruction_from_detection.relatedX_old > gbl_gimbal_instruction_from_detection.centerX :
                    gbl_gimbal_instruction_from_detection.gimbalCornerX = gbl_gimbal_instruction_from_detection.gimbalCornerX - self.const_setting_value.corner_add_value_X  #test20220307
                  
            yaw = -1 * gbl_gimbal_instruction_from_detection.gimbalCornerX * (diffCenterX / gbl_gimbal_instruction_from_detection.centerX)
            yaw = yaw * self.const_setting_value.cornerbias / gbl_gimbal_instruction_from_detection.zoomValueCurrent
            
        elif relatedX > gbl_gimbal_instruction_from_detection.centerX :
            diffCenterX = relatedX - gbl_gimbal_instruction_from_detection.centerX

            if gbl_gimbal_instruction_from_detection.relatedX_old != -1 :
                if gbl_gimbal_instruction_from_detection.relatedX_old < gbl_gimbal_instruction_from_detection.centerX :
                    gbl_gimbal_instruction_from_detection.gimbalCornerX = gbl_gimbal_instruction_from_detection.gimbalCornerX - self.const_setting_value.corner_add_value_X  #test20220307
                elif gbl_gimbal_instruction_from_detection.relatedX_old > gbl_gimbal_instruction_from_detection.centerX :
                    gbl_gimbal_instruction_from_detection.gimbalCornerX = gbl_gimbal_instruction_from_detection.gimbalCornerX + self.const_setting_value.corner_add_value_X  #test20220307

            yaw = gbl_gimbal_instruction_from_detection.gimbalCornerX * (diffCenterX / gbl_gimbal_instruction_from_detection.centerX)
            yaw = yaw * self.const_setting_value.cornerbias / gbl_gimbal_instruction_from_detection.zoomValueCurrent

        else:
            yaw = 0 #　センター線上
        
        #  Y軸移動
        if relatedY < gbl_gimbal_instruction_from_detection.centerY :
            diffCenterY = gbl_gimbal_instruction_from_detection.centerY - relatedY

            if gbl_gimbal_instruction_from_detection.relatedY_old != -1 :
                if gbl_gimbal_instruction_from_detection.relatedY_old < gbl_gimbal_instruction_from_detection.centerY :
                    gbl_gimbal_instruction_from_detection.gimbalCornerY = gbl_gimbal_instruction_from_detection.gimbalCornerY + self.const_setting_value.corner_add_value_Y  #test20220307
                elif gbl_gimbal_instruction_from_detection.relatedY_old > gbl_gimbal_instruction_from_detection.centerY :
                    gbl_gimbal_instruction_from_detection.gimbalCornerY = gbl_gimbal_instruction_from_detection.gimbalCornerY - self.const_setting_value.corner_add_value_Y  #test20220307
            tilt = gbl_gimbal_instruction_from_detection.gimbalCornerY * (diffCenterY / gbl_gimbal_instruction_from_detection.centerY)
            tilt = tilt * self.const_setting_value.cornerbias / gbl_gimbal_instruction_from_detection.zoomValueCurrent
        elif relatedY > gbl_gimbal_instruction_from_detection.centerY :
            diffCenterY = relatedY - gbl_gimbal_instruction_from_detection.centerY

            if gbl_gimbal_instruction_from_detection.relatedY_old != -1 :
                if gbl_gimbal_instruction_from_detection.relatedY_old < gbl_gimbal_instruction_from_detection.centerY :
                    gbl_gimbal_instruction_from_detection.gimbalCornerY = gbl_gimbal_instruction_from_detection.gimbalCornerY - self.const_setting_value.corner_add_value_Y  #test20220307
                elif gbl_gimbal_instruction_from_detection.relatedY_old > gbl_gimbal_instruction_from_detection.centerY :
                    gbl_gimbal_instruction_from_detection.gimbalCornerY = gbl_gimbal_instruction_from_detection.gimbalCornerY + self.const_setting_value.corner_add_value_Y  #test20220307
            tilt = -1 * gbl_gimbal_instruction_from_detection.gimbalCornerY * (diffCenterY / gbl_gimbal_instruction_from_detection.centerY)
            tilt = tilt * self.const_setting_value.cornerbias / gbl_gimbal_instruction_from_detection.zoomValueCurrent
        else:
            tilt = 0            #センター線上

        print("yaw:{}, tilt:{}".format(yaw, tilt))


        diffX = gbl_gimbal_instruction_from_detection.relatedX_old - relatedX
        diffY = gbl_gimbal_instruction_from_detection.relatedY_old - relatedY

        gbl_gimbal_instruction_from_detection.relatedX_old = relatedX
        gbl_gimbal_instruction_from_detection.relatedY_old = relatedY          


        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 1,"action_type": 2,"action_value": str(yaw) +","+ str(tilt)}
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

        return yaw, tilt

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

    def gimbalControlXY(self, x, y):
        global gbl_gimbal_control
        global gbl_camera_control
        global gbl_gimbal_instruction_from_detection
        global gbl_instruction_from_click

        if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
            diagonal_fov = DEFINE.CAMERA_PARAMETER.fov
        else:
            diagonal_fov = 2 * math.degrees(math.atan(math.tan(math.radians(DEFINE.CAMERA_PARAMETER.fov / 2)) / gbl_gimbal_instruction_from_detection.zoomValueCurrent))

        destination = Coodinate(x=x,y=y)
        diagonal_pix =  math.sqrt(gbl_camera_control.width ** 2 + gbl_camera_control.height ** 2)
        horizontal_fov_wide = diagonal_fov * gbl_camera_control.width / diagonal_pix
        vertical_fov_wide = diagonal_fov * gbl_camera_control.height / diagonal_pix

        yaw = 0.0
        pitch = 0.0

        prev_pitch = gbl_gimbal_control.pitch
        prev_yaw = gbl_gimbal_control.yaw

        yaw = math.degrees(math.atan(2 * (destination.x - gbl_camera_control.width / 2) * math.tan(math.radians(horizontal_fov_wide / 2)) / gbl_camera_control.width))
        pitch = -1 * math.degrees(math.atan(2 * (destination.y - gbl_camera_control.height/2) * math.tan(math.radians(vertical_fov_wide / 2)) / gbl_camera_control.height))

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

        # print(f"yaw : {yaw} , pitch : {pitch}")

        # f = True
        # while f:
        #     diff_pitch = abs(abs(prev_pitch - gbl_gimbal_control.pitch) - abs(pitch))
        #     diff_yaw = abs(abs(prev_yaw - gbl_gimbal_control.yaw) - abs(yaw))
        #     # print(f"diff_yaw : {diff_yaw} , diff_pitch : {diff_pitch}")
        #     f = not ((diff_yaw < 1) and (diff_pitch < 1))
        #     time.sleep(0.01)

        return yaw, pitch




#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




    def gimbalControlYawTiltLargeAbsolute(self,yaw,tilt):
        j = json.dumps(
            {"type": "ctrl_drone","cmd": "gimbalctrl","target_type": 1,"action_type": 4,"action_value": str(yaw) +","+ str(tilt)}
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
        # global actionValueCurrent
        # actionValueCurrent = actionvalue
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




    def generate_sampling_group(self, input_prob, camera_height, camera_width, objectdetection_data):
        global gbl_data_from_telemetry
        try :
            shape = 640

            if len(objectdetection_data) <= 0:
                objectdetection_data.clear()
            
            print('=====result start=====')
            for det in input_prob:
                if det[0] == -1:
                    continue

                #クラス
                index = int(det[0])

                #識別率
                confidence = det[1]
                x1 = int(det[2] * camera_width / shape)
                y1 = int(det[3] * camera_height / shape)
                x2 = int(det[4] * camera_width / shape)
                y2 = int(det[5] * camera_height / shape)

                #中心算出
                detection_centerX = int((x1 + x2)/2)
                detection_centerY = int((y1 + y2)/2)

                print("DetectionCenter:", detection_centerX, " ", detection_centerY, " ", x1, " ", y1, " ", x2, " ", y2)

                #面積算出
                detection_square = int((x2 - x1) * (y2 - y1))

                #縦横の長さ算出
                boundingbox_width = int(x2 - x1)
                boundingbox_height = int(y2 -y1)

                #認識率判定
                if confidence > gbl_data_from_telemetry.confidence_bias :
                    print("type(det):{}".format(type(det)))
                    print("[class:{}], confidence:{:.1f} x1:{}, y1:{}, x2:{}, y2:{},boundingbox_width:{},boundingbox_height:{}".format(DEFINE.DISPLAY_ATTRIBUTES.classes[index], confidence, x1, y1, x2, y2, boundingbox_width, boundingbox_height))

                    #グループ化
                    print("objectdetection_data:{}".format(objectdetection_data))
                    if len(objectdetection_data) > 0 :
                        for classdata in objectdetection_data :
                            print("classdata:{}".format(classdata))
                            #クラス比較と中心でグループ化
                            if (int(classdata[0]) == index and abs(detection_centerX - classdata[2]) <= self.const_setting_value.group_bias and abs(detection_centerY - classdata[3]) <= self.const_setting_value.group_bias):
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
                    else :
                        print("==========新規===========")
                        objectdetection_data.append([index, confidence, detection_centerX, detection_centerY, boundingbox_width, boundingbox_height, detection_square, 1])
                        print("objectdetection_data: {}".format(objectdetection_data))
            print('=====result end=====')
            return objectdetection_data
        except Exception as e:
            print("generateSamplingGroup exception:{}".format(e))





#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




    def imshow_while_waiting(self, wait_time:int, cap, frameName:str, interval:int):
        counter_zoom = 0
        wait_zoom_tmp = wait_time
        _cap = cap
        time_zoom = dt.now() + datetime.timedelta(seconds=wait_zoom_tmp)
        print("start:{}".format(dt.now()))
        print("time_zooom:{}".format(time_zoom))
        while dt.now() < time_zoom:
            if counter_zoom % (interval/2) == 0:
                try:
                    _, frame = _cap.read()

                    label = "Remaining waiting time:" + str(time_zoom - dt.now())                                                   
                    frame = cv2.putText(frame,label,(0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),1, cv2.LINE_AA)
                    self.disp_frame = frame
                    # cv2.imshow(frameName, frame)

                    # qキーで終了
                    # if cv2.waitKey(1) & 0xFF == ord('a'):
                    #     break                            
                    time.sleep(0.01)                                
                except:
                    print("error VideoCapture")
                    pass
            counter_zoom += 1
        print("end:{}".format(dt.now()))









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


    def drawDisplayImage(self, img):
        #十字
        # 一致
        # print("line camera_width:{},camera_height:{}".format(gbl_camera_control.width, gbl_camera_control.height))
        cv2.line(img, (0, gbl_gimbal_instruction_from_detection.centerY), (gbl_camera_control.width, gbl_gimbal_instruction_from_detection.centerY), (175, 175, 175), thickness=1)
        cv2.line(img, (gbl_gimbal_instruction_from_detection.centerX, 0), (gbl_gimbal_instruction_from_detection.centerX, gbl_camera_control.height), (175, 175, 175), thickness=1)
        #ターゲット
        # 一致
        cv2.rectangle(img,(gbl_gimbal_instruction_from_detection.centerX - gbl_instruction_from_click.targetSquareWidth // 2, gbl_gimbal_instruction_from_detection.centerY - gbl_instruction_from_click.targetSquareHeight // 2),(gbl_gimbal_instruction_from_detection.centerX + gbl_instruction_from_click.targetSquareWidth // 2, gbl_gimbal_instruction_from_detection.centerY + gbl_instruction_from_click.targetSquareHeight // 2),(175,175,175),1)

        #モード表示
        label = gbl_data_from_telemetry.objectdetection_mode
        img = cv2.putText(img,label,(0, gbl_camera_control.height-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
        img = cv2.putText(img,label,(0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)    #test20220404

        #レンズモード表示
        label = gbl_instruction_from_click.currentCameraLens
        # print("label:{}".format(label)) #test20220330
        if label == DEFINE.CAMERALENS.dummy_pause : 
            img = cv2.putText(img,label,(0, gbl_camera_control.height-10-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
            img = cv2.putText(img,label,(0, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)
        else :
            img = cv2.putText(img,label,(0, gbl_camera_control.height-10-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            img = cv2.putText(img,label,(0, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

        #waypoint表示 
        # print("current_aircraft_waypoint:{}".format(gbl_data_from_telemetry.current_aircraft_waypoint))
        label = "WP:" + str(gbl_data_from_telemetry.current_aircraft_waypoint)
        img = cv2.putText(img,label,(0, gbl_camera_control.height-10-20-35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
        img = cv2.putText(img,label,(0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

        #検知時のパラメータ表示 
        label = "Model:" + self.const_setting_value.objectdetection_target_model + ", Mode:" + str(gbl_data_from_telemetry.objectdetection_tower_mode) + ", conf:" + str(gbl_data_from_telemetry.confidence_bias) + ", conf_show:" + str(self.const_setting_value.confidence_biases["show"]) + ", camera:" + str(gbl_instruction_from_click.currentCameraLens) + ", zoom:" + str(gbl_gimbal_instruction_from_detection.zoomValueCurrent)
        img = cv2.putText(img,label,(0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),1, cv2.LINE_AA)

        #左上の枠
        # 一致
        cv2.rectangle(img, (0, 0),(60,60),(175,175,175),1)

        #左下の枠
        cv2.rectangle(img,(0, gbl_camera_control.height - 60),(120,gbl_camera_control.height),(175,175,175),1)

        #右下の枠
        cv2.rectangle(img,(gbl_camera_control.width - 120, gbl_camera_control.height - 60),(gbl_camera_control.width, gbl_camera_control.height),(175,175,175),1)
        return img


    def analyzeProbGroup(self, prob):
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
            for i in range(len_prob) :
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
            b_synthesis = ((prob[max_y_indx][1] - a_synthesis*prob[max_y_indx][0]) + (prob[min_y_indx][1] - a_synthesis*prob[min_y_indx][0]))/2
            if a_synthesis < 0:
                # 傾きが負である時は右端から始める
                x_on_line = max_x
            else :
                # 傾きが正である時は左端から始める
                x_on_line = min_x

            y_on_line = a_synthesis * x_on_line + b_synthesis
            base_coordinate = Coodinate(x=x_on_line,y=y_on_line)

            return {"Error": False, "slopes": a_synthesis, "base_coordinate": base_coordinate}
    

    def extractSlope(self, slopes):
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
    

    def calculateSlopes(self, prob, base_indx):
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


    def validetatePoint(self , point:Coodinate):
        global gbl_camera_control
        if point.x < 0:
            point.x = 0
        if point.x > gbl_camera_control.width:
            point.x = gbl_camera_control.width
        if point.y < 0:
            point.y = 0
        if point.y > gbl_camera_control.height:
            point.y = gbl_camera_control.height
        return point


    def objectdetection(self):
        global gbl_gimbal_control
        global gbl_camera_control
        global gbl_data_from_telemetry
        global gbl_instruction_from_click
        global gbl_gimbal_instruction_from_detection
        global gbl_save_prev_data

        flg_objectdetection_auto_prev = False
        zoom_detect_process = False

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
        model = Model(DEFINE.DISPLAY_ATTRIBUTES.classes, DEFINE.DISPLAY_ATTRIBUTES.colors, height, width, DEFINE.PATH.model)


        #描画設定
        gbl_camera_control.width = int(width)     #カメラ幅
        gbl_camera_control.height = int(height)    #カメラ高さ
        print("camera_width:{},camera_height:{}".format(gbl_camera_control.width,gbl_camera_control.height))
        gbl_gimbal_instruction_from_detection.relatedX_old = -1
        gbl_gimbal_instruction_from_detection.relatedY_old = -1

        gbl_gimbal_instruction_from_detection.centerX = gbl_camera_control.width // 2   #中心(横)
        gbl_gimbal_instruction_from_detection.centerY = gbl_camera_control.height // 2  #中心(縦)
        gbl_gimbal_instruction_from_detection.gimbalCornerX = self.const_setting_value.maxCornerX
        gbl_gimbal_instruction_from_detection.gimbalCornerY = self.const_setting_value.maxCornerY

        #サンプリング設定
        generate_sampling_group_data = deque()

        # #vmware最大画面用
        # cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        # cv2.setWindowProperty('frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        # cv2.setMouseCallback('frame',self.onMouseClick)


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
                    #物体検知開始
                    print("▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼物体検知開始({}, INTERVAL:{}, sampling_count:{})▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼".format(dt.now(), self.const_setting_value.interval, self.const_setting_value.sampling_count))
                    # if gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom:
                    #         print("物体開始時ズーム初期化(2倍)")
                    #         self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                    #         print("物体開始時レンズ切替(ワイド)")
                    #         self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)                        
                    #         gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.wide
                    #         self.cameraControlFocus(2,0.5,0.5)
                    self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                    self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                    self.cameraControlFocus(2,0.5,0.5)
                    flg_objectdetection_auto_prev = True
                    sum_yaw = 0
                    sum_pitch = 0
                    continue 
                    

                if (counter % self.const_setting_value.interval != 0):
                    pass
                else :
                    counter = 0
                    # モード判定
                    # if gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_220kv:
                    #     if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                    #         print("物体開始時ズーム初期化(2倍)")
                    #         self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                    #         gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.zoom
                    #         #wide -> zoomに切り替え
                    #         print("物体開始時レンズ切替(ワイド→ズーム)")
                    #         self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.zoom)
                    #         print("########################wide->zoom wait {}s#######################".format(self.const_setting_value.wait_time["gimbal"]))    #test20220304
                    #         # #カメラ・ジンバル制御待機(画面描画)
                    #         self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["camera_change"]), cap=cap, frameName="frame", interval=self.const_setting_value.interval)   #add 20220414
                    #         # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                    #         self.gimbalcornerResetOnlyCorner()
                    #         counter += 1
                    #         continue
                    # elif gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom:
                    #     if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.zoom:
                    #         #ズームのとき
                    #         gbl_instruction_from_click.targetSquareWidth = 140
                    #         gbl_instruction_from_click.targetSquareHeight = 120     #test20220407
                    #         gbl_data_from_telemetry.confidence_bias = self.const_setting_value.confidence_biases["taicho"]

                    #     else :
                    #         #ワイドのとき
                    #         gbl_instruction_from_click.targetSquareWidth = 240
                    #         gbl_instruction_from_click.targetSquareHeight = 240

                    #         gbl_instruction_from_click.targetSquareWidth = 360
                    #         gbl_instruction_from_click.targetSquareHeight = 300

                    #         gbl_instruction_from_click.targetSquareWidth = 420
                    #         gbl_instruction_from_click.targetSquareHeight = 360
                    #                 #test20220407
                    #         gbl_data_from_telemetry.confidence_bias = self.const_setting_value.confidence_biases["taicho_wide_to_zoom"]
                    
                    threadhold = 0.0
                    if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                        threadhold = 0.3
                    else:
                        threadhold = 0.4
                    # 保存用
                    # save_image = frame.copy()
                    disp_frame, prob = model.inference(frame, threadhold)
                    time.sleep(1)
                    
                    result = self.analyzeProbGroup(prob)
                    if (result["Error"] == False):
                        tem_a = result["slopes"]
                        base_coordinate = result["base_coordinate"]

                        # start_pt = [int(base_coordinate["x"]), int(base_coordinate["y"])]
                        disp_frame = self.drawDisplayImage(disp_frame)
                        
                        if abs(tem_a) < 1:
                            start_pt = Coodinate(x=base_coordinate.x, y=base_coordinate.y)
                            start_pt = self.validetatePoint(start_pt)
                            if tem_a < 0 :
                                end_pt = Coodinate(x=start_pt.x - gbl_camera_control.width/3,y=start_pt.y - tem_a*gbl_camera_control.width/3)
                            else :
                                end_pt = Coodinate(x=start_pt.x + gbl_camera_control.width/3,y=start_pt.y + tem_a*gbl_camera_control.width/3)
                            end_pt = self.validetatePoint(end_pt)
                            cv2.line(disp_frame, (int(start_pt.x), int(start_pt.y)), (int(end_pt.x), int(end_pt.y)), (0, 0, 255), thickness=3)
                            yaw, pitch = self.gimbalControlXY(base_coordinate.x, base_coordinate.y)
                            sum_yaw += yaw    
                            sum_pitch += pitch
                            if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                                self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.zoom)
                                time.sleep(1)
                                self.cameraControlFocus(2,0.5,0.5)
                                time.sleep(1)
                            else:
                                if zoom_detect_process:
                                    self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_8)
                                    time.sleep(2)
                                    self.cameraControlFocus(2,0.5,0.5)
                                    time.sleep(2)
                                    yaw = 0
                                    pitch = 0
                                    i_count = 0
                                    move_x = gbl_camera_control.width/10
                                    while i_count < 5:
                                        i_count += 1
                                        start_pt = Coodinate(x=(gbl_camera_control.width / 2), y=(gbl_camera_control.height / 2))
                                        start_pt = self.validetatePoint(start_pt)
                                        if tem_a < 0 :
                                            end_pt = Coodinate(x=(start_pt.x - move_x), y=(start_pt.y - tem_a*move_x))
                                        else :
                                            end_pt = Coodinate(x=(start_pt.x + move_x), y=(start_pt.y + tem_a*move_x))
                                        yaw, pitch = self.gimbalControlXY(end_pt.x, end_pt.y)
                                        sum_yaw += yaw    
                                        sum_pitch += pitch
                                        _, frame = cap.read()
                                        disp_frame = self.drawDisplayImage(frame)
                                        self.disp_frame = np.copy(disp_frame)
                                    zoom_detect_process = False
                                    self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                                    time.sleep(2)
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
                        disp_frame = self.drawDisplayImage(disp_frame)

                    # # 画像表示
                    self.disp_frame = np.copy(disp_frame)
                    print("--------------------------------------------------------------------------------------------------------------")


                    #レンズがポーズのときは次に行く
                    if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.dummy_pause :
                        #サンプリング初期化
                        # generate_sampling_group_data.clear()   #サンプリングデータ初期化

                        counter += 1
                        continue

                    #物体検知サンプリング
                    # generate_sampling_group_data = self.generate_sampling_group(prob, gbl_camera_control.height, gbl_camera_control.width, generate_sampling_group_data)

                    if len(generate_sampling_group_data) > 0 :
                        flg_control_gimbal = False
                        max_detection_square = 0
                        min_detection_distance = width    #保留
                        max_confidence_bias = 0
                        target_index_max_detection_square = -1
                        target_index_min_detection_distance = -1
                        target_index_max_confidence_bias = 0

                        for i in range(len(generate_sampling_group_data)):
                            predet = generate_sampling_group_data[i]
                            #サンプリング判定用集計
                            if int(predet[0]) in self.const_setting_value.target_model_class_indexes:
                                if int(predet[7]) >= self.const_setting_value.sampling_count:
                                    flg_control_gimbal = True
                                else:
                                    pass
                            else:
                                continue

                            if max_detection_square < predet[6]:
                                #最大面積算出
                                max_detection_square = predet[6]  #最大面積
                                target_index_max_detection_square = i
                                print("max_detection_square:{}".format(max_detection_square))

                            tmp_detection_distance = np.linalg.norm(np.array([predet[2], predet[3]]) -  np.array([gbl_gimbal_instruction_from_detection.centerX, gbl_gimbal_instruction_from_detection.centerY]))
                            if min_detection_distance > np.linalg.norm(np.array([predet[2], predet[3]]) - np.array([gbl_gimbal_instruction_from_detection.centerX, gbl_gimbal_instruction_from_detection.centerY])):
                                #中心からの最小距離
                                min_detection_distance = np.linalg.norm(np.array([predet[2], predet[3]]) - np.array([gbl_gimbal_instruction_from_detection.centerX, gbl_gimbal_instruction_from_detection.centerY]))  #最小距離
                                target_index_min_detection_distance = i
                                print("min_detection_distance:{},numpy.array([predet[2], predet[3]]):{}".format(min_detection_distance,np.array([predet[2], predet[3]])))

                            if max_confidence_bias < predet[1]:
                                #最大認識率
                                max_confidence_bias = predet[1]  #最大認識率
                                target_index_max_confidence_bias = i
                                print("max_confidence_bias:{}".format(max_confidence_bias))

                        #集計結果
                        print("i:{},max_detection_square:{}".format(target_index_max_detection_square, max_detection_square))
                        print("i:{},min_detection_distance:{}".format(target_index_min_detection_distance, min_detection_distance))
                        print("i:{},max_confidence_bias:{}".format(target_index_max_confidence_bias, max_confidence_bias))

                        for i in range(len(generate_sampling_group_data)):
                            print("----------------------------", i, len(generate_sampling_group_data), "-------------------------")
                            det = generate_sampling_group_data[i]
                            target_detected_index = ""
                            if gbl_data_from_telemetry.objectdetection_mode == DEFINE.OBJECTDETECTION_MODE.default_center :           #中心優先
                                print("中心優先")
                                target_detected_index = target_index_min_detection_distance
                            elif  gbl_data_from_telemetry.objectdetection_mode == DEFINE.OBJECTDETECTION_MODE.default_area :          #面積優先
                                print("面積優先")
                                target_detected_index = target_index_max_detection_square
                            elif  gbl_data_from_telemetry.objectdetection_mode == DEFINE.OBJECTDETECTION_MODE.confidence :    #認識率優先
                                print("認識率優先")
                                target_detected_index = target_index_max_confidence_bias
                            else :                                          #面積優先
                                print("面積優先")
                                target_detected_index = target_index_max_detection_square
                
                            #物体検知
                            if (int(det[0]) in self.const_setting_value.target_model_class_indexes and i == target_detected_index and flg_control_gimbal == True):   #test20220310
                                print("det[2]:{}".format(det[2]))
                                print("centerX - targetSquareWidth // 2:{}".format(gbl_gimbal_instruction_from_detection.centerX - gbl_instruction_from_click.targetSquareWidth // 2))
                                print("centerX + targetSquareWidth // 2:{}".format(gbl_gimbal_instruction_from_detection.centerX + gbl_instruction_from_click.targetSquareWidth // 2))
                                print("det[3]:{}".format(det[3]))
                                print("centerY - targetSquareWidth // 2:{}".format(gbl_gimbal_instruction_from_detection.centerY - gbl_instruction_from_click.targetSquareHeight // 2))
                                print("centerY + targetSquareWidth // 2:{}".format(gbl_gimbal_instruction_from_detection.centerY + gbl_instruction_from_click.targetSquareHeight // 2))
                                #検知後の値を判断
                                tmpzoom = gbl_gimbal_instruction_from_detection.zoomValueCurrent // 2     #テストズーム時に中心判定緩和

                                #changestart 20220401
                                # #中心厳格判定(バウンディングボックスが中央の視覚より小さい場合は中心判定を厳しくする)
                                if det[4] <  gbl_instruction_from_click.targetSquareWidth and ((gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.zoom and gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom) or (self.const_setting_value.flg_target_strict_judgement == True)) :
                                    tmp_targetSquareWidth = gbl_instruction_from_click.targetSquareWidth // 2
                                else:
                                    tmp_targetSquareWidth = gbl_instruction_from_click.targetSquareWidth

                                if det[5] <  gbl_instruction_from_click.targetSquareHeight and ((gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.zoom and gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom) or (self.const_setting_value.flg_target_strict_judgement == True)) :
                                    tmp_targetSquareHeight = gbl_instruction_from_click.targetSquareHeight // 2
                                else:
                                    tmp_targetSquareHeight = gbl_instruction_from_click.targetSquareHeight

                                #中心判定
                                if ((det[2] > gbl_gimbal_instruction_from_detection.centerX - (tmp_targetSquareWidth // 2 * tmpzoom) and det[2] < gbl_gimbal_instruction_from_detection.centerX + (tmp_targetSquareWidth //2 * tmpzoom)  \
                                    and det[3] > gbl_gimbal_instruction_from_detection.centerY - (tmp_targetSquareHeight // 2 * tmpzoom) and det[3] < gbl_gimbal_instruction_from_detection.centerY + (tmp_targetSquareHeight // 2) * tmpzoom) or \
                                    (gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.kensui_500kv and gbl_gimbal_control.pitch > 28 and det[2] > gbl_gimbal_instruction_from_detection.centerX - (tmp_targetSquareWidth // 2 * tmpzoom) and det[2] < gbl_gimbal_instruction_from_detection.centerX + (tmp_targetSquareWidth //2 * tmpzoom)) \
                                    ): #test20211210 change #20220310 add

                                #changeend 20220401
                                    print("中心！")
                                    if ( gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide and gbl_data_from_telemetry.objectdetection_tower_mode != DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom):

                                        #サンプリング初期化
                                        # generate_sampling_group_data.clear()   #サンプリングデータ初期化

                                        #パラメータ初期化
                                        self.gimbalcornerReset()
                                        #ズーム初期値
                                        self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                        #レンズ切り替えまで他の処理はさせない
                                        gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.zoom
                                        #wide -> zoomに切り替え
                                        self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.zoom)
                                        print("########################wide->zoom wait {}s#######################".format(self.const_setting_value.wait_time["gimbal"]))    #test20220304
                                        #カメラ・ジンバル制御待機(画面描画)
                                        self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["camera_change"]),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220414

                                        counter += 1
                                        continue

                                        
                                    #ズーム後の高さおよび幅を確認
                                    if (det[4] >= gbl_camera_control.width * self.const_setting_value.zoom_bias or det[5] >= gbl_camera_control.height * self.const_setting_value.zoom_bias):
                                        #検知終了
                                        #パラメータ初期化
                                        self.gimbalcornerReset()
                                        #ズーム初期値
                                        self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                        #ジンバル復元
                                        print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved))
                                        self.gimbalControlYawTiltLargeAbsolute(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved)
                                        #zoom -> wideに切り替え
                                        self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                                        #カメラ・ジンバル制御待機(画面描画)
                                        self.imshow_while_waiting(wait_time=(7),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220414

                                        # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                        gbl_gimbal_control.mode_old = 2 #test 要確認 20211126
                                        flg_objectdetection_auto = False
                                        print("検知終了！！({})".format(gbl_data_from_telemetry.objectdetection_mode))

                                        #フライトプラン再開
                                        self.droneControlActionResume()
                                        

                                    else :
                                        cv2.circle(disp_frame, (int(det[2]), int(det[3])), 10, (0, 255, 255), thickness=-1) #add 20211014
                                        self.disp_frame = disp_frame
                                        # cv2.imshow('frame', disp_frame) # 1回目
                                        tdatetime = dt.now()
                                        date_str = tdatetime.strftime('%Y%m%d_%H%M%S')
                                        filename = "{}/{}.jpg".format(DEFINE.OUTPUT_PATH, date_str)
                                        cv2.imwrite(filename, frame)
                                        print("Saved. {}".format(filename))

                                        #物体検知のモード判定
                                        if gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_220kv:
                                            self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_12) #12倍
                                            self.cameraControlFocus(2,0.5,0.5)  #AFC、中心
                                            # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["zoom"] + 2),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220414

                                            #パラメータ初期化
                                            self.gimbalcornerReset()
                                            #ズーム初期値
                                            self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                            #ジンバル復元
                                            print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved))
                                            self.gimbalControlYawTiltLargeAbsolute(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved)
                                            #zoom -> wideに切り替え
                                            self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["gimbal"]),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220414

                                            # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                            flg_objectdetection_auto = False
                                            print("220kv耐張検知終了!!({})".format(gbl_data_from_telemetry.objectdetection_mode))

                                            #フライトプラン再開
                                            self.droneControlActionResume()
                                        elif gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.kensui_500kv:
                                            self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_12) #12倍
                                            self.cameraControlFocus(2,0.5,0.5)  #AFC、中心
                                            # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["zoom"] + 2),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220414

                                            #パラメータ初期化
                                            self.gimbalcornerReset()
                                            #ズーム初期値
                                            self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                            #ジンバル復元
                                            print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved))
                                            self.gimbalControlYawTiltLargeAbsolute(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved)
                                            #zoom -> wideに切り替え
                                            self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["gimbal"]),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220414

                                            # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                            gbl_data_from_telemetry.flg_objectdetection_auto = False
                                            print("500kv懸垂検知終了!!({})".format(gbl_data_from_telemetry.objectdetection_mode))

                                            #フライトプラン再開
                                            self.droneControlActionResume()

                                        elif gbl_data_from_telemetry.objectdetection_tower_mode == DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_wide_to_zoom:
                                            if gbl_instruction_from_click.currentCameraLens == DEFINE.CAMERALENS.wide:
                                                #ジンバル制御
                                                print("ジンバル制御！({})".format(gbl_data_from_telemetry.objectdetection_mode))
                                                cv2.circle(disp_frame, (int(det[2]), int(det[3])),10, (0, 255, 255), thickness=-1) #add 20211014
                                                cv2.imshow('frame', disp_frame) # 2回目
                                                print("cornerbias:{},gimbalCornerX:{},gimbalCornerY:{}".format(self.const_setting_value.cornerbias,gbl_gimbal_instruction_from_detection.gimbalCornerX,gbl_gimbal_instruction_from_detection.gimbalCornerY))  #test20220304
                                                tdatetime = dt.now()
                                                date_str = tdatetime.strftime('%Y%m%d_%H%M%S')
                                                filename = "{}/{}.jpg".format(DEFINE.OUTPUT_PATH, date_str)
                                                cv2.imwrite(filename, frame)
                                                print("Saved. {}".format(filename))

                                                self.gimbalControlYawTiltLarge(det[2], det[3])

                                                #wide -> zoomに切り替え
                                                gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.zoom
                                                print("ズームモード時ズーム初期化(2倍)")
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                                # #カメラ・ジンバル制御待機(画面描画)
                                                self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["zoom"]),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220401
                                                self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.zoom)
                                                
                                                # generate_sampling_group_data.clear()   #サンプリングデータ初期化

                                                gbl_gimbal_instruction_from_detection.gimbalCornerX = self.const_setting_value.maxCornerX
                                                gbl_gimbal_instruction_from_detection.gimbalCornerY = self.const_setting_value.maxCornerY
                                                continue  

                                            else:
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_12) #12倍？ #107
                                                self.cameraControlFocus(2, 0.5, 0.5)  #AFC、中心
                                                # generate_sampling_group_data.clear()   #サンプリングデータ初期化

                                                #カメラ・ジンバル制御待機(画面描画)
                                                self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["gimbal"] + 2),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220331

                                                #パラメータ初期化
                                                self.gimbalcornerReset()
                                                #ズーム初期値
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                                #ジンバル復元
                                                print("gimbal_yaw_saved:{}, gimbal_pitch_saved:{}".format(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved))
                                                self.gimbalControlYawTiltLargeAbsolute(gbl_gimbal_control.yaw_saved, gbl_gimbal_control.pitch_saved)
                                                #zoom -> wideに切り替え
                                                gbl_instruction_from_click.currentCameraLens = DEFINE.CAMERALENS.wide
                                                self.cameraControlChangeLens(DEFINE.OSDK_CAMERA_SOURCE_H20T.wide)

                                                #カメラ・ジンバル制御待機(画面描画)
                                                self.imshow_while_waiting(wait_time= self.const_setting.wait_time["gimbal"],cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220331

                                                # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                                gbl_data_from_telemetry.flg_objectdetection_auto = False
                                                flg_objectdetection_auto_prev = False  #test20220404
                                                print("耐張(wide)検知終了！！({})".format(gbl_data_from_telemetry.objectdetection_mode))
                                                #物体検知終了
                                                print("▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲物体検知終了({},INTERVAL:{},sampling_count:{})▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲".format(dt.now(),self.const_setting_value.interval,self.const_setting_value.sampling_count))

                                                #フライトプラン再開
                                                self.droneControlActionResume()
                                        else: 
                                            #物体検知モード指定以外の場合, 中心に近い場合はジンバル制御を停止しズームする
                                            if (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_2):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_4):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_4)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_5):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_5)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_6):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_6)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_8):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_8)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_10):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_10)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_12):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_12)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_16):
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_16)
                                            elif (gbl_gimbal_instruction_from_detection.zoomValueCurrent < DEFINE.ZOOM_VALUE.Val_20):    #add 20220330
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_20)   #add 20220330
                                            else :
                                                self.cameraControlZoom(DEFINE.ZOOM_VALUE.Val_2)
                                            #ズーム後フォーカス処理
                                            self.cameraControlFocus(2,0.5,0.5)  #AFC、中心
                                            # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                            #カメラ・ジンバル制御待機(画面描画)
                                            self.imshow_while_waiting(wait_time=(self.const_setting_value.wait_time["zoom"]),cap=cap,frameName="frame", interval=self.const_setting_value.interval)   #add 20220414

                    
                                else :
                                    #ジンバル制御
                                    print("ジンバル制御！({})".format(gbl_data_from_telemetry.objectdetection_mode))
                                    cv2.circle(disp_frame, (int(det[2]), int(det[3])), 10, (0, 255, 255), thickness=-1) #add 20211014
                                    cv2.imshow('frame', disp_frame) # 3回目
                                    print("cornerbias:{},gimbalCornerX:{},gimbalCornerY:{}".format(self.const_setting_value.cornerbias,gbl_gimbal_instruction_from_detection.gimbalCornerX,gbl_gimbal_instruction_from_detection.gimbalCornerY))  #test20220304

                                    tdatetime = dt.now()
                                    date_str = tdatetime.strftime('%Y%m%d_%H%M%S')
                                    filename = "{}/{}.jpg".format(DEFINE.OUTPUT_PATH, date_str)
                                    cv2.imwrite(filename, frame)
                                    print("Saved. {}".format(filename))

                                    yaw, tilt = self.gimbalControlYawTiltLarge(det[2], det[3])
                                    # generate_sampling_group_data.clear()   #サンプリングデータ初期化
                                    tmp_wait_gimbal = self.const_setting_value.wait_time["gimbal"]
                                    if self.const_setting_value.flg_gimbal_variable_waiting_time == True:
                                        #ジンバル制御時の待ち時間自動調整
                                        tmp_wait_gimbal = 4
                                        if abs(yaw) >= 60 or abs(tilt) >= abs(60):
                                            tmp_wait_gimbal = 4
                                        elif abs(yaw) > abs(tilt) and 60 > abs(yaw):
                                            tmp_wait_gimbal = 2 *  abs(yaw) / 60 + 2
                                        elif abs(tilt) > abs(yaw) and 60 > abs(tilt):
                                            tmp_wait_gimbal = 2 *  abs(tilt) / 60 + 2
                                    print("ジンバル制御waittime:{}".format(tmp_wait_gimbal))
                                    rospy.sleep(tmp_wait_gimbal)
            else :
                #サンプリング初期化
                # generate_sampling_group_data.clear()   #サンプリングデータ初期化

                disp_frame = frame
                #各種描画
                #十字
                # 一致
                cv2.line(disp_frame, (0, gbl_gimbal_instruction_from_detection.centerY), (gbl_camera_control.width, gbl_gimbal_instruction_from_detection.centerY), (175, 175, 175), thickness=1)
                cv2.line(disp_frame, (gbl_gimbal_instruction_from_detection.centerX, 0), (gbl_gimbal_instruction_from_detection.centerX, gbl_camera_control.height), (175, 175, 175), thickness=1)
                #ターゲット
                # 一致
                cv2.rectangle(disp_frame,(gbl_gimbal_instruction_from_detection.centerX - gbl_instruction_from_click.targetSquareWidth // 2, gbl_gimbal_instruction_from_detection.centerY - gbl_instruction_from_click.targetSquareHeight // 2),(gbl_gimbal_instruction_from_detection.centerX + gbl_instruction_from_click.targetSquareWidth // 2, gbl_gimbal_instruction_from_detection.centerY + gbl_instruction_from_click.targetSquareHeight // 2),(175,175,175),1)

                #左上の枠
                # 一致
                cv2.rectangle(disp_frame,(0, 0),(60,60),(175,175,175),1)

                rectangl_width = 60
                rectangl_height = 40 

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

                ### TODO 実験
                # for test in ZoomValues:
                #     cv2.rectangle(disp_frame,(0, currnet_start_y),(rectangl_width,currnet_start_y + rectangl_height),(175,175,175),1)
                #     label = "x" + str(test)
                #     currnet_start_y += rectangl_height
                #     disp_frame = cv2.putText(disp_frame,label,(0, currnet_start_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

                cv2.rectangle(disp_frame,(0, currnet_start_y),(rectangl_width,currnet_start_y + rectangl_height),(175,175,175),1)
                label = "x" + str(gbl_gimbal_instruction_from_detection.zoomValueCurrent)
                currnet_start_y += rectangl_height
                disp_frame = cv2.putText(disp_frame,label,(0, currnet_start_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
                    

                #左下の枠
                cv2.rectangle(disp_frame,(0, gbl_camera_control.height - 60),(120, gbl_camera_control.height),(175,175,175),1)

                #右下の枠
                cv2.rectangle(disp_frame,(gbl_camera_control.width - 120, gbl_camera_control.height - 60),(gbl_camera_control.width, gbl_camera_control.height),(175,175,175),1)

                # #waypoint表示 
                label = "WP:" + str(gbl_data_from_telemetry.current_aircraft_waypoint)
                disp_frame = cv2.putText(disp_frame,label,(0, gbl_camera_control.height-10-20-35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
                disp_frame = cv2.putText(disp_frame,label,(0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)    #test20220404


                # # 画像表示
                # cv2.imshow('frame', disp_frame)
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
