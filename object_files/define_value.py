from enum import Enum

class OBJECTDETECTION_TARGET_MODEL:
    resnet50 = "resnet50"      #resnet50_ssd_model
    insulator = "insulator"    #insulator(がいし)
    def __setattr__(self, name, value):
        pass

class ZOOM_VALUE:
    Val_2  = 2
    Val_4  = 4
    Val_5  = 5
    Val_6  = 6
    Val_8  = 8
    Val_10 = 10
    Val_12 = 12
    Val_16 = 16
    Val_20 = 20
    def __setattr__(self, name, value):
        pass

class INPUT_MODE_GSTREAMER:
    udp = "gstreamer_udp"  #GstreamerでUDP入力(Matrice300のカメラから取得)
    udp_str = 'udpsrc port=50002 buffer-size=2129920 caps=\"application/x-rtp,payload=\(int\)96,encoding-name=\(string\)H264,width=1280,height=720\"  ! rtph264depay ! avdec_h264 !  videoscale ! video/x-raw,width=1280,height=720 ! videoconvert ! appsink sync=false' #20220328 jishou OK
    uri = "gstreamer_uri"  #Gstreamerでストリーミングサーバの映像を取得
    uri_str = 'souphttpsrc location=http://192.168.10.185:8090/test.mjpg !  jpegdec ! videoconvert ! appsink sync=false' #another server 
    def __setattr__(self, name, value):
        pass

class DISPLAY_ATTRIBUTES:
    # insulator用
    classes = ['ins_t', 'ins_t_b'] #ONNX
    colors = [(0,0,175), (175,0,0)] #ONNX
    def __setattr__(self, name, value):
        pass

class PATH:
    model = '/home/terra/models/onnx/ins_t1026.onnx' #Onnx
    output = '/home/terra/screenshot/output' # 画像を出力するフォルダ
    def __setattr__(self, name, value):
        pass

class OBJECTDETECTION_TOWER_MODE(Enum):
    taicho = 1
    kensui = 2

class OBJECTDETECTION_STATUS:
    start =  "objectdetection_start"
    no_action = "no_action"
    def __setattr__(self, name, value):
        pass

class GIMBAL_MODE:
    free = 0
    follow = 2
    def __setattr__(self, name, value):
        pass

class MISSION_STATUS:
    pause = 4
    flying = 3
    def __setattr__(self, name, value):
        pass

class CAMERALENS:
    wide = "wide"
    zoom = "zoom"
    dummy_pause = "pause"
    def __setattr__(self, name, value):
        pass

class OSDK_CAMERA_SOURCE_H20T:
    wide = 1
    zoom = 2
    ir = 3
    def __setattr__(self, name, value):
        pass

class CAMERA_PARAMETER:
    fov = 70
    def __setattr__(self, name, value):
        pass