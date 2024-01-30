class INTERVAL:
    version_202203 = 8
    version_202204 = 4
    def __setattr__(self, name, value):
        pass

class OBJECTDETECTION_TARGET_MODEL:
    resnet50 = "resnet50"      #resnet50_ssd_model
    insulator = "insulator"    #insulator(がいし)
    def __setattr__(self, name, value):
        pass

class CLASS_INSULATOR:
    t_660 = 0    #0に66kvの耐張が入っている前提
    t_220 = 1
    k_220 = 2
    t_500 = 3
    k_500 = 4
    t_ = 5       #5に220kv、500kvの耐張が入っている前提
    k_ = 6       #6に220kv、500kvの懸垂が張っている前提
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

class OBJECTDETECTION_MODE:
    default_area = "area"
    default_center = "center"
    confidence = "confidence"
    def __setattr__(self, name, value):
        pass

class OBJECTDETECTION_TOWER_MODE:
    taicho_220kv = "taicho_220kv"
    kensui_500kv = "kensui_500kv"
    taicho_wide_to_zoom = "taicho_wide_to_zoom"  #起動時のデフォルト設定(DJI Onobard SDKから来るテレメトリで変わることがある)
    def __setattr__(self, name, value):
        pass

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