import define_value as DEFINE

class DataFromTelemetry():
    def __init__(self):
        self.flg_objectdetection_auto = False
        self.objectdetection_mode = DEFINE.OBJECTDETECTION_MODE.default_center
        self.current_aircraft_waypoint = -1
        self.confidence_bias = 0
        self.objectdetection_tower_mode = DEFINE.OBJECTDETECTION_TOWER_MODE.taicho_220kv