import object_files.define_value as DEFINE

class DataFromTelemetry():
    flg_objectdetection_auto = False
    current_aircraft_waypoint = -1
    confidence_bias = 0
    objectdetection_tower_mode = DEFINE.OBJECTDETECTION_TOWER_MODE.taicho