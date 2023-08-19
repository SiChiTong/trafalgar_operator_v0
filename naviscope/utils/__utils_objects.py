from enum import Enum

class AVAILABLE_LANG( str, Enum ):
    FR = "fr"

class AVAILABLE_TOPICS( str, Enum ):
    VELOCITY = "cmd_vel"
    PANTILT = "cmd_cam"
    NAVTARGET = "cmd_navtarget"
    SHUTDOWN = "cmd_shutdown"
    STREAM = "videostream"
    IMU = "imu_sensor",
    HEARTBEAT = "heartbeat"
    WATCHDOG = "watchdog"
    SENSOR = "sensor"
    JOYSTICK = "joy"
    GAMEPLAY = "gameplay"


class SENSORS_TOPICS( str, Enum ):
    IP = "ip"
    WIFI = "rssi"
    OFF_AREA = "isOutsideOfArea"
    INDEX = "index"
    DATAS = "datas"
    BATTERY_GAUGE = "gauge"
    BATTERY_VOLTAGE = "voltage"
    ORIENTATION = "orientation"
    PROPULSION = "propulsion"
    DIRECTION = "direction"
    THRUST = "thrust"
    STEERING = "steer"
    LAT = "latitude"
    LON = "longitude"
    SAT = "satellites"
    AZI = "azimuth"
    SPEED = "speed"
    PITCH = "pitch"
    ROLL = "roll"
    YAW = "yaw"
    DELTA_PITCH = "delta_pitch"
    DELTA_ROLL = "delta_pitch"
    DELTA_YAW = "delta_pitch"
    OBSTACLE_DISTANCE = "obstacle_distance"
    OBSTACLE_ANGLE = "obstacle_angle"
    CAM_PAN = "pan"
    CAM_TILT = "tilt"
    SHORT_PRESS = "shortPress"
    LONG_PRESS = "longPress"
    TEMPERATURE = "temperature"


    

class DISPATCH_TOPICS( str, Enum ):
    CMD_DIRECTION = "cmd_dir"
    CMD_PROPULSION = "cmd_prop"
    CMD_STEERING = "cmd_steer"
    CMD_CAM_PAN = "cmd_pan"
    CMD_CAM_TILT = "cmd_tilt"
    CMD_PILOT = "cmd_pilot"
    CMD_NAVTARGET = "cmd_target"
    ENA_RANGE = "ena_lidar"
    STAT_BATTERY = "stat_bat"
    STAT_PROP = "stat_prop"
    STAT_GPS = "stat_gps"
    STAT_IMU = "stat_imu"
    STAT_CAM = "stat_cam"
    STAT_RANGE = "stat_range"


BOARD_SENSORS_DATAS = {

    SENSORS_TOPICS.IP.value : "",
    SENSORS_TOPICS.WIFI.value : 0,
    SENSORS_TOPICS.OFF_AREA.value : "",
    SENSORS_TOPICS.BATTERY_GAUGE.value : 0,
    SENSORS_TOPICS.BATTERY_VOLTAGE.value : 0,
    SENSORS_TOPICS.DIRECTION.value : 0,
    SENSORS_TOPICS.THRUST.value : 0,
    SENSORS_TOPICS.STEERING.value  : 0,
    SENSORS_TOPICS.LAT.value  : 0,
    SENSORS_TOPICS.LON.value  : 0,
    SENSORS_TOPICS.SAT.value  : 0,
    SENSORS_TOPICS.AZI.value : 0,
    SENSORS_TOPICS.SPEED.value  : 0,
    SENSORS_TOPICS.PITCH.value  : 0,
    SENSORS_TOPICS.ROLL.value  : 0,
    SENSORS_TOPICS.YAW.value  : 0,
    SENSORS_TOPICS.TEMPERATURE.value  : 0,
    SENSORS_TOPICS.OBSTACLE_DISTANCE.value  : 0,
    SENSORS_TOPICS.OBSTACLE_ANGLE.value  : 0,
    SENSORS_TOPICS.CAM_PAN.value : 0,
    SENSORS_TOPICS.CAM_TILT.value  : 0

}



class PEER(str, Enum):
    MASTER = "master"
    USER = "user"
    DRONE = "drone"
    XR = "xr"


class MAP_MARKERS( str, Enum ):
    NAVIGATION = "navigationMarker"
    AREA = "gameAreaMarker"

    
class EXIT_STATE(str, Enum ):
    ALIVE = "alive"
    SHUTDOWN = "shutdown"
    RESTART = "restart"

class DIRECTION_STATE(Enum):
    BACKWARD = -1
    STOP = 0
    FORWARD = 1


DIRECTION_STP = ("IDDLE", "#868686")
DIRECTION_FWD = ("FORWARD", "#028400")
DIRECTION_BWD = ("BACKWARD", "#CB4D00")

WIFI_INTERFACE = "wl"



DRONES_LIST = {
    
    "Aucun" : {
        "index" : -1,
        "color" : ( "#000000", "#ffffff" )
    },
    "Bounty" : {
        "index" : 0,
        "color" : ( "#E3735E", "#A6A6A6" )
    },
    "Arjeroc" : {
        "index" : 1,
        "color" : ( "#1967FF", "#4B4B4B" )
    },
    "Daisy Jack" : {
        "index" : 2,
        "color" : ( "#197600", "#DADADA" )
    },
    "Lady Idosia" : {
        "index" : 3,
        "color" : ( "#DFB40C", "#0C83DF" )
    },
    "Rei Pelluci" : {
        "index" : 4,
        "color" : ( "#E21E1E", "#FFAA19" )
    }

}

