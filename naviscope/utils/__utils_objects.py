from enum import Enum

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
    SAT = "sat"
    AZI = "azimuth"
    SPEED = "speed"
    PITCH = "pitch"
    ROLL = "roll"
    YAW = "yaw"
    DELTA_PITCH = "delta_pitch"
    DELTA_ROLL = "delta_pitch"
    DELTA_YAW = "delta_pitch"
    OBSTACLE = "obstacle"
    CAM_PAN = "pan"
    CAM_TILT = "tilt"
    SHORT_PRESS = "shortPress"
    LONG_PRESS = "longPress"
    TEMPERATURE = "temperature"


class DISPATCH_TOPICS( str, Enum ):
    DIRECTION = "cmd_dir"
    PROPULSION = "cmd_prop"
    STEERING = "cmd_steer"
    CAM_TILT = "cmd_tilt"
    CAM_PAN = "cmd_pan"
    STATUS = "cmd_status"
    NAVTARGET = "cmd_target"


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

DRONES_NAMES = [
    ("Aucun", ( "#000000", "#ffffff" )), 
    ("Bounty", ( "#E3735E", "#A6A6A6")),
    ("Arjeroc", ( "#1967FF", "#4B4B4B" )),    
    ("Daisy Jack", ( "#197600", "#DADADA" )),
    ("Lady Idosia", ( "#DFB40C", "#0C83DF" )),
    ( "Rei Pelluci", ( "#E21E1E", "#FFAA19" ))
]