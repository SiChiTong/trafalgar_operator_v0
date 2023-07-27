from enum import Enum

#change direction / orientation / propulsion to cmd_vel -> twist msg
#change pantilt to cmd_cam

class AVAILABLE_TOPICS( str, Enum ):
    PROPULSION = "propulsion"
    DIRECTION = "direction"
    ORIENTATION = "orientation"
    STEER_ANGLE = "steerAngle"
    STEER_INCREMENT = "steerIncrement"
    STREAM = "videostream"
    IMU = "imu",
    PANTILT = "pantilt"
    NAVTARGET = "navtarget"
    HEARTBEAT = "heartbeat"
    WATCHDOG = "watchdog"
    SENSOR = "sensor"
    SHUTDOWN = "shutdown"
    BUZZER = "buzzer"
    JOYSTICK = "joy"
    GAMEPLAY = "gameplay"

class SENSORS_TOPICS( str, Enum ):
    IP = "ip"
    WIFI = "rssi"
    BATTERY_GAUGE = "gauge"
    BATTERY_VOLTAGE = "voltage"
    ORIENTATION = "orientation"
    PROPULSION = "propulsion"
    DIRECTION = "direction"
    THRUST = "thrust"
    STEERING = "steer"
    LAT = "latitude"
    LON = "longitude"
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


class PEER(str, Enum):
    MASTER = "master"
    USER = "user"
    DRONE = "drone"
    XR = "xr"
    
class EXIT_STATE(str, Enum ):
    SHUTDOWN = "shutdown"
    RESTART = "restart"

DIRECTION_STP = ("EN STANDBY", "#868686")
DIRECTION_FWD = ("MARCHE AVANT", "#028400")
DIRECTION_BWD = ("MARCHE ARRIERE", "#CB4D00")


DRONES_NAMES = [
    ("Aucun", ( "#000000", "#ffffff" )), 
    ("Bounty", ("#A6A6A6", "#EA8E38" )),
    ("Daisy Jack", ( "#197600", "#DADADA" )),
    ("Arjeroc", ("#1967FF", "#4B4B4B" )),
    ("Lady Idosia", ( "#DFB40C", "#0C83DF" )),
    ( "Rei Pelluci", ( "#FC431B", "#FFAA19" ))
]