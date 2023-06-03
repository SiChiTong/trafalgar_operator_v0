from enum import Enum

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
    BATTERY_GAUGE = "gauge"
    BATTERY_VOLTAGE = "voltage"
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
    OBSTACLE = "obstacle"
    CAM_PAN = "pan"
    CAM_TILT = "tilt"

class PEER(str, Enum):
    MASTER = "master"
    USER = "user"
    DRONE = "drone"
    
class EXIT_STATE(str, Enum ):
    SHUTDOWN = "shutdown"
    RESTART = "restart"

DIRECTION_STP = ("EN STANDBY", "#868686")
DIRECTION_FWD = ("MARCHE AVANT", "#028400")
DIRECTION_BWD = ("MARCHE ARRIERE", "#CB4D00")