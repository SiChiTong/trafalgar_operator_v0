from enum import Enum

class AVAILABLE_TOPICS( str, Enum ):
    PROPULSION = "propulsion"
    DIRECTION = "direction"
    ORIENTATION = "orientation"
    STREAM = "videostream"
    IMU = "imu",
    PANTILT = "pantilt"
    HEARTBEAT = "heartbeat"
    WATCHDOG = "watchdog"
    SENSOR = "sensor"
    SHUTDOWN = "shutdown"


class OPERATOR(str, Enum):
    MASTER = "master"
    USER = "user"

class EXIT_STATE(str, Enum ):
    ALIVE = "alive"
    SHUTDOWN = "shutdown"
    RESTART = "restart"