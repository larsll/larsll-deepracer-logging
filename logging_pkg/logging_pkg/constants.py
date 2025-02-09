from enum import IntEnum, Enum

#########################################################################################
# USBFileSystem services.

USB_FILE_SYSTEM_NOTIFICATION_TOPIC = "/usb_monitor_pkg/usb_file_system_notification"
USB_FILE_SYSTEM_SUBSCRIBE_SERVICE_NAME = "/usb_monitor_pkg/usb_file_system_subscribe"
USB_MOUNT_POINT_MANAGER_SERVICE_NAME = "/usb_monitor_pkg/usb_mount_point_manager"

# Output 
LOGS_DEFAULT_FOLDER = "/opt/aws/deepracer/logs"
LOGS_SOURCE_LEAF_DIRECTORY = "logs"
LOGS_DIR_CB = "logs_dir_cb"
LOGS_BAG_FOLDER_NAME_PATTERN = "{}-{}"

# Pause after Force stop
PAUSE_AFTER_FORCE_STOP = 10.0

# Naming
DEFAULT_BAG_NAME = "deepracer"

class RecordingState(IntEnum):
    """ Color to RGB mapping
    Extends:
        Enum
    """
    Stopped = 0
    Running = 1
    Stopping = 2


class NodeState(IntEnum):
    """ Status of node
    Extends:
        Enum
    """
    Starting = 0
    Scanning = 1
    Running = 2
    Error = 3


class LoggingMode(Enum):
    """ Status of node
    Extends:
        Enum
    """
    Never = 0
    USBOnly = 1
    Always = 2

    @classmethod
    def _missing_(cls, name_):
        for member in cls:
            if member.name.lower() == name_.lower():
                return member
