from enum import IntEnum

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
