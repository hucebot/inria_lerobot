BAUDRATE = 1000000
PORT_LEADER = '/dev/ttyACM1'
PORT_FOLLOWER = '/dev/ttyACM0'
PROTOCOL_END = 0

RATE = 100

CAMERAS = [
    {
        'name': 'frontal_camera',
        'width': 640,
        'height': 480,
        'fps': 30,
        'camera_id': 8,
    },
    # {
    #     'name': 'camera_2',
    #     'width': 640,
    #     'height': 480,
    #     'fps': 30,
    #     'camera_id': 1,
    # },
]
