BAUDRATE = 1000000
PORT_LEADER = '/dev/ttyACM0'
PORT_FOLLOWER = '/dev/ttyACM1'
PROTOCOL_END = 0

RATE = 100

CAMERAS = [
    {
        'name': 'frontal_camera',
        'width': 640,
        'height': 480,
        'fps': 30,
        'camera_id': 0,
    },
    # {
    #     'name': 'camera_2',
    #     'width': 640,
    #     'height': 480,
    #     'fps': 30,
    #     'camera_id': 1,
    # },
]
