
class ConnectionToDroneException(Exception):
    def __init__(self):
        super().__init__('Drone is not connected!')