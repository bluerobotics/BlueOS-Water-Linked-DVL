from blueoshelper import request, post
import requests
import json
import time
from math import radians

MAVLINK2REST_URL = "http://127.0.0.1/mavlink2rest"

# holds the last status so we dont flood it
last_status = ""


class Mavlink2RestHelper:
    """
    Responsible for interfacing with Mavlink2Rest
    """

    def __init__(self, vehicle: int=1, component: int=1):
        # store vehicle and component to access telemetry data from
        self.vehicle = vehicle
        self.component = component
        # store vision template data so we don't need to fetch it multiple times
        self.start_time = time.time()
        self.vision_template = """
{{
  "header": {{
    "system_id": 255,
    "component_id": 0,
    "sequence": 0
  }},
  "message": {{
    "type": "VISION_POSITION_DELTA",
    "time_usec": 0,
    "time_delta_usec": {dt},
    "angle_delta": [
      {dRoll},
      {dPitch},
      {dYaw}
    ],
    "position_delta": [
      {dx},
      {dy},
      {dz}
    ],
    "confidence": {confidence}
  }}
}}"""

        self.vision_speed_estimate_template = """
        {{
  "header": {{
    "system_id": 255,
    "component_id": 0,
    "sequence": 0
  }},
  "message": {{
    "type": "VISION_SPEED_ESTIMATE",
    "usec": {us},
    "x": {vx},
    "y": {vy},
    "z": {vz},
    "covariance": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "reset_counter": 0
  }}
}}"""

        self.vision_position_estimate_template = """
{{
  "header": {{
    "system_id": 255,
    "component_id": 0,
    "sequence": 0
  }},
  "message": {{
    "type": "VISION_POSITION_ESTIMATE",
    "usec": {us},
    "x": {x},
    "y": {y},
    "z": {z},
    "roll": {roll},
    "pitch": {pitch},
    "yaw": {yaw},
    "covariance": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "reset_counter": 0
  }}
}}"""

        self.gps_origin_template = """
{{
  "header": {{
    "system_id": 255,
    "component_id": 0,
    "sequence": 0
  }},
  "message": {{
    "type": "SET_GPS_GLOBAL_ORIGIN",
    "latitude": {lat},
    "longitude": {lon},
    "altitude": 0,
    "target_system": 0,
    "time_usec": 0
  }}
}}
        """

        self.rangefinder_template = """
{{
  "header": {{
    "system_id": 255,
    "component_id": 0,
    "sequence": 0
  }},
  "message": {{
    "type": "DISTANCE_SENSOR",
    "time_boot_ms": 0,
    "min_distance": 0,
    "max_distance": 5000,
    "current_distance": {0},
    "mavtype": {{
      "type": "MAV_DISTANCE_SENSOR_LASER"
    }},
    "id": 0,
    "orientation": {{
      "type": "MAV_SENSOR_ROTATION_PITCH_270"
    }},
    "covariance": 0,
    "horizontal_fov": 0.0,
    "vertical_fov": 0.0,
    "quaternion": [
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "signal_quality": 0
  }}
}}
"""

    def get_float(self, path: str, vehicle: int=None, component: int=None) -> float:
        """
        Helper to get mavlink data from mavlink2rest.
        Uses initialised vehicle and component as defaults, unless overridden.
        Example: get_float('/VFR_HUD')
        Returns the data as a float (nan on failure)
        """
        response = self.get(path, vehicle, component)
        if not response:
            return float("nan")
        return float(response)

    def get(self, path: str, vehicle: int=None, component: int=None) -> str:
        """
        Helper to get mavlink data from mavlink2rest
        Uses initialised vehicle and component as defaults, unless overridden.
        Example: get('/VFR_HUD')
        Returns the data as text or False on failure
        """
        vehicle = vehicle or self.vehicle
        component = component or self.component
        vehicle_path = f"/vehicles/{vehicle}/components/{component}/messages"
        response = request(MAVLINK2REST_URL + '/mavlink' + vehicle_path + path)
        if not response:
            return False
        return response

    def get_message_frequency(self, message_name):
        """
        Returns the frequency at which message "message_name" is being received, 0 if unavailable
        """
        try:
          return self.get_float('/{0}/message_information/frequency'.format(message_name))
        except ValueError:
          return 0

    # TODO: Find a way to run this check for every message received without overhead
    # check https://github.com/patrickelectric/mavlink2rest/issues/9

    def ensure_message_frequency(self, message_name, msg_id, frequency):
        """
        Makes sure that a mavlink message is being received at least at "frequency" Hertz
        Returns true if successful, false otherwise
        """
        message_name = message_name.upper()
        current_frequency = self.get_message_frequency(message_name)

        # load message template from mavlink2rest helper
        try:
            data = json.loads(requests.get(
                MAVLINK2REST_URL + '/helper/mavlink?name=COMMAND_LONG').text)
        except:
            return False

        # msg_id = getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + message_name)
        data["message"]["command"] = {"type": 'MAV_CMD_SET_MESSAGE_INTERVAL'}
        data["message"]["param1"] = msg_id
        data["message"]["param2"] = int(1000000/frequency)

        try:
            result = requests.post(MAVLINK2REST_URL + '/mavlink', json=data)
            return result.status_code == 200
        except Exception as error:
            report_status("Error setting message frequency: " + str(error))
            return False

    def set_param(self, param_name, param_type, param_value):
        """
        Sets parameter "param_name" of type param_type to value "value" in the autpilot
        Returns True if succesful, False otherwise
        """
        try:
            data = json.loads(requests.get(
                MAVLINK2REST_URL + '/helper/mavlink?name=PARAM_SET').text)

            for i, char in enumerate(param_name):
                data["message"]["param_id"][i] = char

            data["message"]["param_type"] = {"type": param_type}
            data["message"]["param_value"] = param_value

            result = requests.post(MAVLINK2REST_URL + '/mavlink', json=data)
            return result.status_code == 200
        except Exception as error:
            print("Error setting parameter: " + str(error))
            return False

    def send_vision(self, position_deltas, rotation_deltas=[0, 0, 0], confidence=100, dt=125000):
        "Sends message VISION_POSITION_DELTA to flight controller"
        data = self.vision_template.format(dt=int(dt),
                                           dRoll=rotation_deltas[0],
                                           dPitch=rotation_deltas[1],
                                           dYaw=rotation_deltas[2],
                                           dx=position_deltas[0],
                                           dy=position_deltas[1],
                                           dz=position_deltas[2],
                                           confidence=confidence)

        post(MAVLINK2REST_URL + '/mavlink', data=data)

    def send_vision_speed_estimate(self, speed_estimates):
        "Sends message VISION_SPEED_ESTIMATE to flight controller"
        data = self.vision_speed_estimate_template.format(
                                  us=int((time.time()-self.start_time)*1e6),
                                  vx=speed_estimates[0],
                                  vy=speed_estimates[1],
                                  vz=speed_estimates[2])

        post(MAVLINK2REST_URL + '/mavlink', data=data)

    def send_vision_position_estimate(self, timestamp, position_estimates, attitude_estimates=[0.0, 0.0, 0.0]):
        "Sends message VISION_POSITION_ESTIMATE to flight controller"
        data = self.vision_position_estimate_template.format(
                                  us=int(timestamp*1e3),
                                  roll=radians(attitude_estimates[0]),
                                  pitch=radians(attitude_estimates[1]),
                                  yaw=radians(attitude_estimates[2]),
                                  x=position_estimates[0],
                                  y=position_estimates[1],
                                  z=position_estimates[2])
        post(MAVLINK2REST_URL + '/mavlink', data=data)

    def send_rangefinder(self, distance: float):
        "Sends message DISTANCE_SENSOR to flight controller"
        if distance == -1:
            return
        data = self.rangefinder_template.format(int(distance*100))

        post(MAVLINK2REST_URL + '/mavlink', data=data)

    def set_gps_origin(self, lat, lon):
        data = self.gps_origin_template.format(lat=int(float(lat)*1e7), lon=int(float(lon)*1e7))
        post(MAVLINK2REST_URL + '/mavlink', data=data)

    def get_orientation(self):
        """
        fetches ROV orientation
        """
        return self.get_float('/VFR_HUD/heading')
