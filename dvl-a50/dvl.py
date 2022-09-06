"""
Code for integration of Waterlinked DVL A50 with Companion and ArduSub
"""
import threading
import time
from mavlink2resthelper import Mavlink2RestHelper, GPS_GLOBAL_ORIGIN_ID
from blueoshelper import request
import json
import socket
from select import select
import math
import os
from enum import Enum
from dvlfinder import find_the_dvl
from loguru import logger

from typing import Any, List, Dict

HOSTNAME = "waterlinked-dvl.local"
DVL_DOWN = 1
DVL_FORWARD = 2
LATLON_TO_CM = 1.1131884502145034e5

class MessageType(str, Enum):
    POSITION_DELTA = "POSITION_DELTA"
    POSITION_ESTIMATE = "POSITION_ESTIMATE"
    SPEED_ESTIMATE = "SPEED_ESTIMATE"

    def contains(value):
        return value in set(item.value for item in MessageType)


class DvlDriver (threading.Thread):
    """
    Responsible for the DVL interactions themselves.
    This handles fetching the DVL data and forwarding it to Ardusub
    """
    status = "Starting"
    version = ""
    mav = Mavlink2RestHelper()
    socket = None
    port = 16171  # Waterlinked mentioned they won't allow changing or disabling this
    last_attitude = (0, 0, 0)  # used for calculating the attitude delta
    current_orientation = DVL_DOWN
    enabled = True
    rangefinder = True
    hostname = HOSTNAME
    timeout = 3 # tcp timeout in seconds
    origin = [0, 0]
    settings_path = os.path.join(os.path.expanduser(
        "~"), ".config", "dvl", "settings.json")

    should_send = MessageType.POSITION_DELTA
    reset_counter = 0
    timestamp = 0
    last_temperature_check_time = 0
    temperature_check_interval_s = 30
    temperature_too_hot = 45

    def __init__(self, orientation=DVL_DOWN) -> None:
        threading.Thread.__init__(self)
        self.current_orientation = orientation

    def report_status(self, msg: str) -> None:
        self.status = msg
        logger.debug(msg)

    def load_settings(self) -> None:
        """
        Load settings from .config/dvl/settings.json
        """
        try:
            with open(self.settings_path) as settings:
                data = json.load(settings)
                self.enabled = data["enabled"]
                self.current_orientation = data["orientation"]
                self.hostname = data["hostname"]
                self.origin = data["origin"]
                self.rangefinder = data["rangefinder"]
                self.should_send = data["should_send"]
                logger.debug("Loaded settings: ", data)
        except FileNotFoundError:
            logger.warning("Settings file not found, using default.")
        except ValueError:
            logger.warning("File corrupted, using default settings.")
        except KeyError as error:
            logger.warning("key not found: ", error)
            logger.warning("using default instead")


    def save_settings(self) -> None:
        """
        Load settings from .config/dvl/settings.json
        """
        def ensure_dir(file_path) -> None:
            """
            Helper to guarantee that the file path exists
            """
            directory = os.path.dirname(file_path)
            if not os.path.exists(directory):
                os.makedirs(directory)

        ensure_dir(self.settings_path)
        with open(self.settings_path, 'w') as settings:
            settings.write(json.dumps({
                "enabled": self.enabled,
                "orientation": self.current_orientation,
                "hostname": self.hostname,
                "rangefinder": self.rangefinder,
                "should_send": self.should_send,
            }))

    def get_status(self) -> dict:
        """
        Returns a dict with the current status
        """
        return {
            "status": self.status,
            "enabled": self.enabled,
            "orientation": self.current_orientation,
            "hostname": self.hostname,
            "origin": self.origin,
            "rangefinder": self.rangefinder,
            "should_send": self.should_send,
        }

    @property
    def host(self) -> str:
        """ Make sure there is no port in the hostname allows local testing by where http can be running on other ports than 80"""
        try:
            host = self.hostname.split(":")[0]
        except IndexError:
            host = self.hostname
        return host

    def look_for_dvl(self):
        """
        Waits for the dvl to show up at the designated hostname
        """
        ip = self.hostname
        self.status = f"Trying to talk to dvl at http://{ip}/api/v1/about"
        while not self.version:
            if not request(f"http://{ip}/api/v1/about"):
                self.report_status(f"could not talk to dvl at {ip}, looking for it in the local network...")
            found_dvl = find_the_dvl()
            if found_dvl:
                self.report_status(f"Dvl found at address {found_dvl}, using it instead.")
                self.hostname = found_dvl
                return
            time.sleep(1)

    def wait_for_vehicle(self):
        """
        Waits for a valid heartbeat to Mavlink2Rest
        """
        self.report_status("Waiting for vehicle...")
        while not self.mav.get("/HEARTBEAT"):
            time.sleep(1)

    def set_orientation(self, orientation: int) -> bool:
        """
        Sets the DVL orientation, either DVL_FORWARD of DVL_DOWN
        """
        if orientation in [DVL_FORWARD, DVL_DOWN]:
            self.current_orientation = orientation
            self.save_settings()
            return True
        return False

    def set_should_send(self, should_send):
        if not MessageType.contains(should_send):
            raise Exception(f"bad messagetype: {should_send}")
        self.should_send = should_send
        self.save_settings()

    @staticmethod
    def longitude_scale(lat: float):
        """
        from https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L325
        """
        scale = math.cos(math.radians(lat))
        return max(scale, 0.01)

    def lat_lng_to_NE_XY_cm (self, lat: float, lon: float) -> List[float]:
        """
        From https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L206
        """
        x = (lat-self.origin[0]) * LATLON_TO_CM
        y =  self.longitude_scale((lat + self.origin[0])/2) * LATLON_TO_CM * (lon-self.origin[1])
        return [x, y]

    def has_origin_set(self) -> bool:
        try:
            old_time = self.mav.get_float("/GPS_GLOBAL_ORIGIN/message/time_usec")
        except:
            old_time = 0

        for attempt in range(5):
            logger.debug(f"Trying to read origin, try # {attempt}")
            self.mav.request_message(GPS_GLOBAL_ORIGIN_ID)
            time.sleep(0.5) # make this a timeout?
            try:
                new_origin_data = json.loads(self.mav.get("/GPS_GLOBAL_ORIGIN/message"))
                if new_origin_data["time_usec"] != old_time:
                    self.origin = [new_origin_data["latitude"]*1e-7, new_origin_data["longitude"]*1e-7]
                    return True
                else:
                    continue # try again
            except Exception as e:
                logger.warning(e)
                return False
        return False

    def set_current_position(self, lat: float, lon: float):
        """
        Sets the EKF origin to lat, lon
        """
        # If origin has never been set, set it
        if not self.has_origin_set():
            logger.info("Origin was never set, trying to set it.")
            self.set_gps_origin(lat, lon)
        else:
            logger.info("Origin has already been set, sending POSITION_ESTIMATE instead")
            # if we already have an origin set, send a new position instead
            x, y = self.lat_lng_to_NE_XY_cm(lat, lon)
            depth = float(self.mav.get("/VFR_HUD/message/alt"))

            attitude = json.loads(self.mav.get("/ATTITUDE/message"))
            attitudes = [attitude["roll"], attitude["pitch"], attitude["yaw"]]
            positions = [x, y, -depth]
            self.reset_counter += 1
            self.mav.send_vision_position_estimate(self.timestamp, positions, attitudes, reset_counter=self.reset_counter)

    def set_gps_origin(self, lat: float, lon: float) -> None:
        """
        Sets the EKF origin to lat, lon
        """
        self.mav.set_gps_origin(lat, lon)
        self.origin = [float(lat), float(lon)]
        self.save_settings()

    def set_enabled(self, enable: bool) -> bool:
        """
        Enables/disables the driver
        """
        self.enabled = enable
        self.save_settings()
        return True

    def set_use_as_rangefinder(self, enable: bool) -> bool:
        """
        Enables/disables DISTANCE_SENSOR messages
        """
        self.rangefinder = enable
        self.save_settings()
        if enable:
            self.mav.set_param("RNGFND1_TYPE", "MAV_PARAM_TYPE_UINT8", 10) # MAVLINK
        return True

    def setup_mavlink(self) -> None:
        """
        Sets up mavlink streamrates so we have the needed messages at the
        appropriate rates
        """
        self.report_status("Setting up MAVLink streams...")
        self.mav.ensure_message_frequency('ATTITUDE', 30, 5)

    def setup_params(self) -> None:
        """
        Sets up the required params for DVL integration
        """
        self.mav.set_param("AHRS_EKF_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        # TODO: Check if really required. It doesn't look like the ekf2 stops at all
        self.mav.set_param("EK2_ENABLE", "MAV_PARAM_TYPE_UINT8", 0)

        self.mav.set_param("EK3_ENABLE", "MAV_PARAM_TYPE_UINT8", 1)
        self.mav.set_param("VISO_TYPE", "MAV_PARAM_TYPE_UINT8", 1)
        self.mav.set_param("EK3_GPS_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        self.mav.set_param("EK3_SRC1_POSXY", "MAV_PARAM_TYPE_UINT8", 6) # EXTNAV
        self.mav.set_param("EK3_SRC1_VELXY", "MAV_PARAM_TYPE_UINT8", 6) # EXTNAV
        self.mav.set_param("EK3_SRC1_POSZ", "MAV_PARAM_TYPE_UINT8", 1) # BARO
        if self.rangefinder:
            self.mav.set_param("RNGFND1_TYPE", "MAV_PARAM_TYPE_UINT8", 10) # MAVLINK

    def setup_connections(self, timeout=300) -> None:
        """
        Sets up the socket to talk to the DVL
        """
        while timeout > 0:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                self.socket.setblocking(0)
                return True
            except socket.error:
                time.sleep(0.1)
            timeout -= 1
        self.report_status(f"Setup connection to {self.host}:{self.port} timed out")
        return False

    def reconnect(self):
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except Exception as e:
                self.report_status(f"Unable to reconnect: {e}, looking for dvl again...")
                self.look_for_dvl()
        success = self.setup_connections()
        if success:
            self.last_recv_time = time.time()  # Don't disconnect directly after connect
            return True

        return False

    def handle_velocity(self, data: Dict[str,Any]) -> None:
        # TODO: test if this is used by ArduSub or could be [0, 0, 0]
        # extract velocity data from the DVL JSON

        vx, vy, vz, alt, valid, fom = data["vx"], data["vy"], data["vz"], data["altitude"], data["velocity_valid"], data["fom"]
        dt = data["time"] / 1000
        dx = dt*vx
        dy = dt*vy
        dz = dt*vz

        # fom is the standard deviation. scaling it to a confidence from 0-100%
        # 0 is a very good measurement, 0.4 is considered a inaccurate measurement
        _fom_max = 0.4
        confidence = 100 * (1-min(_fom_max, fom)/_fom_max) if valid else 0
        # confidence = 100 if valid else 0

        # feeding back the angles seemed to aggravate the gyro drift issue
        angles = [0, 0, 0]


        if self.rangefinder:
            self.mav.send_rangefinder(alt)

        if not valid:
            logger.info("Invalid  dvl reading, ignoring it.")
            return

        if self.should_send == MessageType.POSITION_DELTA:
            if self.current_orientation == DVL_DOWN:
                self.mav.send_vision([dx, dy, dz],
                                        angles,
                                        dt=data["time"]*1e3,
                                        confidence=confidence)
            elif self.current_orientation == DVL_FORWARD:
                self.mav.send_vision([dz, dy, -dx],
                                        angles,
                                        dt=data["time"]*1e3,
                                        confidence=confidence)
        elif self.should_send == MessageType.SPEED_ESTIMATE:
            if self.current_orientation == DVL_DOWN:
                self.mav.send_vision_speed_estimate([vx, vy, vz])
            elif self.current_orientation == DVL_FORWARD:
                self.mav.send_vision_speed_estimate([vz, vy, -vx])

    def handle_position_local(self, data):
        if self.should_send == MessageType.POSITION_ESTIMATE:
            x, y, z, roll, pitch, yaw = data["x"], data["y"], data["z"], data["roll"], data["pitch"], data["yaw"]
            self.timestamp = data["ts"]
            self.mav.send_vision_position_estimate(
                    self.timestamp,
                    [x, y, z],
                    [roll, pitch, yaw],
                    reset_counter=self.reset_counter)

    def check_temperature(self):
        now = time.time()
        if now - self.last_temperature_check_time < self.temperature_check_interval_s:
            return
        self.last_temperature_check_time = now
        try:
            status = json.loads(request(f"http://{self.hostname}/api/v1/about/status"))

            temp = float(status["temperature"])
            if temp > self.temperature_too_hot:
                self.report_status(f'DVL is too hot ({temp} C). Please cool it down.')
                self.mav.send_statustext(f'DVL is too hot ({temp} C). Please cool it down.')
        except Exception as e:
            self.report_status(e)

    def run(self):
        """
        Runs the main routing
        """
        self.load_settings()
        self.look_for_dvl()
        self.setup_connections()
        self.wait_for_vehicle()
        self.setup_mavlink()
        self.setup_params()
        time.sleep(1)
        self.report_status("Running")
        self.last_recv_time = time.time()
        buf = ""
        connected = True
        while True:
            if not self.enabled:
                time.sleep(1)
                buf = ""  # Reset buf when disabled
                continue
            r, _, _ = select([self.socket], [], [], 0)
            data = None
            if r:
                try:
                    recv = self.socket.recv(1024).decode()
                    connected = True
                    if recv:
                        self.last_recv_time = time.time()
                        buf += recv
                except socket.error as e:
                    logger.warning("Disconnected")
                    connected = False
                except Exception as e:
                    logger.warning("Error receiveing:", e)
                    pass

            # Extract 1 complete line from the buffer if available
            if len(buf) > 0:
                lines = buf.split("\n", 1)
                if len(lines) > 1:
                    buf = lines[1]
                    data = json.loads(lines[0])

            if not connected:
                buf = ""
                self.report_status("restarting")
                dis = self.reconnect()
                time.sleep(0.003)
                continue

            if not data:
                if time.time() - self.last_recv_time > self.timeout:
                    buf = ""
                    self.report_status("timeout, restarting")
                    connected = self.reconnect()
                time.sleep(0.003)
                continue

            self.status = "Running"

            if "type" not in data:
                continue

            if data["type"] == "velocity":

                self.handle_velocity(data)

            if data["type"] == "position_local":
                self.handle_position_local(data)
            self.check_temperature()
            time.sleep(0.003)