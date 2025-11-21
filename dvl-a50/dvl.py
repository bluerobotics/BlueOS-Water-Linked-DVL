"""
Code for integration of Water Linked DVL A50/A125 with BlueOS and ArduSub
"""

import json
import math
import os
import socket
import threading
import time
from enum import Enum
from select import select
from typing import Any, Dict, List

from loguru import logger

from blueoshelper import request
from dvlfinder import find_the_dvl
from mavlink2resthelper import GPS_GLOBAL_ORIGIN_ID, Mavlink2RestHelper

HOSTNAME = "waterlinked-dvl.local"
DVL_DOWN = 1
DVL_FORWARD = 2
LATLON_TO_CM = 1.1131884502145034e5


class MessageType(str, Enum):
    POSITION_DELTA = "POSITION_DELTA"
    POSITION_ESTIMATE = "POSITION_ESTIMATE"
    SPEED_ESTIMATE = "SPEED_ESTIMATE"

    @staticmethod
    def contains(value):
        return value in set(item.value for item in MessageType)


# pylint: disable=too-many-instance-attributes
# pylint: disable=unspecified-encoding
# pylint: disable=too-many-branches
# pylint: disable=too-many-statements
class DvlDriver(threading.Thread):
    """
    Responsible for the DVL interactions themselves.
    This handles fetching the DVL data and forwarding it to Ardusub
    """

    status = "Starting"
    version = ""
    mav = Mavlink2RestHelper()
    socket = None
    port = 16171  # Water Linked mentioned they won't allow changing or disabling this
    orientation = DVL_DOWN
    enabled = True
    rangefinder = True
    hostname = HOSTNAME
    timeout = 3  # tcp timeout in seconds
    origin = [0, 0]
    saved_settings = [
        "enabled",
        "orientation",
        "hostname",
        "origin",
        "rangefinder",
        "should_send",
    ]
    settings_path = os.path.join(os.path.expanduser("~"), ".config", "dvl", "settings.json")

    should_send = MessageType.POSITION_DELTA
    reset_counter = 0
    timestamp = 0
    last_temperature_check_time = 0
    temperature_check_interval_s = 30
    temperature_too_hot = 45

    def __init__(self, orientation=DVL_DOWN) -> None:
        threading.Thread.__init__(self)
        self.orientation = orientation
        # used for calculating attitude delta
        self.last_attitude = (0, 0, 0)
        self.current_attitude = (0, 0, 0)

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
                for setting_name in self.saved_settings:
                    if setting_name in data:
                        setattr(self, setting_name, data[setting_name])
                    else:
                        default = getattr(self, setting_name)
                        logger.warning(f"key not found: {setting_name} - keeping {default=} instead:")
                logger.debug("Loaded settings: ", data)
        except FileNotFoundError:
            logger.warning("Settings file not found, using default.")
        except ValueError:
            logger.warning("File corrupted, using default settings.")

    @property
    def current_settings(self):
        return {setting_name: getattr(self, setting_name) for setting_name in self.saved_settings}

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
        with open(self.settings_path, "w") as settings:
            settings.write(json.dumps(self.current_settings))

    def get_status(self) -> dict:
        """
        Returns a dict with the current status
        """
        return {"status": self.status, **self.current_settings}

    @property
    def host(self) -> str:
        """Make sure there is no port in the hostname allows local testing by where http can be running on other ports than 80"""
        try:
            host = self.hostname.split(":")[0]
        except IndexError:
            host = self.hostname
        return host

    def look_for_dvl(self):
        """
        Waits for the dvl to show up at the designated hostname
        """
        self.wait_for_cable_guy()
        ip = self.hostname
        self.report_status(f"Trying to talk to dvl at http://{ip}/api/v1/about")
        while "DVL not found":
            if request(f"http://{ip}/api/v1/about"):
                self.report_status(f"DVL found at {ip}, using it.")
                return
            self.report_status(f"Could not talk to dvl at {ip}, looking for it in the local network...")
            try:
                found_dvl = find_the_dvl(report_status=self.report_status)
                if found_dvl is not None:
                    self.report_status(f"Dvl found at address {found_dvl}, using it instead.")
                    self.hostname = found_dvl
                    self.save_settings()
                    return
            except Exception as e:
                self.report_status(f"Unable to find dvl: {e}")
            time.sleep(1)

    def wait_for_cable_guy(self):
        while not request("http://host.docker.internal/cable-guy/v1.0/ethernet"):
            self.report_status("waiting for cable-guy to come online...")
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
            self.orientation = orientation
            self.save_settings()
            return True
        return False

    def set_should_send(self, should_send):
        if not MessageType.contains(should_send):
            raise ValueError(f"bad messagetype: {should_send}")
        self.should_send = should_send
        self.save_settings()

    @staticmethod
    def longitude_scale(lat: float):
        """
        from https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L325
        """
        scale = math.cos(math.radians(lat))
        return max(scale, 0.01)

    def lat_lng_to_NE_XY_cm(self, lat: float, lon: float) -> List[float]:
        """
        From https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L206
        """
        x = (lat - self.origin[0]) * LATLON_TO_CM
        y = self.longitude_scale((lat + self.origin[0]) / 2) * LATLON_TO_CM * (lon - self.origin[1])
        return [x, y]

    def has_origin_set(self) -> bool:
        try:
            old_time = self.mav.get_float("/GPS_GLOBAL_ORIGIN/message/time_usec")
            if math.isnan(old_time):
                logger.warning("Unable to read current time for GPS_GLOBAL_ORIGIN, using 0")
                old_time = 0
        except Exception as e:
            logger.warning(f"Unable to read current time for GPS_GLOBAL_ORIGIN, using 0: {e}")
            old_time = 0

        for attempt in range(5):
            logger.debug(f"Trying to read origin, try # {attempt}")
            self.mav.request_message(GPS_GLOBAL_ORIGIN_ID)
            time.sleep(0.5)  # make this a timeout?
            try:
                new_origin_data = json.loads(self.mav.get("/GPS_GLOBAL_ORIGIN/message"))
                if new_origin_data["time_usec"] != old_time:
                    self.origin = [new_origin_data["latitude"] * 1e-7, new_origin_data["longitude"] * 1e-7]
                    return True
                continue  # try again
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
            # code expects degrees, but the ATTITUDE message gives radians
            attitudes = [math.degrees(attitude[axis]) for axis in ("roll", "pitch", "yaw")]
            positions = [x, y, -depth]
            self.reset_counter += 1
            self.mav.send_vision_position_estimate(
                self.timestamp, positions, attitudes, reset_counter=self.reset_counter
            )

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
            self.mav.set_param("RNGFND1_TYPE", "MAV_PARAM_TYPE_UINT8", 10)  # MAVLINK
        return True

    def load_params(self, selector: str) -> bool:
        """
        Load EK3_SRC1 parameters to match the use case:
        "dvl"       The DVL will be used for horizontal position and velocity
        "dvl_gps"   The GPS will be used for horizontal position, and the DVL will be used for horizontal velocity
        """
        if selector == "dvl":
            self.mav.set_param("EK3_GPS_TYPE", "MAV_PARAM_TYPE_UINT8", 3)  # Disable
            self.mav.set_param("EK3_SRC1_POSXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
            self.mav.set_param("EK3_SRC1_VELXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
            self.mav.set_param("EK3_SRC1_POSZ", "MAV_PARAM_TYPE_UINT8", 1)  # BARO
            return True
        if selector == "dvl_gps":
            self.mav.set_param("EK3_GPS_TYPE", "MAV_PARAM_TYPE_UINT8", 0)  # Enable
            self.mav.set_param("EK3_SRC1_POSXY", "MAV_PARAM_TYPE_UINT8", 3)  # GPS
            self.mav.set_param("EK3_SRC1_VELXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
            self.mav.set_param("EK3_SRC1_POSZ", "MAV_PARAM_TYPE_UINT8", 1)  # BARO
            return True
        return False

    def setup_mavlink(self) -> None:
        """
        Sets up mavlink streamrates so we have the needed messages at the
        appropriate rates
        """
        self.report_status("Setting up MAVLink streams...")
        self.mav.ensure_message_frequency("ATTITUDE", 30, 5)
        self.mav.ensure_message_frequency("VFR_HUD", 74, 5)

    def setup_params(self) -> None:
        """
        Sets up the required params for DVL integration -- but leave the EK3_SRC1 params alone
        """
        self.mav.set_param("AHRS_EKF_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        # TODO: Check if really required. It doesn't look like the ekf2 stops at all
        self.mav.set_param("EK2_ENABLE", "MAV_PARAM_TYPE_UINT8", 0)

        self.mav.set_param("EK3_ENABLE", "MAV_PARAM_TYPE_UINT8", 1)
        self.mav.set_param("VISO_TYPE", "MAV_PARAM_TYPE_UINT8", 1)
        if self.rangefinder:
            self.mav.set_param("RNGFND1_TYPE", "MAV_PARAM_TYPE_UINT8", 10)  # MAVLINK

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

    def handle_velocity(self, data: Dict[str, Any]) -> None:
        # extract velocity data from the DVL JSON
        vx, vy, vz, alt, valid, fom = (
            data["vx"],
            data["vy"],
            data["vz"],
            data["altitude"],
            data["velocity_valid"],
            data["fom"],
        )
        dt = data["time"] / 1000
        dx = dt * vx
        dy = dt * vy
        dz = dt * vz

        # fom is the standard deviation. scaling it to a confidence from 0-100%
        # 0 is a very good measurement, 0.4 is considered a inaccurate measurement
        _fom_max = 0.4
        confidence = 100 * (1 - min(_fom_max, fom) / _fom_max) if valid else 0
        # confidence = 100 if valid else 0

        if not valid:
            logger.info("Invalid  dvl reading, ignoring it.")
            return

        if self.rangefinder and alt > 0.05:
            self.mav.send_rangefinder(alt)

        position_delta = [0, 0, 0]
        attitude_delta = [0, 0, 0]
        if self.should_send == MessageType.POSITION_DELTA:
            dRoll, dPitch, dYaw = [
                current_angle - last_angle
                for (current_angle, last_angle) in zip(self.current_attitude, self.last_attitude)
            ]
            if self.orientation == DVL_DOWN:
                position_delta = [dx, dy, dz]
                attitude_delta = [dRoll, dPitch, dYaw]
            elif self.orientation == DVL_FORWARD:
                position_delta = [dz, dy, -dx]
                attitude_delta = [dYaw, dPitch, -dRoll]
            self.mav.send_vision(position_delta, attitude_delta, dt=data["time"] * 1e3, confidence=confidence)
        elif self.should_send == MessageType.SPEED_ESTIMATE:
            velocity = [vx, vy, vz] if self.orientation == DVL_DOWN else [vz, vy, -vx]  # DVL_FORWARD
            self.mav.send_vision_speed_estimate(velocity)

        self.last_attitude = self.current_attitude

    def handle_position_local(self, data):
        self.current_attitude = data["roll"], data["pitch"], data["yaw"]
        if self.should_send == MessageType.POSITION_ESTIMATE:
            x, y, z = data["x"], data["y"], data["z"]
            self.timestamp = data["ts"]
            self.mav.send_vision_position_estimate(
                self.timestamp, [x, y, z], self.current_attitude, reset_counter=self.reset_counter
            )

    def check_temperature(self):
        now = time.time()
        if now - self.last_temperature_check_time < self.temperature_check_interval_s:
            return
        self.last_temperature_check_time = now
        try:
            status = json.loads(request(f"http://{self.hostname}/api/v1/about/status"))

            temp = float(status["temperature"])
            if temp > self.temperature_too_hot:
                self.report_status(f"DVL is too hot ({temp} C). Please cool it down.")
                self.mav.send_statustext(f"DVL is too hot ({temp} C). Please cool it down.")
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
                    logger.warning(f"Disconnected: {e}")
                    connected = False
                except Exception as e:
                    logger.warning(f"Error receiving: {e}")

            # Extract 1 complete line from the buffer if available
            if len(buf) > 0:
                lines = buf.split("\n", 1)
                if len(lines) > 1:
                    buf = lines[1]
                    data = json.loads(lines[0])

            if not connected:
                buf = ""
                self.report_status("restarting")
                self.reconnect()
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
            elif data["type"] == "position_local":
                self.handle_position_local(data)

            self.check_temperature()
            time.sleep(0.003)
        logger.error("Driver Quit! This should not happen.")
