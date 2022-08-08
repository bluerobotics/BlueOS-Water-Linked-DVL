"""
Code for integration of Waterlinked DVL A50 with BlueOS and ArduSub
"""
import threading
import time
from mavlink2resthelper import Mavlink2RestHelper
from blueoshelper import request
import json
import socket
from select import select
import math
import os
from enum import Enum

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
    port = 0
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

    def __init__(self, orientation=DVL_DOWN):
        threading.Thread.__init__(self)
        self.current_orientation = orientation

    def load_settings(self):
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
                print("Loaded settings: ", data)
        except FileNotFoundError:
            print("Settings file not found, using default.")
        except ValueError:
            print("File corrupted, using default settings.")
        except KeyError as error:
            print("key not found: ", error)
            print("using default instead")


    def save_settings(self):
        """
        Load settings from .config/dvl/settings.json
        """
        def ensure_dir(file_path):
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
    def host(self):
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
            # TODO: get mdns back in here
            try:
                self.version = request(f"http://{ip}/api/v1/about")
            except Exception as e:
                print(f"could not open url at ip {ip} : {e}")
                self.status = f"could not open url at ip {ip} : {e}"
            time.sleep(1)

    def detect_port(self):
        """
        Fetchs the TCP port from the DVL api, stores in self.port
        """
        while not self.port:
            time.sleep(1)
            # try old api first
            try:
                outputs_raw = request(
                    "http://{0}/api/v1/outputs".format(self.hostname))
                if outputs_raw:
                    outputs = json.loads(outputs_raw)
                    for output in outputs:
                        if output["format"] != "json_v3":
                            continue
                        if "port" not in output:
                            print("no port data from API?!")
                            self.port = 16171
                            print("Unable to get port from API, trying to use port {self.port}")
                        self.port = int(output["port"])
                        print("Using port {0} from API".format(self.port))
            # then the new one
            except:
                port_raw = request(
                    "http://{0}/api/v1/outputs/tcp".format(self.hostname))
                if port_raw:
                    data = json.loads(port_raw)
                    if "port" not in data:
                        print("no port data from API?!")
                        self.port = 16171
                    self.port = data["port"]
                    print("Using port {0} from API".format(self.port))

    def wait_for_vehicle(self):
        """
        Waits for a valid heartbeat to Mavlink2Rest
        """
        self.status = "Waiting for vehicle..."
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

    def longitude_scale(self, lat):
        scale = math.cos(lat * (math.pi / 180.0))
        return max(scale, 0.01)

    def latLngToXY (self, lat, lon):
        x = (lat-self.origin[0]) * LATLON_TO_CM
        y =  self.longitude_scale((lat + self.origin[0])/2) * LATLON_TO_CM * (lon-self.origin[1])
        return [x, y]

    def has_origin_set(self):
        data = json.loads(self.mav.get("/GLOBAL_POSITION_INT/message"))
        return data['lat'] != 0 or data['lon'] != 0

    def set_current_position(self, lat, lon):
        """
        Sets the EKF origin to lat, lon
        """
        lat = float(lat)
        lon = float(lon)
        if not self.has_origin_set():
            self.mav.set_gps_origin(lat, lon)
            self.origin = [lat, lon]
            self.save_settings()
        else:
            x, y = self.latLngToXY (lat, lon)
            depth = float(self.mav.get("/VFR_HUD/message/alt"))

            attitude = json.loads(self.mav.get("/ATTITUDE"))
            positions = [x, y, -depth]
            self.reset_counter += 1
            self.mav.send_vision_position_estimate(self.timestamp, positions, reset_counter=self.reset_counter)

    def set_gps_origin(self, lat, lon):
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

    def set_hostname(self, hostname: str) -> bool:
        """
        Sets the hostname where the driver looks for the DVL
        (typically waterlinked-dvl.local)
        """
        try:
            self.hostname = hostname
            if self.socket:  # Don't crash if the socket hasn't been configured yet
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            self.setup_connections(timeout=1)
            self.save_settings()
            return True
        except Exception as e:
            print("Failed set hostname: '{}'".format(e))
            return False

    def setup_mavlink(self):
        """
        Sets up mavlink streamrates so we have the needed messages at the
        appropriate rates
        """
        self.status = "Setting up MAVLink streams..."
        self.mav.ensure_message_frequency('ATTITUDE', 30, 5)

    def setup_params(self):
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

    def setup_connections(self, timeout=300):
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
        self.status = f"Setup connection to {self.host}:{self.port} timed out"
        return False

    def reconnect(self):
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except:
                pass
        success = self.setup_connections()
        if success:
            self.last_recv_time = time.time()  # Don't disconnect directly after connect
            return True

        return False

    def update_attitude(self) -> list:
        """
        Fetchs the Attitude and calculate Attitude deltas
        """
        # Getting the data from mavlink2rest takes around 5ms
        attitude_raw = self.mav.get("/ATTITUDE")
        if not attitude_raw:
            # TODO: report status?
            print("Failed fetching attitude!")
            return [0, 0, 0]
        attitude = json.loads(attitude_raw)
        current_attitude = (
            attitude["roll"], attitude["pitch"], attitude["yaw"])
        angles = list(map(float.__sub__, current_attitude, self.last_attitude))
        angles[2] = angles[2] % (math.pi*2)
        self.last_attitude = current_attitude
        return angles

    def handle_velocity(self, data):
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

        # feeding back the angles seem to aggravate the gyro drift issue
        # angles = self.update_attitude()
        angles = [0, 0, 0]


        if self.rangefinder:
            self.mav.send_rangefinder(alt)

        if not valid:
            print("invalid reading, ignoring it.")
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

    def run(self):
        """
        Runs the main routing
        """
        self.load_settings()
        self.look_for_dvl()
        self.detect_port()
        self.setup_connections()
        self.wait_for_vehicle()
        self.setup_mavlink()
        self.setup_params()
        time.sleep(1)
        #self.set_gps_origin(*self.origin)
        self.status = "Running"
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
                    print("Disconnected")
                    connected = False
                except Exception as e:
                    print("Error receiveing:", e)
                    pass

            # Extract 1 complete line from the buffer if available
            if len(buf) > 0:
                lines = buf.split("\n", 1)
                if len(lines) > 1:
                    buf = lines[1]
                    data = json.loads(lines[0])

            if not connected:
                buf = ""
                self.status = "restarting"
                dis = self.reconnect()
                time.sleep(0.003)
                continue

            if not data:
                if time.time() - self.last_recv_time > self.timeout:
                    buf = ""
                    self.status = "timeout, restarting"
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

            time.sleep(0.003)
