import logging
from copy import deepcopy
from enum import Enum
from pprint import pformat
from typing import TYPE_CHECKING

from lerobot.utils.import_utils import _st3215_available, require_package

from ..encoding_utils import decode_sign_magnitude, encode_sign_magnitude
from ..motors_bus import Motor, MotorCalibration, NameOrID, SerialMotorsBus, Value, get_address
from .tables import (
    FIRMWARE_MAJOR_VERSION,
    FIRMWARE_MINOR_VERSION,
    MODEL_BAUDRATE_TABLE,
    MODEL_CONTROL_TABLE,
    MODEL_ENCODING_TABLE,
    MODEL_NUMBER,
    MODEL_NUMBER_TABLE,
    MODEL_PROTOCOL,
    MODEL_RESOLUTION,
    SCAN_BAUDRATES,
)

if TYPE_CHECKING or _st3215_available:
    import st3215
else:
    st3215 = None

DEFAULT_PROTOCOL_VERSION = 0
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT_MS = 1000

NORMALIZED_DATA = ["Goal_Position", "Present_Position"]

logger = logging.getLogger(__name__)

class OperatingMode(Enum):
    # position servo mode
    POSITION = 0
    # The motor is in constant speed mode, which is controlled by parameter 0x2e, and the highest bit 15 is
    # the direction bit
    VELOCITY = 1
    # PWM open-loop speed regulation mode, with parameter 0x2c running time parameter control, bit11 as
    # direction bit
    PWM = 2
    # In step servo mode, the number of step progress is represented by parameter 0x2a, and the highest bit 15
    # is the direction bit
    STEP = 3

class DriveMode(Enum):
    NON_INVERTED = 0
    INVERTED = 1


class TorqueMode(Enum):
    ENABLED = 1
    DISABLED = 0

class WaveshareMotorsBus(SerialMotorsBus):

    apply_drive_mode = True
    available_baudrates = deepcopy(SCAN_BAUDRATES)
    default_baudrate = DEFAULT_BAUDRATE
    default_timeout = DEFAULT_TIMEOUT_MS
    model_baudrate_table = deepcopy(MODEL_BAUDRATE_TABLE)
    model_ctrl_table = deepcopy(MODEL_CONTROL_TABLE)
    model_encoding_table = deepcopy(MODEL_ENCODING_TABLE)
    model_number_table = deepcopy(MODEL_NUMBER_TABLE)
    model_resolution_table = deepcopy(MODEL_RESOLUTION)
    normalized_data = deepcopy(NORMALIZED_DATA)

    def __init__():
        super().__init__()

    def _handshake(self):
        return super()._handshake()

    def configure_motors(self):
        return super().configure_motors()

    def _find_single_motor(self, motor, initial_baudrate = None):
        return super()._find_single_motor(motor, initial_baudrate)

    def _find_single_motor_p0(self, motor, initial_baudrate = None):
        return super()._find_single_motor_p0(motor, initial_baudrate)

    def _find_single_motor_p1(self, motor, initial_baudrate = None):
        return super()._find_single_motor_p1(motor, initial_baudrate)
    
    def config_motors(self, motors: list[Motor]):
        return super().config_motors(motors)
    
    @property
    def is_calibrated(self):
        return super().is_calibrated

    def read_calibration(self):
        return super().read_calibration()
    
    def write_calibration(self, calibration_dict, cache = True):
        return super().write_calibration(calibration_dict, cache)
    
    def _get_half_turn_homings(self, positions):
        return super()._get_half_turn_homings(positions)
    
    def disable_torque(self, motors = None, num_retry = 0):
        return super().disable_torque(motors, num_retry)
    
    def _disable_torque(self, motor, model, num_retry = 0):
        return super()._disable_torque(motor, model, num_retry)
    
    def enable_torque(self, motors = None, num_retry = 0):
        return super().enable_torque(motors, num_retry)
    
    def _encode_sign(self, data_name, ids_values):
        return super()._encode_sign(data_name, ids_values)
    
    def _decode_sign(self, data_name, ids_values):
        return super()._decode_sign(data_name, ids_values)
    
    def _split_into_byte_chunks(self, value, length):
        return super()._split_into_byte_chunks(value, length)
    
    def _broadcast_ping(self, model_number, protocol_version):
        return super()._broadcast_ping(model_number, protocol_version)
    
    def broadcast_ping(self, num_retry = 0, raise_on_error = False):
        return super().broadcast_ping(num_retry, raise_on_error)
    
    def _read_firmware_version(self, motor):
        return super()._read_firmware_version(motor)
    
    def _read_model_number(self, motor):
        return super()._read_model_number(motor)