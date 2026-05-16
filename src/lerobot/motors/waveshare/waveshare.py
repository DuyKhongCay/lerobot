import logging
from copy import deepcopy
from enum import Enum
from pprint import pformat
from typing import TYPE_CHECKING

from lerobot.utils.import_utils import _st3215_available, require_package

from ..encoding_utils import decode_sign_magnitude, encode_sign_magnitude
from ..motors_bus import (
    Motor,
    MotorCalibration,
    NameOrID,
    SerialMotorsBus,
    Value,
    get_address,
)
from .tables import (
    FIRMWARE_MAJOR_VERSION,
    FIRMWARE_MINOR_VERSION,
    MODEL_BAUDRATE_TABLE,
    MODEL_CONTROL_TABLE,
    MODEL_ENCODING_TABLE,
    MODEL_NUMBER_TABLE,
    MODEL_PROTOCOL,
    MODEL_RESOLUTION,
    SCAN_BAUDRATES,
    SERVO_VERSION_NUMBER,
)

if TYPE_CHECKING or _st3215_available:
    from st3215 import values as sts
    from st3215.group_sync_read import GroupSyncRead as St3215GroupSyncRead
    from st3215.group_sync_write import GroupSyncWrite as St3215GroupSyncWrite
    from st3215.port_handler import PortHandler as St3215PortHandler
    from st3215.protocol_packet_handler import (
        protocol_packet_handler as St3215PacketHandler,
    )
else:
    sts = None
    St3215GroupSyncRead = None
    St3215GroupSyncWrite = None
    St3215PortHandler = None
    St3215PacketHandler = None

DEFAULT_PROTOCOL_VERSION = 0
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT_MS = 1000

NORMALIZED_DATA = ["Goal_Position", "Present_Position"]

logger = logging.getLogger(__name__)


class St3215PacketHandlerAdapter:
    def __init__(self, port_handler):
        self._handler = St3215PacketHandler(port_handler)

    def __getattr__(self, name):
        return getattr(self._handler, name)

    def getTxRxResult(self, result):
        return self._handler.getTxRxResult(result)

    def getRxPacketError(self, error):
        return self._handler.getRxPacketError(error)

    def ping(self, port, id):
        return self._handler.ping(id)

    def read1ByteTxRx(self, port, id, address):
        return self._handler.read1ByteTxRx(id, address)

    def read2ByteTxRx(self, port, id, address):
        return self._handler.read2ByteTxRx(id, address)

    def read4ByteTxRx(self, port, id, address):
        return self._handler.read4ByteTxRx(id, address)

    def writeTxRx(self, port, id, address, length, data):
        return self._handler.writeTxRx(id, address, length, data)

    def txPacket(self, port, txpacket):
        return self._handler.txPacket(txpacket)

    def sts_makeword(self, low, high):
        return self._handler.sts_makeword(low, high)

    def sts_makedword(self, low, high):
        return self._handler.sts_makedword(low, high)

    def scs_makeword(self, low, high):
        return self._handler.sts_makeword(low, high)

    def scs_makedword(self, low, high):
        return self._handler.sts_makedword(low, high)


def patch_setBaudRate(self, baudrate):  # noqa: N802
    self.baudrate = baudrate
    if self.is_open:
        return self.setupPort()
    return True


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

    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
        protocol_version: int = DEFAULT_PROTOCOL_VERSION,
    ):
        require_package("st3215", extra="st3215", import_name="st3215")
        super().__init__(port, motors, calibration)
        self.protocol_version = protocol_version
        self._assert_same_protocol()
        self.port_handler = St3215PortHandler(self.port)
        self.port_handler.setBaudRate = patch_setBaudRate.__get__(
            self.port_handler, St3215PortHandler
        )
        self.packet_handler = St3215PacketHandlerAdapter(self.port_handler)
        self.sync_reader = St3215GroupSyncRead(self.packet_handler, 0, 0)
        self.sync_writer = St3215GroupSyncWrite(self.packet_handler, 0, 0)
        self._comm_success = sts.COMM_SUCCESS
        self._no_error = 0x00

    def _assert_same_protocol(self) -> None:
        if any(MODEL_PROTOCOL[model] != self.protocol_version for model in self.models):
            raise RuntimeError("Some motors use an incompatible protocol.")

    def _assert_protocol_is_compatible(self, instruction_name: str) -> None:
        if instruction_name == "sync_read" and self.protocol_version == 1:
            raise NotImplementedError(
                "'Sync Read' is not available with Waveshare motors using Protocol 1. Use 'Read' sequentially instead."
            )
        if instruction_name == "broadcast_ping" and self.protocol_version == 1:
            raise NotImplementedError(
                "'Broadcast Ping' is not available with Waveshare motors using Protocol 1. "
                "Use 'Ping' sequentially instead."
            )

    def _assert_same_firmware(self) -> None:
        firmware_versions = self._read_firmware_version(self.ids, raise_on_error=True)
        if len(set(firmware_versions.values())) != 1:
            raise RuntimeError(
                f"Some Motors use different firmware versions:\n{pformat(firmware_versions)}\n"
            )

    def _handshake(self) -> None:
        self._assert_motors_exist()
        self._assert_same_firmware()

    def _find_single_motor(
        self, motor: str, initial_baudrate: int | None = None
    ) -> tuple[int, int]:
        if self.protocol_version == 0:
            return self._find_single_motor_p0(motor, initial_baudrate)
        else:
            return self._find_single_motor_p1(motor, initial_baudrate)

    def _find_single_motor_p0(
        self, motor: str, initial_baudrate: int | None = None
    ) -> tuple[int, int]:
        model = self.motors[motor].model
        search_baudrates = (
            [initial_baudrate]
            if initial_baudrate is not None
            else self.model_baudrate_table[model]
        )
        expected_model_nb = self.model_number_table[model]

        for baudrate in search_baudrates:
            self.set_baudrate(baudrate)
            id_model = self.broadcast_ping()
            if id_model:
                found_id, found_model = next(iter(id_model.items()))
                if found_model != expected_model_nb:
                    raise RuntimeError(
                        f"Found one motor on {baudrate=} with id={found_id} but it has a "
                        f"model number '{found_model}' different than the one expected: '{expected_model_nb}'. "
                        f"Make sure you are connected only connected to the '{motor}' motor (model '{model}')."
                    )
                return baudrate, found_id

        raise RuntimeError(
            f"Motor '{motor}' (model '{model}') was not found. Make sure it is connected."
        )

    def _find_single_motor_p1(
        self, motor: str, initial_baudrate: int | None = None
    ) -> tuple[int, int]:
        model = self.motors[motor].model
        search_baudrates = (
            [initial_baudrate]
            if initial_baudrate is not None
            else self.model_baudrate_table[model]
        )
        expected_model_nb = self.model_number_table[model]

        for baudrate in search_baudrates:
            self.set_baudrate(baudrate)
            for id_ in range(sts.MAX_ID + 1):
                found_model = self.ping(id_)
                if found_model is not None:
                    if found_model != expected_model_nb:
                        raise RuntimeError(
                            f"Found one motor on {baudrate=} with id={id_} but it has a "
                            f"model number '{found_model}' different than the one expected: '{expected_model_nb}'. "
                            f"Make sure you are connected only connected to the '{motor}' motor (model '{model}')."
                        )
                    return baudrate, id_

        raise RuntimeError(
            f"Motor '{motor}' (model '{model}') was not found. Make sure it is connected."
        )

    def configure_motors(
        self, return_delay_time=0, maximum_acceleration=254, acceleration=254
    ) -> None:
        for motor in self.motors:
            self.write("Return_Delay_Time", motor, return_delay_time)
            if self.protocol_version == 0:
                self.write("Maximum_Acceleration", motor, maximum_acceleration)
            self.write("Acceleration", motor, acceleration)

            if self.motors[motor].model == "st3215":
                phase = self.read("Phase", motor, normalize=False)
                if int(phase) & 0x10:
                    self.write("Phase", motor, int(phase) & ~0x10)

    @property
    def is_calibrated(self) -> bool:
        motors_calibration = self.read_calibration()
        if set(motors_calibration) != set(self.calibration):
            return False

        same_ranges = all(
            self.calibration[motor].range_min == cal.range_min
            and self.calibration[motor].range_max == cal.range_max
            for motor, cal in motors_calibration.items()
        )
        if self.protocol_version == 1:
            return same_ranges

        same_offsets = all(
            self.calibration[motor].homing_offset == cal.homing_offset
            for motor, cal in motors_calibration.items()
        )
        return same_ranges and same_offsets

    def read_calibration(self) -> dict[str, MotorCalibration]:
        offsets, mins, maxes = {}, {}, {}
        for motor in self.motors:
            mins[motor] = self.read("Min_Position_Limit", motor, normalize=False)
            maxes[motor] = self.read("Max_Position_Limit", motor, normalize=False)
            offsets[motor] = (
                self.read("Homing_Offset", motor, normalize=False)
                if self.protocol_version == 0
                else 0
            )

        calibration = {}
        for motor, m in self.motors.items():
            calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=int(offsets[motor]),
                range_min=int(mins[motor]),
                range_max=int(maxes[motor]),
            )

        return calibration

    def write_calibration(
        self, calibration_dict: dict[str, MotorCalibration], cache: bool = True
    ) -> None:
        for motor, calibration in calibration_dict.items():
            if self.protocol_version == 0:
                self.write("Homing_Offset", motor, calibration.homing_offset)
            self.write("Min_Position_Limit", motor, calibration.range_min)
            self.write("Max_Position_Limit", motor, calibration.range_max)

        if cache:
            self.calibration = calibration_dict

    def _get_half_turn_homings(
        self, positions: dict[NameOrID, Value]
    ) -> dict[NameOrID, Value]:
        half_turn_homings: dict[NameOrID, Value] = {}
        for motor, pos in positions.items():
            model = self._get_motor_model(motor)
            max_res = self.model_resolution_table[model] - 1
            half_turn_homings[motor] = pos - int(max_res / 2)

        return half_turn_homings

    def disable_torque(
        self, motors: int | str | list[str] | None = None, num_retry: int = 0
    ) -> None:
        for motor in self._get_motors_list(motors):
            self.write(
                "Torque_Enable", motor, TorqueMode.DISABLED.value, num_retry=num_retry
            )
            self.write("Lock", motor, 0, num_retry=num_retry)

    def _disable_torque(self, motor: int, model: str, num_retry: int = 0) -> None:
        addr, length = get_address(self.model_ctrl_table, model, "Torque_Enable")
        self._write(addr, length, motor, TorqueMode.DISABLED.value, num_retry=num_retry)
        addr, length = get_address(self.model_ctrl_table, model, "Lock")
        self._write(addr, length, motor, 0, num_retry=num_retry)

    def enable_torque(
        self, motors: int | str | list[str] | None = None, num_retry: int = 0
    ) -> None:
        for motor in self._get_motors_list(motors):
            self.write(
                "Torque_Enable", motor, TorqueMode.ENABLED.value, num_retry=num_retry
            )
            self.write("Lock", motor, 1, num_retry=num_retry)

    def _encode_sign(
        self, data_name: str, ids_values: dict[int, int]
    ) -> dict[int, int]:
        for id_ in ids_values:
            model = self._id_to_model(id_)
            encoding_table = self.model_encoding_table.get(model)
            if encoding_table and data_name in encoding_table:
                sign_bit = encoding_table[data_name]
                ids_values[id_] = encode_sign_magnitude(ids_values[id_], sign_bit)

        return ids_values

    def _decode_sign(
        self, data_name: str, ids_values: dict[int, int]
    ) -> dict[int, int]:
        for id_ in ids_values:
            model = self._id_to_model(id_)
            encoding_table = self.model_encoding_table.get(model)
            if encoding_table and data_name in encoding_table:
                sign_bit = encoding_table[data_name]
                ids_values[id_] = decode_sign_magnitude(ids_values[id_], sign_bit)

        return ids_values

    def _split_into_byte_chunks(self, value: int, length: int) -> list[int]:
        if length == 1:
            data = [value]
        elif length == 2:
            data = [value & 0xFF, (value >> 8) & 0xFF]
        elif length == 4:
            data = [
                value & 0xFF,
                (value >> 8) & 0xFF,
                (value >> 16) & 0xFF,
                (value >> 24) & 0xFF,
            ]
        return data

    def _broadcast_ping(self) -> tuple[dict[int, int], int]:
        data_list: dict[int, int] = {}

        status_length = 6

        rx_length = 0
        wait_length = status_length * sts.MAX_ID

        txpacket = [0] * 6

        tx_time_per_byte = (1000.0 / self.port_handler.getBaudRate()) * 10.0

        txpacket[sts.PKT_ID] = sts.BROADCAST_ID
        txpacket[sts.PKT_LENGTH] = 2
        txpacket[sts.PKT_INSTRUCTION] = sts.INST_PING

        result = self.packet_handler.txPacket(self.port_handler, txpacket)
        if result != sts.COMM_SUCCESS:
            self.port_handler.is_using = False
            return data_list, result

        self.port_handler.setPacketTimeoutMillis(
            (wait_length * tx_time_per_byte) + (3.0 * sts.MAX_ID) + 16.0
        )

        rxpacket = []
        while not self.port_handler.isPacketTimeout() and rx_length < wait_length:
            rxpacket += self.port_handler.readPort(wait_length - rx_length)
            rx_length = len(rxpacket)

        self.port_handler.is_using = False

        if rx_length == 0:
            return data_list, sts.COMM_RX_TIMEOUT

        while True:
            if rx_length < status_length:
                return data_list, sts.COMM_RX_CORRUPT

            for idx in range(0, (rx_length - 1)):
                if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
                    break

            if idx == 0:
                checksum = 0
                for idx in range(2, status_length - 1):
                    checksum += rxpacket[idx]

                checksum = ~checksum & 0xFF
                if rxpacket[status_length - 1] == checksum:
                    result = sts.COMM_SUCCESS
                    data_list[rxpacket[sts.PKT_ID]] = rxpacket[sts.PKT_ERROR]

                    del rxpacket[0:status_length]
                    rx_length = rx_length - status_length

                    if rx_length == 0:
                        return data_list, result
                else:
                    result = sts.COMM_RX_CORRUPT
                    del rxpacket[0:2]
                    rx_length = rx_length - 2
            else:
                del rxpacket[0:idx]
                rx_length = rx_length - idx

    def broadcast_ping(
        self, num_retry: int = 0, raise_on_error: bool = False
    ) -> dict[int, int] | None:
        self._assert_protocol_is_compatible("broadcast_ping")
        for n_try in range(1 + num_retry):
            ids_status, comm = self._broadcast_ping()
            if self._is_comm_success(comm):
                break
            logger.debug(f"Broadcast ping failed on port '{self.port}' ({n_try=})")
            logger.debug(self.packet_handler.getTxRxResult(comm))

        if not self._is_comm_success(comm):
            if raise_on_error:
                raise ConnectionError(self.packet_handler.getTxRxResult(comm))
            return None

        ids_errors = {
            id_: status for id_, status in ids_status.items() if self._is_error(status)
        }
        if ids_errors:
            display_dict = {
                id_: self.packet_handler.getRxPacketError(err)
                for id_, err in ids_errors.items()
            }
            logger.error(
                f"Some motors found returned an error status:\n{pformat(display_dict, indent=4)}"
            )

        return self._read_model_number(list(ids_status), raise_on_error)

    def _read_firmware_version(
        self, motor_ids: list[int], raise_on_error: bool = False
    ) -> dict[int, str]:
        firmware_versions = {}
        for id_ in motor_ids:
            firm_ver_major, comm, error = self._read(
                *FIRMWARE_MAJOR_VERSION, id_, raise_on_error=raise_on_error
            )
            if not self._is_comm_success(comm) or self._is_error(error):
                continue

            firm_ver_minor, comm, error = self._read(
                *FIRMWARE_MINOR_VERSION, id_, raise_on_error=raise_on_error
            )
            if not self._is_comm_success(comm) or self._is_error(error):
                continue

            firmware_versions[id_] = f"{firm_ver_major}.{firm_ver_minor}"

        return firmware_versions

    def _read_model_number(
        self, motor_ids: list[int], raise_on_error: bool = False
    ) -> dict[int, int]:
        model_numbers = {}
        for id_ in motor_ids:
            model_nb, comm, error = self._read(
                *SERVO_VERSION_NUMBER, id_, raise_on_error=raise_on_error
            )
            if not self._is_comm_success(comm) or self._is_error(error):
                continue

            model_numbers[id_] = model_nb

        return model_numbers
