FIRMWARE_MAJOR_VERSION = (0, 1)
FIRMWARE_MINOR_VERSION = (1, 1)
SERVO_VERSION_NUMBER = (3, 2)

MAGNETIC_ENCODED_STS_MEMORY_TABLE = {
    # EPROM
    "Firmware_Major_Version": FIRMWARE_MAJOR_VERSION,  # read-only
    "Firmware_Minor_Version": FIRMWARE_MINOR_VERSION,  # read-only
    "Servo_Main_Version_Number": SERVO_VERSION_NUMBER,  # read-only
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Angle_Limit": (9, 2),
    "Max_Maximum_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Phase": (18, 1),
    "Uninstallation_Condition": (19, 1),
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "Points_Limit": (25, 1),
    "CW_Intensitive_Area": (26, 1),
    "CCW_Intensitive_Area": (27, 1),
    "Protection_Current": (28, 2),
    "Angular_Resolution": (30, 1),
    "Position_Correction": (31, 2),
    "Operating_Mode": (33, 1),
    "Protective_Torque": (34, 1),
    "Protection_Time": (35, 1),
    "Overload_Torque": (36, 1),
    "Speed_closed_loop_P_proportional_coefficient": (37, 1),
    "Over_Current_Protection_Time": (38, 1),
    "Velocity_closed_loop_I_integral_coefficient": (39, 1),
    # SRAM
    "Torque_Switch": (40, 1),
    "Acceleration": (41, 1),
    "Target_Location": (42, 2),
    "Runtime": (44, 2),
    "Running_Speed": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock_Symbol": (55, 1),
    "Current_Location": (56, 2),  # read-only
    "Current_Speed": (58, 2),  # read-only
    "Current_Load": (60, 2),  # read-only
    "Current_Voltage": (62, 1),  # read-only
    "Current_Temperature": (63, 1),  # read-only
    "Async_Write_Flag": (64,1),     # read-only
    "Servo_Status": (65, 1),  # read-only
    "Mobile_sign": (66, 1),  # read-only
    "Current_Current": (69, 2),  # read-only
    # "Goal_Position_2": (71, 2),  # read-only
    # # Factory
    # "Moving_Velocity": (80, 1),
    # "Moving_Velocity_Threshold": (80, 1),
    # "DTs": (81, 1),  # (ms)
    # "Velocity_Unit_factor": (82, 1),
    # "Hts": (83, 1),  # (ns) valid for firmware >= 2.54, other versions keep 0
    # "Maximum_Velocity_Limit": (84, 1),
    # "Maximum_Acceleration": (85, 1),
    # "Acceleration_Multiplier ": (86, 1),  # Acceleration multiplier in effect when acceleration is 0
}
ST_SERIES_BAUDRATE_TABLE = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    57_600: 5,
    38_400: 6,
    19_200: 7,
}

MODEL_CONTROL_TABLE = {
    "st3215": MAGNETIC_ENCODED_STS_MEMORY_TABLE,
}

MODEL_RESOLUTION = {
    "st3215": 4096,
}

MODEL_BAUDRATE_TABLE = {
    "st3215": ST_SERIES_BAUDRATE_TABLE,
}

# Sign-Magnitude encoding bits
ST_SERIES_ENCODINGS_TABLE = {
    "Present_Load": 10,
    "Homing_Offset": 11,
    "Goal_Position": 15,
    "Goal_Velocity": 15,
    "Goal_Speed": 15,
    "Present_Position": 15,
    "Present_Velocity": 15,
    "Present_Speed": 15,
}

MODEL_ENCODING_TABLE = {
    "sts3215": ST_SERIES_ENCODINGS_TABLE,
}

SCAN_BAUDRATES = [
    4_800,
    9_600,
    14_400,
    19_200,
    38_400,
    57_600,
    115_200,
    128_000,
    250_000,
    500_000,
    1_000_000,
]

MODEL_NUMBER_TABLE = {
    "sts3215": 68,
}

MODEL_PROTOCOL = {
    "st3215": 0,
}