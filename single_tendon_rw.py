import os
import atexit
import numpy as np

from dynamixel_sdk import * # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class SingleTendon:
    """ Base class for one Dynamixel-driven tendon. """

    #********* DYNAMIXEL Model definition *********
    #***** (Use only one definition at a time) *****
    MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
    # MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
    # MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
    # MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
    # MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
    # MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


    # Control table address
    if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
        ADDR_TORQUE_ENABLE          = 64
        ADDR_GOAL_POSITION          = 116
        ADDR_PRESENT_POSITION       = 132
        DXL_MINIMUM_POSITION_VALUE  = -1048575     # -256[rev]
        DXL_MAXIMUM_POSITION_VALUE  = 1048575      # 256[rev]
        DXL_TICK2RAD                = 0.001533981   # 2*pi/4096
        BAUDRATE                    = 3000000   # Max Baudrate of XL-430 is 4.5Mbps
    elif MY_DXL == 'PRO_SERIES':
        ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 596
        ADDR_PRESENT_POSITION       = 611
        DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 57600
    elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
        ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 564
        ADDR_PRESENT_POSITION       = 580
        DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 57600
    elif MY_DXL == 'XL320':
        ADDR_TORQUE_ENABLE          = 24
        ADDR_GOAL_POSITION          = 30
        ADDR_PRESENT_POSITION       = 37
        DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
        BAUDRATE                    = 57600

    # DYNAMIXEL Protocol Version (1.0 / 2.0)
    # https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0

    # Factory default ID of all DYNAMIXEL is 1
    DXL_ID                      = 1

    # Use the actual port assigned to the U2D2.
    # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME                  = '/dev/ttyUSB0'

    ADDR_OPERATING_MODE         = 11
    OP_EXTENDED_POSITION        = 4     # The value of Extended Position Control Mode that can be set through the Operating Mode (11)
    OP_CURRENT_BASED_POSITION   = 5     # The value of Current Based Position Control Mode that can be set through the Operating Mode (11)
    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold
    ADDDR_PROFILE_VELOCITY      = 112
    PROFILE_VELOCITY_UNIT       = 0.229 # rpm

    ESC_ASCII_VALUE             = 0x1b
    SPACE_ASCII_VALUE           = 0x20

    def __init__(self, servo_id=1, spool_radius_mm=4.0, 
                 zero_offset_rad=0.0,
                 device_name=DEVICENAME,
                 operating_mode=OP_EXTENDED_POSITION):

        self.servo_id = servo_id
        self.spool_radius_mm = spool_radius_mm
        self.zero_offset_rad = zero_offset_rad
        self.operating_mode = operating_mode
        self.speed_rpm = 10.0 # rev/min (default)

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(device_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate to change defaults
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set operating mode to extended position control mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.servo_id, self.ADDR_OPERATING_MODE, self.operating_mode)
        self._confirm_comm_result(dxl_comm_result, dxl_error)

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.servo_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        self._confirm_comm_result(dxl_comm_result, dxl_error)
        print(f"[ID:{self.servo_id:03d}] has been successfully connected")

        atexit.register(self.cleanup)

    def _mm_to_rad(self, mm):
        return mm / self.spool_radius_mm

    def _rad_to_mm(self, rad):
        return rad * self.spool_radius_mm

    def _rpm_to_rad_per_sec(self, rpm):
        return rpm * 2 * np.pi / 60

    def _rad_per_sec_to_rpm(self, rad_per_sec):
        return rad_per_sec * 60 / (2 * np.pi)

    def _confirm_comm_result(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"{self.packetHandler.getRxPacketError(dxl_error)}")

    def cleanup(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.servo_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self._confirm_comm_result(dxl_comm_result, dxl_error)
        # Close port
        self.portHandler.closePort()

    def set_zero_offset_rad(self, zero_offset_rad: float):
        self.zero_offset_rad = zero_offset_rad

    def set_zero_offset_mm(self, zero_offset_mm: float):
        self.zero_offset_rad = zero_offset_mm / self.spool_radius_mm

    def set_zero_offset_rad_to_current_position(self):
        self.zero_offset_rad -= self.get_joint_rad()

    def set_max_speed_rpm(self, max_speed_rpm: float):
        if max_speed_rpm < 0:
            raise Exception('max_speed_rpm must be positive')
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.servo_id, self.ADDDR_PROFILE_VELOCITY, int(max_speed_rpm/self.PROFILE_VELOCITY_UNIT))
        self._confirm_comm_result(dxl_comm_result, dxl_error)
        self.speed_rpm = max_speed_rpm

    def set_max_speed_mm_per_sec(self, max_speed_mm_per_sec: float):
        # if input is a float, set all servos to the same speed
        max_speed_rpm = self._rad_per_sec_to_rpm(self._mm_to_rad(max_speed_mm_per_sec))
        self.set_max_speed_rpm(max_speed_rpm)

    def async_set_joint_rad(self, goal_position_rad):
        goal_position_rad -= self.zero_offset_rad
        goal_position_tick = int(goal_position_rad / self.DXL_TICK2RAD)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.servo_id, self.ADDR_GOAL_POSITION, goal_position_tick)
        self._confirm_comm_result(dxl_comm_result, dxl_error)

    def set_joint_rad(self, goal_position_rad, wait_time=None, timeout=1, tol_rad=0.025):
        self.async_set_joint_rad(goal_position_rad)
        if wait_time is None:
            wait_time = np.abs(goal_position_rad - self.get_joint_rad()) \
                        / self._rpm_to_rad_per_sec(self.speed_rpm)

        start_time = time.time()
        while True:
            if abs(self.get_joint_rad() - goal_position_rad) < tol_rad:
                break
            if time.time() - start_time > wait_time + timeout:
                print(f"{self.servo_id:03d} timeout while waiting for joint to reach goal position")
                break
            time.sleep(0.1)

    def get_joint_rad(self) -> float:
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.servo_id, self.ADDR_PRESENT_POSITION)
        self._confirm_comm_result(dxl_comm_result, dxl_error)
        # dxl_present_position is a signed 32bit integer
        # convert from 32bit unsigned to 32bit signed
        if dxl_present_position > 2147483647:
            dxl_present_position = dxl_present_position - 4294967295 - 1
        return dxl_present_position * self.DXL_TICK2RAD + self.zero_offset_rad

    def async_set_joint_mm(self, goal_positions_mm) -> None:
        # set servo positions plus zero offsets
        goal_position_rad = goal_positions_mm / self.spool_radius_mm
        self.async_set_joint_rad(goal_position_rad)

    def set_joint_mm(self, goal_positions_mm, wait_time=None, timeout=1, tol_mm=0.1) -> None:
        # set servo positions plus zero offsets
        goal_position_rad = self._mm_to_rad(goal_positions_mm)
        tol_rad = self._mm_to_rad(tol_mm)
        self.set_joint_rad(goal_position_rad, wait_time, timeout, tol_rad)

    def get_joint_mm(self) -> float:
        # read servo positions minus zero offsets
        joint_rad = self.get_joint_rad()
        return joint_rad * self.spool_radius_mm

