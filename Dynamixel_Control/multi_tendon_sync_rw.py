import os
import atexit
import numpy as np

from dynamixel_sdk import * # Uses Dynamixel SDK library
from .util import signed_int_to_4_bytes, four_bytes_unsigned_to_signed_int, mm_to_rad, \
    rad_to_mm, rad_per_sec_to_rpm, get_prepend_string

class MultiTendonSyncRW:
    """ Base class for multiple Dynamixel-driven tendon, with group sync read/write across all servos. """

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
        LEN_GOAL_POSITION           = 4            # Data Byte Length
        ADDR_PRESENT_POSITION       = 132
        LEN_PRESENT_POSITION        = 4            # Data Byte Length
        DXL_MINIMUM_POSITION_VALUE  = -1048575     # -256[rev]
        DXL_MAXIMUM_POSITION_VALUE  = 1048575      # 256[rev]
        DXL_TICK2RAD                = 0.001533981  # 2*pi/4096
        BAUDRATE                    = 3000000      # Max Baudrate of XL-430 is 4.5Mbps
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


    ADDR_OPERATING_MODE         = 11
    OP_EXTENDED_POSITION        = 4     # The value of Extended Position Control Mode that can be set through the Operating Mode (11)
    OP_CURRENT_BASED_POSITION   = 5     # The value of Current Based Position Control Mode that can be set through the Operating Mode (11)
    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold
    ADDR_PROFILE_VELOCITY      = 112
    LEN_PROFILE_VELOCITY       = 4      # Data Byte Length
    PROFILE_VELOCITY_UNIT       = 0.229 # rpm


    def __init__(self, servo_ids=np.array([4,6,5,1,3,2,10,12,11,7,9,8]), spool_radii_mm=np.array([6.875,6.875,6.875,6.875,6.875,6.875,6.875,6.875,6.875,6.875,6.875,6.875]), 
                 zero_offsets_rad=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), device_name='COM10'):
        # device_name: Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        
        self.max_speed_mm_per_sec = 2.0 * spool_radii_mm
        self.servo_ids = servo_ids
        self.spool_radii_mm = spool_radii_mm
        self.zero_offsets_tick = self._rad_to_tick(zero_offsets_rad)

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(device_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite for target position
        self.groupSyncWritePos = GroupSyncWrite(self.portHandler, self.packetHandler, 
                                                self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)

        # Initialize GroupSyncWrite for target velocity
        self.groupSyncWriteVel = GroupSyncWrite(self.portHandler, self.packetHandler, 
                                                self.ADDR_PROFILE_VELOCITY, self.LEN_PROFILE_VELOCITY)

        # Initialize GroupSyncRead instace for current position
        self.groupSyncReadPos = GroupSyncRead(self.portHandler, self.packetHandler, 
                                              self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()
        
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Set operating mode to extended position control mode
        for servo_id in self.servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, 
                                                                    self.ADDR_OPERATING_MODE, self.OP_EXTENDED_POSITION)
            self._confirm_comm_result(dxl_comm_result, dxl_error, servo_id)
        # Enable Dynamixel Torque
        for servo_id in self.servo_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, 
                                                                           self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            self._confirm_comm_result(dxl_comm_result, dxl_error, servo_id)
        # Add parameter storage for reading present position value
        for servo_id in self.servo_ids:
            dxl_addparam_result = self.groupSyncReadPos.addParam(servo_id)
            if dxl_addparam_result != True:
                print(get_prepend_string(servo_id) + f"groupSyncReadPos addparam failed")
                quit()

        # Add parameter storage for writing present position value
        for servo_id in self.servo_ids:
            dxl_addparam_result = self.groupSyncWritePos.addParam(servo_id, signed_int_to_4_bytes(0))
            if dxl_addparam_result != True:
                print(get_prepend_string(servo_id) + f"groupSyncWritePos addparam failed")
                quit()

        # Add parameter storage for writing present velocity value
        for servo_id in self.servo_ids:
            dxl_addparam_result = self.groupSyncWriteVel.addParam(servo_id, signed_int_to_4_bytes(0))
            if dxl_addparam_result != True:
                print(get_prepend_string(servo_id) + f"groupSyncWriteVel addparam failed")
                quit()
        
        atexit.register(self.cleanup)
        print("All servos initialized")
    
    def _tick_to_rad(self, tick):
        return tick * self.DXL_TICK2RAD
    
    def _rad_to_tick(self, rad):
        return (rad / self.DXL_TICK2RAD).astype(int)

    def _confirm_comm_result(self, dxl_comm_result, dxl_error, servo_id):
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{servo_id:03d}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{servo_id:03d}] {self.packetHandler.getRxPacketError(dxl_error)}")

    def cleanup(self):
        self.groupSyncReadPos.clearParam()
        self.groupSyncWritePos.clearParam()
        self.groupSyncWriteVel.clearParam()
        for servo_id in self.servo_ids:
            # Disable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, 
                                                                           self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self._confirm_comm_result(dxl_comm_result, dxl_error, servo_id)
        self.portHandler.closePort()
   
    def get_tendons_tick(self):
        # Syncread present position
        dxl_comm_result = self.groupSyncReadPos.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        current_ticks = np.zeros_like(self.servo_ids)
        for i, servo_id in enumerate(self.servo_ids):
            dxl_getdata_result = self.groupSyncReadPos.isAvailable(servo_id, self.ADDR_PRESENT_POSITION, 
                                                                   self.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print(get_prepend_string(servo_id) + f"groupSyncReadPos getdata failed")
                return False
            current_ticks[i] = four_bytes_unsigned_to_signed_int(
                self.groupSyncReadPos.getData(servo_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION))
        return current_ticks - self.zero_offsets_tick

    def get_tenons_rad(self):
        return self._tick_to_rad(self.get_tendons_tick())

    def get_tendons_mm(self):
        return rad_to_mm(self.get_tenons_rad(), self.spool_radii_mm)

    def set_zero_offsets_to_current_position(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_ids
        current_ticks = self.get_tendons_tick()
        for servo_id in servo_ids:
            idx = np.where(self.servo_ids == servo_id)[0][0]
            self.zero_offsets_tick[idx] += current_ticks[idx]

    def async_set_tendons_tick(self, goal_ticks):
        goal_ticks = goal_ticks + self.zero_offsets_tick
        for servo_id, goal_tick in zip(self.servo_ids, goal_ticks):
            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_changeparam_result = self.groupSyncWritePos.changeParam(servo_id, signed_int_to_4_bytes(goal_tick))
            if dxl_changeparam_result != True:
                print(get_prepend_string(servo_id) + f"groupSyncWritePos addparam failed")
                return False
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWritePos.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        return True
    
    def async_set_tendons_rad(self, goal_rad):
        goal_ticks = self._rad_to_tick(goal_rad)
        return self.async_set_tendons_tick(goal_ticks)

    def async_set_tendons_mm(self, goal_mm):
        goal_ticks = self._rad_to_tick(mm_to_rad(goal_mm, self.spool_radii_mm))
        return self.async_set_tendons_tick(goal_ticks)
    
    def async_set_tendons_mm_together(self, goal_mm):
        ''' Set the goal tendon lengths and speeds to reach the goal tendon lengths together
            WARNING: this function changes the target speeds of the servos'''
        current_tendons_mm = self.get_tendons_mm()
        max_joint_speed = np.max(np.abs(goal_mm - current_tendons_mm))
        if max_joint_speed > 0.01:  # avoid devision by 0 error
            goal_tendon_speeds = self.max_speed_mm_per_sec * np.abs(goal_mm - current_tendons_mm) / np.max(np.abs(goal_mm - current_tendons_mm))
        else:
            goal_tendon_speeds = self.max_speed_mm_per_sec * np.ones_like(goal_mm)
        if not self.set_tendons_speeds_mm_per_sec(goal_tendon_speeds):
            return False
        return self.async_set_tendons_mm(goal_mm)

    def async_set_tendons_home(self):
        return self.async_set_tendons_mm(np.zeros_like(self.servo_ids))

    def set_tendons_speeds_rpm(self, goal_speeds_rpm):
        for servo_id, goal_rpm in zip(self.servo_ids, goal_speeds_rpm):
            if goal_rpm < 0:
                print(get_prepend_string(servo_id) + f"WARNING: goal speed should be positive")
            dxl_changeparam_result = self.groupSyncWriteVel.changeParam(servo_id, 
                                                        signed_int_to_4_bytes(int(goal_rpm/self.PROFILE_VELOCITY_UNIT)))
            if dxl_changeparam_result != True:
                print(get_prepend_string(servo_id) + f"groupSyncWritePos addparam failed")
                return False
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWriteVel.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        return True

    def set_tendons_speeds_mm_per_sec(self, goal_speeds_mm_per_sec):
        goal_speeds_rpm = rad_per_sec_to_rpm(mm_to_rad(goal_speeds_mm_per_sec, self.spool_radii_mm))
        return self.set_tendons_speeds_rpm(goal_speeds_rpm)

def main():
    pass

if __name__ == "__main__":
    main()
