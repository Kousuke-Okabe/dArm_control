#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <dynamixel_sdk.h>            // Uses Dynamixel SDK library
#include <termios.h>

#define STDIN_FILENO 0

// Control table address
#define ADDR_TORQUE_ENABLE              64                 // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION              116
#define ADDR_DATA                       126
#define ADDR_PRESENT_CURRENT            126
#define ADDR_PRESENT_VELOCITY           128
#define ADDR_PRESENT_POSITION           132

// Data Byte Length
#define LEN_GOAL_POSITION               4
#define LEN_DATA                        10
#define LEN_PRESENT_CURRENT             2
#define LEN_PRESENT_VELOCITY            4
#define LEN_PRESENT_POSITION            4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#2 ID: 3
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_POSITION_OFFSET_VALUE       2048                // Offset value for joint angle = 0[rad]


class dArm : public hardware_interface::RobotHW
{
public:
  dArm();
  ~dArm();
  void read();
  void write();

private:
  int getch();
  void allocate_goal_position(uint8_t* param_goal_position, int dxl_goal_position);

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[3] = {0.0, 0.0, 0.0};
  double pos[3];
  double vel[3];
  double eff[3];

  //DYNAMIXEL

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite = dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

  // // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead = dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_DATA, LEN_DATA);

  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_getdata_result = false;                  // GetParam result
  bool dxl_addparam_result = false;                 // addParam result
  uint8_t dxl_error = 0;                            // Dynamixel error

  int32_t dxl1_present_position, dxl2_present_position, dxl3_present_position;          // Present position
  int32_t dxl1_present_velocity, dxl2_present_velocity, dxl3_present_velocity;          // Present velocity
  signed short int dxl1_present_current, dxl2_present_current, dxl3_present_current;    // Present cuurrent

  uint8_t param_goal_position_1[4], param_goal_position_2[4], param_goal_position_3[4];

};

dArm::dArm()
{ 
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_q1("q1", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_q1);

  hardware_interface::JointStateHandle state_handle_q2("q2", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_q2);

  hardware_interface::JointStateHandle state_handle_q3("q3", &pos[2], &vel[2], &eff[2]);
  jnt_state_interface.registerHandle(state_handle_q3);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle vel_handle_q1(jnt_state_interface.getHandle("q1"), &cmd[0]);
  jnt_vel_interface.registerHandle(vel_handle_q1);

  hardware_interface::JointHandle vel_handle_q2(jnt_state_interface.getHandle("q2"), &cmd[1]);
  jnt_vel_interface.registerHandle(vel_handle_q2);

  hardware_interface::JointHandle vel_handle_q3(jnt_state_interface.getHandle("q3"), &cmd[2]);
  jnt_vel_interface.registerHandle(vel_handle_q3);
  
  registerInterface(&jnt_vel_interface);

  // Open port
  if (portHandler->openPort())
  {
      ROS_INFO("Succeeded to open the port!");
  }
  else
  {
      ROS_INFO("Failed to open the port!");
      ROS_INFO("Press any key to terminate...");
      getch();
      return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
      ROS_INFO("Succeeded to change the baudrate!");
  }
  else
  {
      ROS_INFO("Failed to change the baudrate!");
      ROS_INFO("Press any key to terminate...");
      getch();
      return;
  }

  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
      ROS_INFO("Dynamixel#%d has been successfully connected ", DXL1_ID);
  }
  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
      ROS_INFO("Dynamixel#%d has been successfully connected", DXL2_ID);
  }
  // Enable Dynamixel#3 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
      ROS_INFO("Dynamixel#%d has been successfully connected ", DXL3_ID);
  }

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
  {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
      return;
  }
  // Add parameter storage for Dynamixel#2 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
  {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
      return;
  }
  // Add parameter storage for Dynamixel#3 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL3_ID);
  if (dxl_addparam_result != true)
  {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL3_ID);
      return;
  }
}

dArm::~dArm(){
  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
  }
  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
  }
  // Disable Dynamixel#3 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
      ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();
}

void dArm::read(){
  // Syncread
  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
      ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (groupSyncRead.getError(DXL1_ID, &dxl_error))
  {
      ROS_INFO("[ID:%03d] %s", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
  }
  else if (groupSyncRead.getError(DXL2_ID, &dxl_error))
  {
      ROS_INFO("[ID:%03d] %s", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
  }
  else if (groupSyncRead.getError(DXL3_ID, &dxl_error))
  {
      ROS_INFO("[ID:%03d] %s", DXL3_ID, packetHandler->getRxPacketError(dxl_error));
  }

  // Check if groupsyncread data of Dynamixel#1 is available
  dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_DATA, LEN_DATA);
  if (dxl_getdata_result != true)
  {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
      return;
  }
  // Check if groupsyncread data of Dynamixel#2 is available
  dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_DATA, LEN_DATA);
  if (dxl_getdata_result != true)
  {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
      return;
  }
  // Check if groupsyncread data of Dynamixel#3 is available
  dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_DATA, LEN_DATA);
  if (dxl_getdata_result != true)
  {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL3_ID);
      return;
  }

  // Get Dynamixel#1 present position value
  dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
  // Get Dynamixel#2 present position value
  dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
  // Get Dynamixel#3 present position value
  dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

  // Get Dynamixel#1 present velocity value
  dxl1_present_velocity = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  // Get Dynamixel#2 present velocity value
  dxl2_present_velocity = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  // Get Dynamixel#3 present velocity value
  dxl3_present_velocity = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);

  // Get Dynamixel#1 present current value
  dxl1_present_current = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
  // Get Dynamixel#2 present current value
  dxl2_present_current = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
  // Get Dynamixel#3 present current value
  dxl3_present_current = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);


  pos[0] = dxl1_present_position;
  pos[1] = dxl2_present_position;
  pos[2] = dxl3_present_position;

  vel[0] = dxl1_present_velocity;
  vel[1] = dxl2_present_velocity;
  vel[2] = dxl3_present_velocity;

  eff[0] = dxl1_present_current;
  eff[1] = dxl2_present_current;
  eff[2] = dxl3_present_current;

}

void dArm::write(){
  ROS_INFO_STREAM("q1 command " << cmd[0] << " q2 command " << cmd[1] << " q3 command " << cmd[2]);

  allocate_goal_position(param_goal_position_1, (int)cmd[0]);
  allocate_goal_position(param_goal_position_2, (int)cmd[1]);
  allocate_goal_position(param_goal_position_3, (int)cmd[2]);

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position_1);
  if (dxl_addparam_result != true)
  {
  fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
  return;
  }
  // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2);
  if (dxl_addparam_result != true)
  {
  fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
  return;
  }
  // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position_3);
  if (dxl_addparam_result != true)
  {
  fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL3_ID);
  return;
  }

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

int dArm::getch(){
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

void dArm::allocate_goal_position(uint8_t* param_goal_position, int dxl_goal_position){
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));
        return;
}