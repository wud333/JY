#include "ros/ros.h"
#define PI 3.14159265
//Dynamixel Control
#include <test_thread/test_thread.h>


uint8_t dxl_error = 0;                          // Dynamixel error
uint8_t param_goal_position[4];
int dxl_comm_result = COMM_TX_FAIL;

int32_t dxl1_present_position = 0;
int32_t dxl2_present_position = 0;


bool dxl_addparam_result = false;                // addParam result
bool dxl_getdata_result = false;

//Declare object
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
double i = 0;

bool is_run = true; //전역변수
int Control_Cycle = 10; //ms

//시간 관련
double t = 0;
double dt = 10; //ms
double T = 3000; //1000ms=1s

double L1 = 0.12;
double L2 = 0.1;

int dxl_present_posi1 = 0;
int dxl_goal_posi1 = 1000;

struct End_point {
  double x;
  double y;
};

struct Joint {
  double TH1;
  double TH2;
};


//functions
void process(void);
void dxl_initailize(void);
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi);
void dxl_go(void);
int radian_to_tick(double radian);
void read_dxl_postion(void);
void dxl_add_param(void);
struct End_point EP_goal;
struct End_point get_present_XY(void);
int radian_to_tick1(double radian);
int radian_to_tick2(double radian);
double tick_to_radian_1(int tick);
double tick_to_radian_2(int tick);
struct Joint J_goal;
struct Joint Compute_IK(struct End_point EP);
void *p_function(void * data) {

  dxl_initailize();

  static struct timespec next_time;
//static struct timespec curr_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(is_run) //1하면 종료가 잘 안됨. is run으로
  {
    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + Control_Cycle * 1000000) % 1000000000;

    process();

    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next_time,NULL);
  }
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "thread_pub_node");
  ros::NodeHandle nh;

  pthread_t pthread; //배열 자료형 스레드2개 배열 스레드 한개만 쓸 거 변수로 바꿈
  int thr_id;
  int status;
  char p1[] = "thread_1";

  sleep(1); //1초 쉼

  thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1); //2
  //에러 확인
  if(thr_id < 0) {
    ROS_ERROR("pthread0 create error");
    exit(EXIT_FAILURE);
  }


  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("thread", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  is_run  = false;

  return 0;
}

void process(void) {

 static struct End_point target;

 static struct End_point EP_goal;
  EP_goal.x = 0;
  EP_goal.y = 0.15;

  //J_goal = Compute_IK(EP_goal);
  //set_dxl_goal(J_goal.TH1,J_goal.TH2);
  //dxl_go();
  //groupSyncWrite.clearParam();

  read_dxl_postion();
  static struct End_point E;
  E =  get_present_XY();
  ROS_INFO("x:%lf, y:%lf",E.x,E.y);

  if (t <= T) {

  target.x =  E.x + (EP_goal.x - E.x )*0.5*(1 - cos(PI* t/T));
  target.y =  E.y + (EP_goal.y - E.y )*0.5*(1 - cos(PI* t/T));

  static struct Joint joint_goal;
  joint_goal = Compute_IK(target);
  set_dxl_goal(radian_to_tick1(joint_goal.TH1), radian_to_tick2(joint_goal.TH2));
  dxl_go();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();

  t=t+dt;
  }
  else{
   // t = 0;
  }
}

//open port, set baud, torqeu on dxl 1,2
void dxl_initailize(void) {

  portHandler->openPort();

  portHandler->setBaudRate(BAUDRATE);

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_add_param();
}

//goal position 지정
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi) {

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_1_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_1_posi));

  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_2_posi));

  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
}

//통신보내주는 역할
void dxl_go(void) {
  dxl_comm_result = groupSyncWrite.txPacket();
}

struct Joint Compute_IK(struct End_point EP) {

  //IK soluttion
  /*
  struct Joint J;
  J.TH1 = atan2(sin(J.TH1), cos(J.TH2));
  J.TH2 = atan2(sqrt((1 - pow(cos(J.TH1), 2))), cos(J.TH2));
  return J;
  */

  double x = EP.x;
  double y = EP.y;
  double alpha = atan2(y,x);
  double L = sqrt(pow(x,2)+pow(y,2));
  double beta = acos((pow(L1,2)+pow(L2,2)-pow(L,2))/(2*L1*L2));
  double gamma = atan2(x,y);
  double delta = acos((pow(L1,2)+pow(L,2)-pow(L2,2))/(2*L1*L));

  double th2 = PI - beta;
  double th1 = (PI)/2 - gamma - delta;

  printf("%f , %f",th1,th2);

  struct Joint J;
  J.TH1 = th1;
  J.TH2 = th2;
  return J;
}


struct End_point Compute_FK(struct Joint J) {

  //FK soluttion

  struct End_point E;￣
  E.x = L1 * cos(J.TH1) + L2 * cos(J.TH1 + J.TH2);
  E.y = L1 * sin(J.TH1) + L2 * sin(J.TH1 + J.TH2);
  return E;
}

int radian_to_tick1(double radian){
  int tick = radian*(2048/PI);
  tick = (tick - 1024);
  if(tick < 0){
    tick += 4096;
  }
  return tick;
}
￣
int radian_to_tick2(double radian){
  int tick = radian*(2048/PI);
  if(tick < 0){
    tick += 4096;
  }
  return tick;
}

double tick_to_radian_1(int tick){
   double radian = (PI/(double)2048)*(tick+1024);
   return radian;
}

double tick_to_radian_2(int tick){
  double radian = (PI/(double)2048)*tick;
  return radian;

}

void read_dxl_postion(void){

 dxl_comm_result = groupSyncRead.txRxPacket();
 dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
 dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
}

void dxl_add_param(void){

  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    }
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
    }
  }

struct End_point get_present_XY(void){

  struct Joint j;
  j.TH1 = tick_to_radian_1(dxl1_present_position) ;
  j.TH2 = tick_to_radian_2(dxl2_present_position);
  return Compute_FK(j);
}






