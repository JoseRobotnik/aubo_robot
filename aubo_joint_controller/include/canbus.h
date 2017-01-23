#ifndef CANBUS_H
#define CANBUS_H

void *m_hCan; /**< Handle of CAN bus */

int JointCanInit(const char* DeviceNode);
int CanInit(const char* szDeviceNode);

int JointCanUninit();

int JointRecvack(unsigned short int joint, unsigned char cmd, unsigned char index);

int can_setTagPosRadio(int joint, float radio, unsigned char jointWMode, unsigned short int jointType);
float JointReadPos(int joint, int cmd, int opt, unsigned char model);
int JointSetPos(int joint, float angle, int cmd, unsigned char model, unsigned char writeype);
int JointSetVel(int joint, float angle, int cmd, unsigned char model, unsigned char writeype);
int JointSetMaxRPM(int joint, int rpm, unsigned char writeype);

int JointSendMsg(unsigned short int joint, unsigned char cmd, unsigned char index, unsigned char *data, int data_length);
int JointReadMsg(int joint, unsigned char addr, char length, unsigned char *data);

unsigned short int read_can_data(int joint, unsigned char addr);
int write_can_data(int joint, unsigned char addr, unsigned short int num);

int joint_enable(int joint, int opt);

float j_read_zero_posoffset(int joint, int cmd, unsigned char model);
int j_wead_zero_posoffset(int joint, float angle, int cmd, unsigned char model, unsigned char writeype);

void *can_read(void *);

unsigned short int JointReadWordData(int joint, unsigned char addr);
float JointReadTemperature(int joint);
#endif // CANBUS_H
