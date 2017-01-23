#ifndef ROBOTCONNECT_H
#define ROBOTCONNECT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{ JOINT_CANBUS = 0, JOINT_MACBUS } JointBusType;

typedef struct JOINT_POINT
{
    int joint_id;			/** Joint ID */ 
    float jointpos;			/** Angle or Radian */
    unsigned short int joint_type;	/** J60/J80 type */  
    unsigned char joint_w_mode;		/** R/W Mode */	
}roadpoint;
 

/**
*	init CAN Bus
*	总线类型(Bus type) JOINT_MACBUS/JOINT_CANBUS
*	return 1 success  0 fail
**/
int robotconnect_init(const char* szDeviceNode, const JointBusType type);


/**
*	关闭总线设备(close CAN Bus)
*	总线类型 JOINT_MACBUS/JOINT_CANBUS
*	return 1 success   0 fail
**/
int robotconnect_destroy(const JointBusType type);


/**
*	Write joint radian data (old interface)
*	joint: Joint ID
*	radio: 弧度数值(radian value)
*	writeype: read/write mode 
*	model: 机械臂(joint) MODEL_TYPE_J60/MODEL_TYPE_J80
*	Return: 1 success  0 fail 
**/
int joint_roadpoint_set(int joint, float radio, unsigned short int jointType, unsigned char jointWMode);

/** joint_roadpoint_set alias */
#define	joint_set_radian joint_roadpoint_set


/**
*	Read joint data
*	joint: Joint ID
*	cmd:   角度或弧度(angle or radian) JOINT_ANGLE/JOINT_RADIAN 
*	model: 机械臂 MODEL_TYPE_J60/MODEL_TYPE_J80
*	Return: succes,return joint angle or radian,  fail return 0 
**/
float joint_readpos(int joint, int cmd, int opt, unsigned char model);


/**
*	Write angle/radian joint data 
*	joint: Joint ID
*	angle: 角度或弧度数值  Input angle or radian
*	cmd:   角度或弧度 JOINT_ANGLE/JOINT_RADIAN 
*	model: 机械臂 MODEL_TYPE_J60 and MODEL_TYPE_J80
*	writeype: read/write mode 
*	Return: 1 success  0 fail
**/
int joint_setpos(int joint, float angle, int cmd, unsigned char model, unsigned char writeype);
int joint_setvel(int joint, float angle, int cmd, unsigned char model, unsigned char writeype);
int joint_setmaxrpm(int joint, int rpm, unsigned char writeype);


/**
*	CAN总线设备打开/关闭状态 CAN Device Open/Close State.
*	return: 1 open can , 0 close can
**/
int get_can_state(void);

int joint_brake_enable(int joint);
int joint_brake_disable(int joint);

//#define Jointstate struct jointstate
//Jointstate{
//	int	CurrentI;	/** Current of driver */
//	int	SpeedMoto;	/** Speed of driver */
//	float	jointPosJ;	/** Current position in radian */
//	float	CurrentVol;	/** Rated voltage of motor. Unit: mV */
//	float	CurrentTemp;	/** Current temprature of joint */
//};

//Jointstate get_joint_state(void);

#ifdef __cplusplus
}
#endif

#endif 
