#ifndef _UM7_REGISTERMAP_H_
#define _UM7_REGISTERMAP_H_

#define CREG_COM_SETTINGS 		0x00
#define CREG_COM_RATES1 		0x01
#define CREG_COM_RATES2 		0x02
#define CREG_COM_RATES3 		0x03
#define CREG_COM_RATES4 		0x04
#define CREG_COM_RATES5 		0x05
#define CREG_COM_RATES6 		0x06
#define CREG_COM_RATES7 		0x07
#define CREG_MISC_SETTINGS		0x08

#define CREG_HOME_NORTH			0x09
#define CREG_HOME_EAST			0x0A
#define CREG_HOME_UP			0x0B

#define DREG_HEALTH 			0x55

#define DREG_GYRO_RAW_XY  		0x56
#define DREG_GYRO_RAW_Z  		0x57
#define DREG_GYRO_RAW_TIME		0x58

#define DREG_ACCEL_RAW_XY		0x59
#define DREG_ACCEL_RAW_Z		0x5A
#define DREG_ACCEL_RAW_TIME             0x5B

#define DREG_TEMPERATURE 		0x5F

#define DREG_ALL_PROC  			0x61

#define DREG_GYRO_PROC_X  		0x61
#define DREG_GYRO_PROC_Y  		0x62
#define DREG_GYRO_PROC_Z  		0x63
#define DREG_GYRO_PROC_TIME		0x64

#define DREG_ACCEL_PROC_X		0x65
#define DREG_ACCEL_PROC_Y		0x66
#define DREG_ACCEL_PROC_Z		0x67
#define DREG_ACCEL_PROC_TIME            0x67

#define DREG_MAG_PROC_X 		0x69
#define DREG_MAG_PROC_Y 		0x6A
#define DREG_MAG_PROC_Z 		0x6B
#define DREG_MAG_PROC_TIME 		0x6C

#define DREG_QUAT_AB			0x6D
#define DREG_QUAT_CD			0x6E
#define DREG_QUAT_TIME			0x6F

#define DREG_EULER_PHI_THETA            0x70
#define DREG_EULER_PSI			0x71
#define DREG_EULER_PHI_THETA_DOT        0x72
#define DREG_EULER_PSI_DOT		0x73
#define DREG_EULER_TIME			0x74

#define DREG_VELOCITY_N			0x79
#define DREG_VELOCITY_E			0x7A
#define DREG_VELOCITY_UP		0x7B
#define DREG_VELOCITY_TIME		0x7C

#define PT_HAS_DATA 			0b10000000
#define PT_IS_BATCH 			0b01000000
#define PT_BL_3		 		0b00100000
#define PT_BL_2 			0b00010000
#define PT_BL_1				0b00001000
#define PT_BL_0 			0b00000100
#define PT_CF	 			0b00000001
#define PT_READ	 			0b00000000

#endif // _UM7_REGISTERMAP_H_
