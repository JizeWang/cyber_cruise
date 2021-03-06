/***************************************************************************

    file                 : user3.cpp
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  user module for CyberParking

 ***************************************************************************/

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_parking";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberParking" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	printf("OK!\n");
	return 0;
}

/*
     WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

/* 
    define your variables here.
    following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
	printf("lotX %.6f  lotY %.6f carX %.6f  carY %.6f",_lotX,_lotY,_carX,_carY);
}

static int flag = 0;
static float k,b,dist;
static int flagt = 0;

static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){   
	/* write your own code here */
	if(abs(_lotAngle)>(PI/2 - 0.05) && abs(_lotAngle)< (PI/2 + 0.05))                       //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
		dist = abs(_carX - _lotX);
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);
		dist = abs(k*_carX - _carY +b)/sqrt(k*k + 1);
	}

	
	if ( flagt == 1 ){
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = -1;
		*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/3.14;
	}else
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 0.5 ) {    //用车速判断是否完成泊车
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed < 0.01){
			    *bFinished = true;
				flagt = 1;}
			else
			{*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			flag = 1; 
		}else if( ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10)&& ( flagt ==2 )) { //接近停车位时，控制车的朝向与车位一致，速度控制在2
			//*cmdSteer = 0;
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			else if(_speed >0.07){*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			else{*bFinished = true;flagt=1; }
			flag = 2;
		} else if (((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2)&&(flagt!=2)){//较接近停车位时，给一个大的转向，速度控制在10			
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			if (fabs(fabs(_caryaw)-fabs(_lotAngle))<0.015){
				flagt = 2;
			}
			flag = 3;
		} else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 5000) { //到一定范围时，将车子调整至道路右侧，增加转弯半径，速度控制在15
			*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/3.14;
			if( _speed > 15 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			flag = 4;
		}
		else {			                                                                         //其它路段按巡线方式行驶
			*cmdAcc = 1;//油门给100%
		    *cmdBrake = 0;//无刹车
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;//设定舵机方向
		    *cmdGear = 1;//档位始终挂1
			flag = 5;
	    }
	}

	
	if(*bFinished)
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10 ) { //接近停车位时，控制车的朝向与车位一致，速度控制在2
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
		}
		else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2 ){//较接近停车位时，给一个大的转向，速度控制在10			
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
		}
		else {			                                                                         //其它路段按巡线方式行驶
			*cmdAcc = 1;//油门给100%
		    *cmdBrake = 0;//无刹车
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;//设定舵机方向
		    *cmdGear = 1;//档位始终挂1
	    }
	}
	


	printf("Steer:%.2f\tflag:%d\tspeed:%.2f\tdist:%.2f\tlotAngle:%.2f\tcaryaw:%.2f\tflagt:%d\n",*cmdSteer,flag,_speed,dist,_lotAngle,_caryaw,flagt);
	printf("Steer:%.2f\tflag:%d\tflagt:%d\n",*cmdSteer,flag,flagt);
}
