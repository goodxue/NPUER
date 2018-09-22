// Strategy.cpp : Defines the entry point for the DLL application.
//

#include "stdafx.h"
#include "Strategy.h"
#include <queue>
#include <fstream>
#include <math.h>

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
    }
    return TRUE;
}

const double PI = 3.1415923;
#define RL 3.09727//机器人车长
#define BL 1.9856 //球的直径

std::queue<Vector3D> history;
int PlayTime = 0;
int Flag = 1;
PID ControlL,ControlR;
std::ofstream fo("text.txt");
double SpeedSum = 0;
int sumTime = _SumTime;
int persec = 0;

extern "C" STRATEGY_API void Create ( Environment *env )
{
	// allocate user data and assign to env->userData
	// eg. env->userData = ( void * ) new MyVariables ();
}//比赛开始时系统调用一次


extern "C" STRATEGY_API void Destroy ( Environment *env )
{
	// free any user data created in Create ( Environment * )
	// eg. if ( env->userData != NULL ) delete ( MyVariables * ) env->userData;
}//比赛结束时系统调用一次


extern "C" STRATEGY_API void Strategy ( Environment *env )
{
	PlayTime++;
	//Move(env->home[1], 20, 20);
	
	if (history.size() < 66) history.push(env->home[1].pos);
	else {
		history.pop();
		history.push(env->home[1].pos);
	}


	////if (PlayTime < 100) Initial(env);

	//
		predictBall(env, 1);
		
	//

	if (Distance(history.back(), history.front()) < 1) {
		Vector3D destin;
		double ballx = env->currentBall.pos.x, bally = env->currentBall.pos.y, robx = env->home[1].pos.x, roby = env->home[1].pos.y;
		destin.x = (ballx - robx) / 2 + robx;
		destin.y = (bally - roby) / 2 + roby;
		if (robx < 8.5){              //在球场左侧
			destin.x += 3;
		}
		else if (robx > 91.6) {
			destin.x -= 3;
		}
		if (roby < 8){
			destin.y += 3;
		}
		else if (roby > 75.6) {
			destin.y -= 3;
		}
		Move2(env->home[1], destin.x, destin.y);
	}
	else
		Move2(env->home[1], env->predictedBall.pos.x, env->predictedBall.pos.y);

	//	fo << "Ball's position: " <<"("<< env->predictedBall.pos.x << ","<<env->predictedBall.pos.y <<")"<< std::endl;
	//	double ballx = env->currentBall.pos.x, bally = env->currentBall.pos.y, robx = env->home[1].pos.x, roby = env->home[1].pos.y;

	//	if (Distance(env->home[1], env->currentBall) < 4) { //机器人正在运球
	//		if (bally > 70 || bally < 12){                         //球临近上下边界
	//			if (roby < bally - 1.5) {										//从下往上运球
	//				if (ballx < 50){											//在球场左侧
	//					Rotate(env->home[1], -50);									//向右转
	//				}
	//				else {
	//					Rotate(env->home[1], 50);									//向左转
	//				}
	//			}
	//			else if (roby > bally + 1.5){										//从上到下
	//				if (ballx < 50){
	//					Rotate(env->home[1], 50);
	//				}
	//				else {
	//					Rotate(env->home[1], -50);
	//				}
	//			}
	//		}

	//		if (ballx < 12 || ballx > 85) {                 //球临近左右边界
	//			if (robx < ballx - 1.5) {                                      //从左往右运球
	//				if (bally < 40) {                                          //在球场下侧
	//					Rotate(env->home[1], 50);                                  //向左转
	//				}
	//				else {
	//					Rotate(env->home[1], -50);
	//				}
	//			}
	//			else if (robx > ballx + 1.5) {
	//				if (bally < 40) {
	//					Rotate(env->home[1], -50);
	//				}
	//				else {
	//					Rotate(env->home[1], 50);
	//				}
	//			}
	//		}
	//	}
	//}

	//当两次球的坐标一样，说明球停止，根据球在场地中的位置，采用不同的方法
	/*if (env->currentBall.pos.x == env->lastBall.pos.x && env->currentBall.pos.y == env->lastBall.pos.y) {

	}*/

	//避障：每次检测机器人与球之间是否又其他机器人，有的话采用避障算法，求出障碍物的圆域半径，绕过去就ok了
	// the below codes are just for demonstration purpose....don't take this seriously please.

	//env->home[1].velocityLeft = 125;
	//env->home[1].velocityRight = 125;
}

void Initial(Environment *env)
{
	for (int i = 2; i < 5; i++){
		Move(env->home[i], 96.5232, 48.1415 + 3.0973*(5-i));
	}
}

void Velocity(Robot &robot, int vl, int vr)
{
	if (vl > 125) vl = 125;
	else if (vl < -125) vl = -125;
	if (vr > 125) vr = 125;
	else if (vr < -125) vr = -125;
	robot.velocityLeft = vl;
	robot.velocityRight = vr;
	PrintSpeed(vl, vr);
}

void Rotate(Robot &robot, int theta)
{
	int  vl = 0, vr = 0;
	//theta = aimed_angle - robot.rotation;
	while (theta > 180) theta -= 360;
	while (theta < -180) theta += 360;
	if (theta < -90) theta += 180;
	else if (theta > 90) theta -= 180;
	if (abs(theta) > 80) {
		vl = (int)(-25.0 / 90.0 * (double)theta);
		vr = (int)(25.0 / 90.0 * (double)theta);
	}
	else if (abs(theta) > 50) {
		vl = (int)(-20 / 90.0 * (double)theta);
		vr = (int)(20 / 90.0 * (double)theta);
	}
	else if (abs(theta) > 20) {
		vl = (int)(-11 / 90.0 * (double)theta);
		vr = (int)(11 / 90.0 * (double)theta);
	}
	else if (abs(theta) > 10) {
		vl = (int)(-8.0 / 90.0 * (double)theta);
		vr = (int)(8.0 / 90.0 * (double)theta);
	}
	Velocity(robot, vl, vr);
}

void Move(Robot &robot, double x, double y)
{
	double dx, dy, dis, K = 10.0 / 90.0;
	int aimed_angle = 0, theta = 0, vl, vr, vc = 110;
	dx = x - robot.pos.x;
	dy = y - robot.pos.y;

	dis = sqrt(dx * dx + dy * dy);
	if (dx == 0 && dy == 0) aimed_angle = 90;
	else aimed_angle = (int)(180.0 / PI * atan2((double)(dy), (double)(dx)));
	theta = aimed_angle - (int)robot.rotation;
	while (theta > 180) theta -= 360;
	while (theta < -180) theta += 360;

	if (dis > 100) K = 17.0 / 90;
	else if (dis > 50) K = 19.0 / 90;   //参数待调整
	else if (dis > 30) K = 21.0 / 90;    //同上
	else if (dis > 20) K = 23.0 / 90;     //同上
	else K =  25.0 / 90;

	double cont = abs(180 - theta);
	if (abs(theta) < cont){
		
			vr = (int)(vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) + K * theta) * Flag;
			vl = (int)(vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) - K * theta) * Flag;
		
	}
	else {
		if (theta > 0) {
			vr = (int)(-vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) + K * cont) * Flag;
			vl = (int)(-vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) - K * cont) * Flag;
		}
		else{
			vr = (int)(-vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) - K * cont) * Flag;
			vl = (int)(-vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) + K * cont) * Flag;
		}
	}
	fo << "PlayTime: " << PlayTime << std::endl;
	fo << "vLeft: " << vl << "\t" << "vRighr" << vr << std::endl;

	//下面部分是源码，但是有些地方不清楚
	/*if (abs(theta) > 95) {
		theta += 180;
		if (theta > 360) theta -= 360;
		if (theta > 80) theta = 80;
		if (theta < -80) theta = -80;

		if (dis < 5.0 && abs(theta) < 40)
			K = 0.5;
		vr = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * dis)) - 0.3) + K*theta);
		vl = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * dis)) - 0.3) - K*theta);

	}

	else if (theta < 85 && theta > -85)
	{
		if (dis < 5.0 && abs(theta) < 40)
			K = 0.5;
		vr = (int)(vc * (1.0 / (1.0 + exp(-3.0 * dis)) - 0.3) + K * theta);
		vl = (int)(vc * (1.0 / (1.0 + exp(-3.0 * dis)) - 0.3) - K * theta);
	}
	else
	{
		vr = (int)(+.17 * theta+vc);
		vl = (int)(-.17 * theta+vc);
	}*/
	Velocity(robot, vl, vr);
}
		/*if (theta > 180) theta -= 360;
		if (dis < 5.0 && abs(theta) < 40)
			k = 0.1;
		vr = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * dis)) - 0.3) + k*theta);
		vl = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * dis)) - 0.3) + k*theta);

	}
	velocity(robot, vl, vr);*/

double Distance(Robot & robot, Ball &ball)
{
	return sqrt(pow(robot.pos.x - ball.pos.x, 2) + pow(robot.pos.y - ball.pos.y, 2));
}

double Distance(Vector3D &pos1, Vector3D &pos2)
{
	return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
}

bool isBallMoving(Ball &last, Ball &current)
{
	return last.pos.x == current.pos.x && last.pos.y == current.pos.y;
}

void predictBall(Environment *env,int step)
{
	Ball ans;
	double dx = env->currentBall.pos.x - env->lastBall.pos.x, dy = env->currentBall.pos.y - env->lastBall.pos.y;
	env->predictedBall.pos.x = env->currentBall.pos.x + step * dx;
	env->predictedBall.pos.y = env->currentBall.pos.y + step * dy;
}

void Move2(Robot &robot, double x, double y)
{
	double vr, vl, vc = 100;
	double Rotation = robot.rotation,K;
	double fx = cos(Rotation/180.0 * PI), fy = sin(Rotation/180.0 * PI);

	double dx = x - robot.pos.x,dy = y - robot.pos.y;
	double Facing = DotPro(fx, fy, dx, dy);
	double dis = sqrt(dx * dx + dy * dy);
	double aimed_angle = (int)(180.0 / PI * atan2((double)(dy), (double)(dx)));
	double theta = (180.0 / PI * acos( Facing /sqrt(dx * dx + dy * dy))); //这里修改为了dx dy
	if (dis > 100) K = 1.2;// 17.0 / 90;
	else if (dis > 50) K = 0.9;// 19.0 / 90;   //参数待调整
	else if (dis > 30) K = 0.8;// 21.0 / 90;    //同上
	else if (dis > 10) K = 0.7;// 23.0 / 90;     //同上
	//else ControlL.Kp = ControlR.Kp = 5;//25.0 / 90;
	if (Facing >= 0) {
		if (Flag != 1) {
			Flag = 1;
			ControlL.init();
			ControlR.init();
		}
		double crosp = CroPro(fx, fy, dx, dy);
		if (crosp < 0) theta = -theta;
		vr = VC + ControlL.PID_start(theta,dis);		
		vl = VC + ControlR.PID_start(-theta,dis);
		if (abs(theta) > 20){
			vr += theta >= 0 ? exp(-0.2 * dis) * Makeup : -exp(-0.2 * dis) * Makeup;
			vl += theta >= 0 ? -exp(-0.2 * dis) * Makeup : exp(-0.2 * dis) * Makeup;
		}
		/*vr = (int)(vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) + K * theta) ;
		vl = (int)(vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) - K * theta) ;*/
	}
	else {
		if (Flag != 0) {
			Flag = 0;
			ControlL.init();
			ControlR.init();
		}
		double crosp = CroPro(-fx, -fy, dx, dy);
		if (crosp >= 0) theta = 180 - theta;
		else theta = theta - 180;
		vr = -VC + ControlL.PID_start(theta,dis);		
		vl = -VC + ControlR.PID_start(-theta,dis);
		if (abs(theta) > 20){
			vr += theta >= 0 ? exp(-0.2 * dis) * Makeup : -exp(-0.2 * dis) * Makeup;
			vl += theta >= 0 ? -exp(-0.2 * dis) * Makeup : exp(-0.2 * dis) * Makeup;
		}
		/*vr = (int)(-vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) + K * theta) ;
		vl = (int)(-vc * (1.0 / (1.0 + exp(-2.0 * dis)) - 0.2) - K * theta) ;*/
	}
	Velocity(robot, vl, vr);
	//fo << "PlayTime: " << PlayTime << std::endl;
	//fo << "vLeft: " << vl << "\t" << "vRighr" << vr << std::endl;
	//fo << "【theta】  " << theta << std::endl;
	///*fo << "dx: " << dx << "     dy: " << dy <<"       fx: "<<fx<<"        fy: "<<fy<<"          Rotation: "<<Rotation<< std::endl;
	//fo << "pos.x: " << robot.pos.x << "     pos.y: " << robot.pos.y << std::endl << std::endl;*/
	//fo << "ControlL.err:  " << ControlL.err << "             ControlL.err_next:  " << ControlL.err_next << "           ControlL.err_last:  " << ControlL.err_last << std::endl;
}

double DotPro(double x1, double y1, double x2, double y2)
{
	return x1 * x2 + y1 * y2 ;
}

double CroPro(double x1, double y1, double x2, double y2)
{
	return x1 * y2 - x2 * y1;
}

void PrintSpeed(double vl, double vr)
{
	SpeedSum += (abs(vl) + abs(vr)) / 2;
	sumTime--;
	if (sumTime == 0) {
		persec++;
		fo << "speed per sec: 【" << SpeedSum / _SumTime << "】       in  "<<persec<<"   sec "<< std::endl;
		SpeedSum = 0;
		sumTime = _SumTime;
	}
}