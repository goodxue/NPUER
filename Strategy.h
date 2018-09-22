#ifndef Strategy_H
#define Strategy_H

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the STRATEGY_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// STRATEGY_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef STRATEGY_EXPORTS
#define STRATEGY_API __declspec(dllexport)
#else
#define STRATEGY_API __declspec(dllimport)
#endif


const long PLAYERS_PER_SIDE = 5;

// gameState
const long FREE_BALL = 1;
const long PLACE_KICK = 2;
const long PENALTY_KICK = 3;
const long FREE_KICK = 4;
const long GOAL_KICK = 5;

// whosBall
const long ANYONES_BALL = 0;
const long BLUE_BALL = 1;
const long YELLOW_BALL = 2;

// global variables
const double FTOP = 77.2392;
const double FBOT = 6.3730;
const double GTOPY = 49.6801;
const double GBOTY = 33.9320;
const double GRIGHT = 97.3632;
const double GLEFT = 2.8748;
const double FRIGHTX = 93.4259;
const double FLEFTX = 6.8118;

const int VC = 125;                            // last:118   110          较为成功
const double _Kp = 7.3;                          //last:7.1            6
const double _Ki = 2.7;                        //last:2.1  3.1         1.1
const double _Kd = 1.2;                        //last:0.8              0.8
const double Makeup = 50;                      //last:30  20
const int _SumTime = 66;

typedef struct
{
	double x, y, z;// x 和 y 为坐标值

} Vector3D;

typedef struct
{
	long left, right, top, bottom;
} Bounds;

typedef struct
{
	Vector3D pos;// 机器人坐标
	double rotation;// 机器人方向角
	double velocityLeft, velocityRight;// 机器人左右轮速度
} Robot;

typedef struct
{
	Vector3D pos;// 机器人的坐标位置
	double rotation;// 机器人当前的转角
} OpponentRobot;

typedef struct
{
	Vector3D pos;  // 小球的坐标位置
} Ball;

typedef struct
{
	Robot home[PLAYERS_PER_SIDE];		//我方机器人数组
	OpponentRobot opp[PLAYERS_PER_SIDE];//敌方机器人数组
	Ball currentBall,					//当前小球的位置
		 lastBall,						//上一次小球的位置
		 predictedBall;					//预计的小球的位置
	Bounds fieldBounds,					//场地范围
		   goalBounds;					//球门的位置与范围
	long gameState;						//当前比赛的状态
	long whosBall;						//由谁控制球
	void *userData;						//用户自定义信息
} Environment;						//环境信息

typedef void (*MyStrategyProc)(Environment*);

/* MUST BE IMPLEMENTED */
extern "C" STRATEGY_API void Create ( Environment *env ); // implement this function to allocate user data and assign to Environment->userData
extern "C" STRATEGY_API void Strategy ( Environment *env );
extern "C" STRATEGY_API void Destroy ( Environment *env ); // implement this function to free user data created in  Create (Environment*)

struct PID{
	double SetSpeed;
	double ActualSpeed;
	double err;
	double err_next, err_last;
	double Kp, Ki, Kd;
	PID(){
		SetSpeed = 0;
		ActualSpeed = 0;
		err = 0;
		err_last = err_next = 0;
		Kp = _Kp;
		Ki = _Ki;
		Kd = _Kd;
	}
	double PID_start(double theta,double dis){
		if (dis < 10) Kp = 0.55;
		err = theta;
		/*if (err_next > 60){
			err_next = 0;
		}
		if (err_last > 60){
			err_last = 0;
		}*/
		//double increasmentSpeed = Kp * err - Ki * err_next + Kd * err_last;
		//ActualSpeed += incresamentSpeed;
		double Integral = err * Ki;

		/*if (Integral > 30) Integral = 30;
		else if (Integral < -30) Integral = -30;*/

		if (dis < 1.3) {
			Kp = 0;            //last:0
			Ki = 1.3;          //last:1.8
			Kd = 0;            //last:0.1
		}
		else{
			Kp = _Kp;
			Kd = _Kd;
		}

		double increasmentSpeed = Kp * (err - err_next) + Integral + Kd * (err - 2 * err_next + err_last);
		err_last = err_next;
		err_next = err;
		/*double equalize = exp(-0.2 * dis) * Makeup;
		equalize *= theta >= 0 ? -1 : 1;*/
		return increasmentSpeed;//+ equalize;
	}
	void init() {
		err = 0;
		err_last = 0;
		err_next = 0;
	}
};

void Initial(Environment *env);
void Velocity(Robot &robot, int vl, int vr);
void Rotate(Robot &robot, int aimed_angle);
void Move(Robot &robot, double x, double y);
void Move2(Robot &robot, double x, double y);
double Distance(Robot &robot, Ball &ball);
double Distance(Vector3D &pos1, Vector3D &pos2);
bool isBallMoving(Ball &last, Ball &current);
void predictBall(Environment *env,int step = 1);
void Position1(Robot &robot, double x, double y);
double DotPro(double x1, double y1, double x2, double y2);
double CroPro(double x1, double y1, double x2, double y2);
void PrintSpeed(double vl, double vr);
#endif // Strategy_H