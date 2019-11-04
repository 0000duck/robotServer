#include <iostream>
#include "robotServer.h"
#include "robotClient.h"
#include "functions.h"

// 此处每次发布版本要手动更新
const char *LIB_INFO = "librobotServer version: 1.0.5 (2019-10-29, 08:30)";
#define VERSION	"1.0.5" // 此处每次发布版本要手动更新
#define NAME	"robotServerExe"

using namespace rclib;
using namespace std;

void initLib();
#define TRANS_FILE "/TransformMat.txt"
#define POINT_FILE "/border.txt"

///移动传送带
void moveCSD(bool flag);
void string2doubleForVec(Common::VECTOR_STRING& vecSour, std::vector<double>& vecT);
void MoveTraj(std::list<Terminal>& terPointList);
bool test();
static void show_info(char *argv[])
{
	if (0 == strcmp("-v", argv[1]))
	{
		printf("%s version: %s (%s, %s)\n", NAME, VERSION, __DATE__, __TIME__);
		return;
	}
	else if (0 == strcmp("-h", argv[1]))
	{
		printf("Usage: %s [options]\n", NAME);
		printf("Options:\n");
		printf("  void\tRun %s\n", NAME);
		printf("  -v\tDisplay %s version information\n", NAME);
		printf("  -h\tDisplay help information\n");
		return;
	}

	printf("Usage: %s [-v | -h]\n", NAME);
	return;
}
int main(int argc, char* argv[]){
	if (2 == argc)
	{
		show_info(argv);
		return 0;
	}
#ifdef WIN32
	if (!Common::bAuthorise())
	{
		printf("no licence\n");
		return -1;
	}
#endif
    //RobotClient::initance()->initSystem("192.168.253.135",8080);
    //test();
    RobotServer::initance()->initSystem();
    RobotServer::delInitance();

    return 0;
}

/**
对于动态库CMakeList的这种写法有问题，该接口只参与编译，不使用。
*/

void initLib()
{
//    RobotClient::initance();
//    RobotServer::initance();
//    RobotServer::initance()->nullApi();
//    RobotClient::initance()->nullApi();
//    RobotClient::initance()->waitCommandEnd();

}


///移动传送带
void moveCSD(bool flag)
{
	SWITCHSTATE DI1 = RobotClient::initance()->getDigitalInput(PORT_1);
	SWITCHSTATE DI2 = RobotClient::initance()->getDigitalInput(PORT_2);
	RobotIO ioV = RobotClient::initance()->getRobotIO();
	DI1 = ioV.getDigitalInputState()[PORT_1];
	if (flag)
	{
		//从相机运动到机器人
		if (DI1 == SWITCHON)
		{
			//DO1=1
			RobotClient::initance()->setDigitalOutput(PORT_1, SWITCHON);
			while (true)
			{
				sleep_ms(1000);
				DI1 = RobotClient::initance()->getDigitalInput(PORT_1);
				if (DI1 == SWITCHOFF)
				{
					RobotClient::initance()->setDigitalOutput(PORT_1, SWITCHOFF);
					break;
				}
			}
		}
		else
		{
			cout << "DI1 == SWITCHOFF" << endl;
		}
	}
	else
	{
		//从机器人到相机
		if (DI2 == SWITCHON)
		{
			RobotClient::initance()->setDigitalOutput(PORT_2, SWITCHON);
			while (true)
			{
				sleep_ms(1000);
				DI2 = RobotClient::initance()->getDigitalInput(PORT_2);
				if (DI2 == SWITCHOFF)
				{
					RobotClient::initance()->setDigitalOutput(PORT_2, SWITCHOFF);
					break;
				}
			}
		}
		else
		{
			cout << "DI2 == SWITCHOFF" << endl;
		}
	}


}
void string2doubleForVec(Common::VECTOR_STRING& vecSour, std::vector<double>& vecT)
{
	for (auto iter = vecSour.begin(); iter != vecSour.end(); iter++)
	{
		double value = atof((*iter).c_str());
		vecT.push_back(value);
	}

}
void MoveTraj(std::list<Terminal>& terPointList)
{
	if (terPointList.empty())
	{
		cout << "terPointList is empty!" << endl;
		return;
	}

	//运动
	int nRt = 0;
    double gVel = 0.1;double vel = 30;
	double acc = 0.8; double jerk = 0.8; double turn = 0.3;
	COORDINATESYSTEM frame = COORDINATE_BASE;
	auto iter = terPointList.begin();
	Terminal terPre = *iter;
	RobotClient::initance()->setServoState(SWITCHON);
	RobotClient::initance()->waitCommandEnd();
	RobotClient::initance()->programVelocity(gVel);//程序速度

	///调整姿态
    terPre[TERMINAL_Z] += 50;
	nRt = RobotClient::initance()->moveJoint(terPre,0.5);
	if (nRt == 0)
	{
		nRt = RobotClient::initance()->waitCommandEnd();
	}
	RobotClient::initance()->moveStartCon();
	for (iter++; iter != terPointList.end(); iter++)
	{
		Terminal& ter = *iter;
		double nor = (ter.getPoint() - terPre.getPoint()).norm();
		if (abs(nor) < 5)
		{
			continue;
		}
		nRt = RobotClient::initance()->moveLineCon(ter, vel, acc,jerk,turn,frame);
		terPre = ter;
		if (nRt)
		{
			break;
		}
    }
    RobotClient::initance()->moveEndCon();
    RobotClient::initance()->waitCommandEnd();
    terPre[TERMINAL_Z] += 50;
    nRt = RobotClient::initance()->moveJoint(terPre,0.5);
    if (nRt == 0)
    {
        nRt = RobotClient::initance()->waitCommandEnd();
    }
	RobotClient::initance()->setErrorClear();
	RobotClient::initance()->setServoState(SWITCHOFF);
}
bool test()
{
    string strPath = Common::GetModuleFullPath(true);
    //string strPath("H:\\Scan HighSpeed 0805\\shoe");
	string strV = Common::ReadFileToString((strPath + TRANS_FILE).c_str());
	Common::LIST_STRING lst;
	lst = Common::ToLists(strV, " ");
	if (lst.size() != 16)
	{
		cout << "hMatrix size=" << lst.size() << endl;
		return false;
	}
	std::vector<double> vecValue;
	for (auto iter = lst.begin(); iter != lst.end(); iter++)
	{
		string strVaule = *iter;
		vecValue.push_back(atof(strVaule.c_str()));
	}
	CMatrix<double> mValue(4, 4, vecValue);
	HomogeneousMatrix hMatrix(mValue);		//齐次矩阵
	bool bLoop = true;
    //while (bLoop)
	{
//		int n = 0;
//		printf("Enter 0 to exit or 1 to move!\n");
//		scanf("%d", &n);
//		if (n == 0)
//		{
//			break;
//		}
		//从文件中获取点POINT_FILE
		std::list<string> lstPoint;		//文件中点
		std::list<Terminal> terPointList;		//转换后点
		Common::ReadFileToList((strPath + POINT_FILE).c_str(), lstPoint);

		//左乘齐次矩阵
		//保存转换后的点
		stringstream out;
		out << "*****************************************" << endl;
		for (auto iter = lstPoint.begin(); iter != lstPoint.end(); iter++)
		{
			Common::VECTOR_STRING vecField = Common::ToVectors(*iter, " ");
			if (vecField.size() != 6)
			{
				continue;
			}
			std::vector<double> vecDoub;
			string2doubleForVec(vecField, vecDoub);
			//法向量
            //UnitVector normVector(-vecDoub[3], -vecDoub[4], -vecDoub[5]);
            UnitVector normVector(0, 0, -1);
            DualVector dvVar;
			//dvVar.setNormVector(normVector);
			dvVar.setDirVector(normVector);
			AttitudeAngle Atti = dvVar.getAttitudeAngle();
            Point point(vecDoub[0], vecDoub[1], vecDoub[2]+5);
			HomogeneousMatrix homo(point, Atti); //hMatrix是转换矩阵
			Terminal ter((hMatrix*homo).getTerminal());
			terPointList.push_back(ter);

			//保存转换后的点
			out << ter[TERMINAL_X] << "\t" << ter[TERMINAL_Y] << "\t" << ter[TERMINAL_Z] << "\t"
				<< ter[TERMINAL_A] << "\t" << ter[TERMINAL_B] << "\t" << ter[TERMINAL_C] << endl;
		}
        fstream outF; outF.open(strPath + "/hrPoint.txt", ios_base::out | ios_base::app);
		outF.is_open();
		outF << out.str();
		outF.close();
		//运动
		MoveTraj(terPointList);
	}
	return true;
}
