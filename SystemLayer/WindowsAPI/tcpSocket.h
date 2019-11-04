#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#ifndef _WINSOCK2API_
//#define _WINSOCK2API_
//#define _WINSOCKAPI_ 
#endif
//#include <Winsock2.h>
#include <process.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>

#include "systemLib.h"

namespace rclib{


#define ROBOT_TEMP_SEND_FILE "RobotSendFile.xml"
#define ROBOT_TEMP_RECV_FILE "RobotRecvFile.xml"

enum TCPSOURCETYPE { TCPSOURCE_SERVER, TCPSOURCE_CLIENT, TCPSOURCE_OTHER, TCPSOURCE_UNDEFINE };
enum TCPPACKAGETYPE { TCPPACKAGE_MSG, TCPPACKAGE_FILE };

struct PagTCP{
    std::string label;      // 消息或者文件类型的标签，一般为4个字符的描述
    TCPSOURCETYPE src;      // 消息来源：0:控制器服务端 1:示教器客户端 2:机器人库客户端 3:其他客户端设备，例如视觉
    TCPPACKAGETYPE type;    // 消息类型：0:指令 1:文件
    int num;                // 消息序号，消息包较大需要拆包时使用
    int allnum;             // 消息总数，消息包较大需要拆包时使用
    std::string content;	// 消息内容
};

class TCPClient{
public:
    TCPClient();
    _declspec(dllexport)~TCPClient();

    bool tcpClientInit(const char* server_ip, int server_port);

    void send_msg(const char *label, std::string msg);
    void send_file(const char *label, const char *filePath);

    std::string getTmpFileName();
	std::string m_strPath;				//程序运行路径
private:
    static TCPClient* tcpClient;
    bool WinSockInit();

    int send_package(int fd, PagTCP& pag);
    int recv_package(int fd, PagTCP& new_package, std::string& rest_package);
	static unsigned int _stdcall recv_msg_file(LPVOID lpParameter);

    _declspec(dllexport) virtual void excuteFile(int fd, TCPSOURCETYPE src, std::string command);
    _declspec(dllexport) virtual void excuteCommand(int fd, TCPSOURCETYPE src, std::string command, std::string content);
private:
    SOCKET m_socket_fd;
    struct sockaddr_in m_server_addr;
    Mutex m_tcp_mutex;
    std::string m_tmpFileName;

	enum FILE_TYPE_VALUE { CSWJ, PZWJ, ZBWJ };
	enum COMMAND_TYPE_VALUE {
		CWXX, JGXX, TSXX,
		WCIO, WCZL, CXSD, CXXH, GJMD, TDDL, SJWZ,
        IOZT, SRZT, SCZT, MNSR, MNSC, MNDL, CSYX, FSZT, SFZT, TSZT, YXMS, KZXT,
		XGGJ, XGYH, TCPF, USRF
	};

	std::map<std::string, COMMAND_TYPE_VALUE> m_commandType;
};


class TCPServer{
public:
	TCPServer();
	~TCPServer();

	void tcpServerInit(int listen_port);

	void send_msg_exceptFd(int client_fd, const char *label, std::string msg);  // 除了这个fd都发送

	void send_msg(int client_fd, const char *label, std::string msg);
	void send_file(int client_fd, const char *label, const char *filePath);
	void send_msg_type(TCPSOURCETYPE src, const char *label, std::string content);
	void send_file_type(TCPSOURCETYPE src, const char *label, const char *filePath);

	void setControlFd(int fd);
	bool isControlFd(int fd);
	int controlFd();
	int setFirstConnect(int fd, TCPSOURCETYPE src);
	bool isFirstConnect(int fd);
	std::string m_strPath;
private:
	static TCPServer* tcpServer;

	static THREAD_ROUTINE_RETURN (_stdcall tcpServerConnect)(void* lpParameter);
	void tcpServerClose();

	int send_package(int fd, PagTCP& pag);
	int recv_package(int fd, PagTCP& new_package, std::string& rest_package);
	static THREAD_ROUTINE_RETURN (_stdcall recv_msg_file)(void* lpParameter);

	virtual void excuteFile(int fd, TCPSOURCETYPE src, std::string command);
	virtual void excuteCommand(int fd, TCPSOURCETYPE src, std::string command, std::string content);
	bool WinSockInit();

	int m_control_fd;
	int m_socket_fd;
	Mutex m_tcp_mutex;

	std::map<TCPSOURCETYPE, int> m_mapClientNum;
	std::map<int, Thread> m_mapClientThread;
	std::map<int, TCPSOURCETYPE> m_mapClientType;
};

}

#endif
