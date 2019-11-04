#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <libgen.h>
#include "tcpSocket.h"
//#include "baselib.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace std;
using namespace rclib;

#define PAGHEAD string("TCPHEAD")
#define PAGTAIL string("TCPTAIL")
#define SEGCHAR string("x")

#define PAGSIZE 1024
#define CLIENT_MAXCONNECT 10    //teach maximum connection
#define OTHER_MAXCONNECT 3   //other maximum connection
#define MAXCONNECT CLIENT_MAXCONNECT+OTHER_MAXCONNECT

static int string_to_int(const string& p_content);
static string num_to_string(int p_int);

//路径分隔符
#ifdef _WIN32
#define PATH_SEPARATOR      '\\'
#define PATH_SEP_STRING     "\\"
#else
#define PATH_SEPARATOR      '/'
#define PATH_SEP_STRING     "/"
#endif

std::string GetModuleFullPath(bool bLastPath)
{
	char chPath[256] = { 0 };

#ifdef _WIN32
	::GetModuleFileName(NULL, chPath, sizeof(chPath));
#else
	memset(chPath, 0, sizeof(chPath));
	::readlink("/proc/self/exe", chPath, sizeof(chPath));
#endif
	std::string strModuleFileName = chPath;

	if (bLastPath == false)
	{
		return strModuleFileName.c_str();
	}

	//如果文件名称有生僻汉字并且第二字节是'\\'时，用find_last_of的方法就会有问题
	//比如："c:\\ABC癨.txt"就会返回"c:\\ABC\\"，而用_splitpath和_makepath正确返回"c:\\"
//	size_t nPos = strModuleFileName.find_last_of(PATH_SEP_STRING);
//	std::string strLastPath = strModuleFileName.substr(0, nPos);

#ifdef _WIN32
	char szDrive[_MAX_DRIVE];
	char szDir[_MAX_DIR];
	::_splitpath(chPath, szDrive, szDir, NULL, NULL);
	::_makepath(chPath, szDrive, szDir, NULL, NULL);
	//去除最后的'\\'
	if (chPath[strlen(chPath) - 1] == PATH_SEPARATOR)
	{
		chPath[strlen(chPath) - 1] = '\0';
	}

#else
	::dirname(chPath);
#endif

	std::string strLastPath = chPath;
	return strLastPath.c_str();
}

bool CxxDeleteDirectory (const char * lpszDirectory)
{
    if (!lpszDirectory || '\0' == *lpszDirectory)
    {
        return true;
    }

    std::string szDirectory = lpszDirectory;
    if (PATH_SEPARATOR != *szDirectory.rbegin())
        szDirectory += PATH_SEPARATOR;
    DIR *lpDir;
    struct dirent Entry;
    struct dirent *lpEntry;
    struct stat sb;

    if((lpDir = ::opendir(lpszDirectory)) == NULL)
    {
        return false;
    }
    while (1)
    {
        ::readdir_r(lpDir, &Entry, &lpEntry);
        if (lpEntry == NULL)
        {
            break;
        }
        //过滤当前目录(.)和上一级目录(..)
        if (strcmp(lpEntry->d_name, ".") == 0 || strcmp(lpEntry->d_name, "..") == 0)
        {
            continue;
        }
        std::ostringstream oFile;
        oFile << szDirectory << lpEntry->d_name;
        //判断该条目是否是目录
        if (::stat(oFile.str().c_str(), &sb) >= 0 && S_ISDIR(sb.st_mode))
        {
            if (!CxxDeleteDirectory (oFile.str().c_str()))
            {
//				DBGPRINT (DBG_ERROR, "DeleteDirectory:" << lpEntry->d_name);
            }
        }
        else
        {
            ::unlink (oFile.str().c_str());
        }
    }
    ::closedir(lpDir);
    return ::rmdir (lpszDirectory);
}

TCPClient* TCPClient::tcpClient;

TCPClient::TCPClient(){
    tcpClient = this;
    m_pid = getpid();
    m_strPath = GetModuleFullPath(false) +  "File" PATH_SEP_STRING;

    if(access(m_strPath.c_str(),F_OK)==0)
     {
        CxxDeleteDirectory(m_strPath.c_str());
     }
    if(mkdir(m_strPath.c_str(), 0777)==-1)
    {
       perror("mkdir   error");
    }
}

TCPClient::~TCPClient(){
    close(m_socket_fd);
    if(access(m_strPath.c_str(),F_OK)==0)
     {
         CxxDeleteDirectory(m_strPath.c_str());
     }
}

bool TCPClient::tcpClientInit(const char* server_ip, int server_port){
    fd_set fdRead;
    FD_ZERO(&fdRead);

    while (true){
        sleep_ms(1000);

        if ((m_socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0){
            printf("create socket error: %s(errno:%d)\n)", strerror(errno), errno);
            exit(-1);
        }

        sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);

        if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0){
            printf("inet_pton error for %s\n", server_ip);
            return false;
        }

        if (connect(m_socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0){
            printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
            close(m_socket_fd);
            return false;
        }

        Thread tcp_recv_thread;
        int ret;
        if(ret = tcp_recv_thread.create(recv_msg_file, &m_socket_fd)!=0){
            printf("create tcp recv thread failed\n");
            return false;
        }

        break;
    }

    printf("connected to server\n");
    return true;
}

int TCPClient::recv_package(int fd, PagTCP& new_package, string& rest_package){
    string packageStr = rest_package;
    int length = PAGSIZE + 100;
    while(1){
        int index = packageStr.find(PAGHEAD);
        if(index >= 0){
            packageStr = packageStr.substr(index);
            index = packageStr.find(PAGTAIL);
            if(index >= 0){
                rest_package = packageStr.substr(index + string(PAGTAIL).length());
                packageStr = packageStr.substr(string(PAGHEAD).length(), index-string(PAGHEAD).length());
                break;
            }
        }

        char* recvBuff = new char[length+1];

        int ret = recv(fd, recvBuff, length, 0);
        if(ret < 0){
            printf("Read server head failed ,ret=%d erron=%d\n",ret,errno);
            delete []recvBuff;
            return ret;
        }
        if(ret == 0){
            printf("Connection is closed by Server.\n");
            delete []recvBuff;
            return ret;
        }
        recvBuff[ret] = 0;

        packageStr += string(recvBuff);
        delete []recvBuff;
    }

    new_package.label = packageStr.substr(0, 4);
    packageStr = packageStr.substr(4);

    int index;
    index = packageStr.find(SEGCHAR);
    new_package.src = TCPSOURCETYPE(string_to_int(packageStr.substr(0, index)));
    packageStr = packageStr.substr(index+1);
    index = packageStr.find(SEGCHAR);
    new_package.type = TCPPACKAGETYPE(string_to_int(packageStr.substr(0, index)));
    packageStr = packageStr.substr(index+1);
    index = packageStr.find(SEGCHAR);
    new_package.num = string_to_int(packageStr.substr(0, index));
    packageStr = packageStr.substr(index+1);
    index = packageStr.find(SEGCHAR);
    new_package.allnum = string_to_int(packageStr.substr(0, index));
    packageStr = packageStr.substr(index+1);

    new_package.content = packageStr;

    return 1;
}

THREAD_ROUTINE_RETURN TCPClient::recv_msg_file(void* lpParameter){
    thread_detach();
    thread_name("TCPClient::recv_msg_file");

    int accept_fd = *(long *)lpParameter;

    string rest_package;
    fstream out;
    string msgContent;

    while(1){
        PagTCP new_package;
        int ret = tcpClient->recv_package(accept_fd, new_package, rest_package);
        if(ret == 0||((ret < 0)&&errno==EBADF)){
            break;
        }
        else if(ret < 0){
            continue;
        }

        if(new_package.type == TCPPACKAGE_MSG){
            msgContent += new_package.content;
//            if(new_package.label != "GJMD"){
//                cout << ">>> recv msg: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum << ", " << msgContent << endl;
                //DBGPRINT(DBG_CLIENT,">>> recv msg: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum << ", " << msgContent );
//            }
            if(new_package.num < new_package.allnum){
                continue;
            }

            tcpClient->excuteCommand(accept_fd,new_package.src, new_package.label, msgContent);
            msgContent.clear();
        }
        else if(new_package.type == TCPPACKAGE_FILE){
            cout << ">>> recv file: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum << endl;
            //DBGPRINT(DBG_CLIENT, ">>> recv file: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum);
            if(!out.is_open()){
                out.open((tcpClient->m_strPath+ ROBOT_TEMP_RECV_FILE).c_str(), ios::out | ios::binary | ios::ate);
                if(!out){
                    printf("Open File failed\n");
                    break;
                }
                out.seekg(0, ios::beg);
            }

            out.write(new_package.content.c_str(), new_package.content.length());

            if(new_package.num < new_package.allnum){
                continue;
            }

            printf("file from server %d is received\n", accept_fd);
            out.close();
            tcpClient->excuteFile(accept_fd, new_package.src, new_package.label);
        }
    }
    return 0;
}

int TCPClient::send_package(int fd, PagTCP& pag){
    string str = PAGHEAD;
    str += pag.label;
    str += num_to_string(pag.src) + SEGCHAR;
    str += num_to_string(pag.type) + SEGCHAR;
    str += num_to_string(pag.num) + SEGCHAR;
    str += num_to_string(pag.allnum) + SEGCHAR;
    str += pag.content;
    str += PAGTAIL;

    if(pag.type == TCPPACKAGE_MSG){
        cout << ">>> send msg: " << pag.label << ", " << pag.num << "/" << pag.allnum << ", " << pag.content << endl;
    }
    else if(pag.type == TCPPACKAGE_FILE){
        cout << ">>> send file: " << pag.label << ", " << pag.num << "/" << pag.allnum << endl;
    }

    if(send(fd, str.c_str(), str.length(), 0) < 0){
        printf("send TCP failed: %d\n", errno);
        close(fd);
        return -1;
    }

    return 0;
}

void TCPClient::send_msg(const char *label, std::string msg){
    m_tcp_mutex.lockMutex();

    int size = msg.length();
    int time = size/PAGSIZE + 1;
    for(int i = 0; i < time; i++){
        int length = PAGSIZE;
        if(i == time - 1){
            length = size%PAGSIZE;
        }
        PagTCP pag;
        pag.label = string(label);
        pag.src = TCPSOURCE_CLIENT;
        pag.type = TCPPACKAGE_MSG;
        pag.num = i+1;
        pag.allnum = time;
        pag.content = msg.substr(i * PAGSIZE, length);

        if(send_package(m_socket_fd, pag) < 0){
            m_tcp_mutex.unlockMutex();
            return;
        }
    }

    m_tcp_mutex.unlockMutex();
}

void TCPClient::send_file(const char *label, const char *filePath){
    m_tcp_mutex.lockMutex();
    char * buffer;
    int size;
    fstream in;
    in.open(filePath, ios::in|ios::binary|ios::ate);
    if(!in){
        m_tcp_mutex.unlockMutex();
        printf("Open File failed\n");
        return;
    }
    size = in.tellg();
    in.seekg (0, ios::beg);
    buffer = new char [size];
    in.read (buffer, size);
    in.close();
    string fileBuf(buffer);
    delete[] buffer;

    int time = size/PAGSIZE + 1;
    printf(">>> start to send file %s\n", label);
    for(int i = 0; i < time; i++){
        int length = PAGSIZE;
        if(i == time - 1){
            length = size%PAGSIZE;
        }
        PagTCP pag;
        pag.label = string(label);
        pag.src = TCPSOURCE_CLIENT;
        pag.type = TCPPACKAGE_FILE;
        pag.num = i+1;
        pag.allnum = time;
        pag.content = fileBuf.substr(i * PAGSIZE, length);

        if(send_package(m_socket_fd, pag) < 0){
            m_tcp_mutex.unlockMutex();
            return;
        }
    }
    printf(">>> finish sending file\n");
    m_tcp_mutex.unlockMutex();
}

void TCPClient::excuteCommand(int fd,TCPSOURCETYPE src, std::string command, std::string content){
    printf("deal new command\n");
}

void TCPClient::excuteFile(int fd,TCPSOURCETYPE src, std::string command){
    printf("deal new file\n");
}

TCPServer* TCPServer::tcpServer;

TCPServer::TCPServer(){
    tcpServer = this;

    m_control_fd = -1;

    m_mapClientNum[TCPSOURCE_CLIENT] = 0;
    m_mapClientNum[TCPSOURCE_OTHER] = 0;
    m_strPath = GetModuleFullPath(true) + PATH_SEP_STRING"config"PATH_SEP_STRING;
}

TCPServer::~TCPServer(){
    tcpServerClose();
}

void TCPServer::tcpServerInit(int listen_port){
    sockaddr_in myserver;
    memset(&myserver, 0, sizeof(myserver));
    myserver.sin_family = AF_INET;
    myserver.sin_addr.s_addr = htonl(INADDR_ANY);
    myserver.sin_port = htons(listen_port);

    if((m_socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0){
        printf("socket() failed\n");
        exit(-1);
    }

    if(bind(m_socket_fd, (sockaddr*) &myserver,sizeof(myserver)) < 0){
        printf("bind() failed, ret = %d\n", errno);
        exit(-1);
    }

    if(listen(m_socket_fd, 10) < 0){
        printf("listen() failed, ret = %d\n\n", errno);
        exit(-1);
    }

    Thread tcp_connect_thread;
    int ret;
    if(ret = tcp_connect_thread.create(tcpServerConnect, NULL)!=0){
        printf("create tcp connect thread failed %d\n", ret);
        exit(-1);
    }
    tcp_connect_thread.join();
}

THREAD_ROUTINE_RETURN TCPServer::tcpServerConnect(void* lpParameter){
    thread_detach();
    thread_name("TCPServer::tcpServerConnect");

    socklen_t sin_size = sizeof(struct sockaddr_in);
    sockaddr_in remote_addr;
    int accept_fd;
    while(1){
        if(tcpServer->m_mapClientThread.size() >= MAXCONNECT){
            sleep_ms(1000);
            continue;
        }
        if((accept_fd = accept(tcpServer->m_socket_fd,(struct sockaddr*) &remote_addr,&sin_size)) == -1){
            printf("Accept error!\n");
            continue;
        }
        printf("Received a connection from %s, port: %d\n", (char*) inet_ntoa(remote_addr.sin_addr), accept_fd);        
        tcpServer->m_mapClientType.insert(pair<int, TCPSOURCETYPE>(accept_fd, TCPSOURCE_UNDEFINE));
        Thread tcp_recv_thread;
        int ret;
        if(ret = tcp_recv_thread.create(recv_msg_file, &accept_fd) !=0){
            printf("create tcp recv thread failed %d\n", ret);
            exit(-1);
        }
        tcpServer->m_mapClientThread.insert(pair<int, Thread>(accept_fd, tcp_recv_thread));       
    }
}

void TCPServer::tcpServerClose(){
    //cout << "clear thread" << endl;
    for(map<int, Thread>::iterator it = m_mapClientThread.begin(); it != m_mapClientThread.end(); it++){
        close(it->first);
        (it->second).cancel();
        (it->second).join();
    }
    close(m_socket_fd);
    m_mapClientNum.clear();
    m_mapClientThread.clear();
    m_mapClientType.clear();
}

int TCPServer::recv_package(int fd, PagTCP& new_package, string& rest_package){
    string packageStr = rest_package;
    int length = PAGSIZE + 100;
    while(1){
        int index = packageStr.find(PAGHEAD);
        if(index >= 0){
            packageStr = packageStr.substr(index);
            index = packageStr.find(PAGTAIL);
            if(index >= 0){
                rest_package = packageStr.substr(index + string(PAGTAIL).length());
                packageStr = packageStr.substr(string(PAGHEAD).length(), index-string(PAGHEAD).length());
                break;
            }
        }

        char* recvBuff = new char[length+1];

        int ret = recv(fd, recvBuff, length, 0);
        if(ret < 0){
            printf("Read client head failed ret=%d\t errno=%d\n",ret,errno);
            delete []recvBuff;
            return ret;
        }
        if(ret == 0){
            printf("Connection is closed by Client .\n");
            delete []recvBuff;
            return ret;
        }
        recvBuff[ret] = 0;

        packageStr += string(recvBuff);
        delete []recvBuff;
    }

    new_package.label = packageStr.substr(0, 4);
    packageStr = packageStr.substr(4);

    int index;
    index = packageStr.find(SEGCHAR);
    new_package.src = TCPSOURCETYPE(string_to_int(packageStr.substr(0, index)));
    packageStr = packageStr.substr(index+1);
    index = packageStr.find(SEGCHAR);
    new_package.type = TCPPACKAGETYPE(string_to_int(packageStr.substr(0, index)));
    packageStr = packageStr.substr(index+1);
    index = packageStr.find(SEGCHAR);
    new_package.num = string_to_int(packageStr.substr(0, index));
    packageStr = packageStr.substr(index+1);
    index = packageStr.find(SEGCHAR);
    new_package.allnum = string_to_int(packageStr.substr(0, index));
    packageStr = packageStr.substr(index+1);

    new_package.content = packageStr;

    return 1;
}

THREAD_ROUTINE_RETURN TCPServer::recv_msg_file(void* lpParameter){
    thread_detach();
    thread_name("TCPServer::recv_msg_file");

    int accept_fd = *(long *)lpParameter;
    string threadName = string("tcp_recv")+num_to_string(accept_fd);
    thread_name(threadName.c_str());

    string rest_package;
    fstream out;
    string msgContent;

    while(1){
        PagTCP new_package;
        int ret = tcpServer->recv_package(accept_fd, new_package, rest_package);
        if(ret == 0||((ret < 0)&&errno==EBADF)){
            break;
        }
        else if(ret < 0){
            continue;
        }

        if(tcpServer->controlFd()!=-1 &&tcpServer->controlFd()!=accept_fd&&(new_package.label != "JLLJ"))
        {
            tcpServer->send_msg(accept_fd, "WCIO", "0|");
            tcpServer->send_msg(accept_fd, "WCZL", "0|");
            continue;
        }

        if(new_package.type == TCPPACKAGE_MSG){
            msgContent += new_package.content;
            if(new_package.label != "GJMD"){
                cout << ">>> recv msg: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum << ", " << msgContent << endl;
                //DBGPRINT(DBG_SERVER, ">>> recv msg: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum << ", " << msgContent );
            }
            if(new_package.num < new_package.allnum){
                continue;
            }

            tcpServer->excuteCommand(accept_fd, new_package.src, new_package.label, msgContent);
            msgContent.clear();
        }
        else if(new_package.type == TCPPACKAGE_FILE){
            cout << ">>> recv file: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum << endl;
            //DBGPRINT(DBG_SERVER,">>> recv file: " << new_package.label << ", " << new_package.num << "/" << new_package.allnum);
            if(accept_fd != tcpServer->m_control_fd){
                continue;
            }

            if(!out.is_open()){
                out.open((tcpServer->m_strPath+ ROBOT_TEMP_RECV_FILE).c_str(), ios::out | ios::binary | ios::ate);
                if(!out){
                    printf("Open File failed\n");
                    break;
                }
                out.seekg(0, ios::beg);
            }

            out.write(new_package.content.c_str(), new_package.content.length());

            if(new_package.num < new_package.allnum){
                continue;
            }

            printf("file from server %d is received\n", accept_fd);
            out.close();

            //dos_to_unix(ROBOT_TEMP_RECV_FILE);

            tcpServer->excuteFile(accept_fd, new_package.src, new_package.label);
        }
    }

    if(accept_fd == tcpServer->m_control_fd){
        tcpServer->m_control_fd = -1;
    }

    if(tcpServer->m_mapClientType[accept_fd] == TCPSOURCE_OTHER){
        tcpServer->excuteCommand(accept_fd, TCPSOURCE_OTHER, "GBLJ", "1");
    }

    tcpServer->m_mapClientNum[tcpServer->m_mapClientType[accept_fd]]--;
    tcpServer->m_mapClientThread.erase(accept_fd);
    tcpServer->m_mapClientType.erase(accept_fd);

    printf("end recv thread\n");
	return 0;
}

int TCPServer::send_package(int fd, PagTCP& pag){
    string str = PAGHEAD;
    str += pag.label;
    str += num_to_string(pag.src) + SEGCHAR;
    str += num_to_string(pag.type) + SEGCHAR;
    str += num_to_string(pag.num) + SEGCHAR;
    str += num_to_string(pag.allnum) + SEGCHAR;
    str += pag.content;
    str += PAGTAIL;

//    if(pag.type == TCPPACKAGE_MSG){
//        cout << ">>> send msg: " << pag.label << ", " << pag.num << "/" << pag.allnum << ", " << pag.content << endl;
//    }
//    else if(pag.type == TCPPACKAGE_FILE){
//        cout << ">>> send file: " << pag.label << ", " << pag.num << "/" << pag.allnum << endl;
//    }

    if(send(fd, str.c_str(), str.length(), 0) < 0){
        printf("send TCP failed: %d\t%s\t%s\n", errno ,strerror(errno),str.c_str());
        close(fd);
        return -1;
    }

    return 0;
}

void TCPServer::send_msg_exceptFd(int client_fd, const char *label, string msg){
    for(map<int, TCPSOURCETYPE>::iterator it = m_mapClientType.begin(); it != m_mapClientType.end(); it++){
        if(TCPSOURCE_CLIENT == it->second && client_fd != it->first){
            send_msg(it->first, label, msg);
        }
    }
}

void TCPServer::send_msg(int client_fd, const char *label, string msg){
    m_tcp_mutex.lockMutex();

    int size = msg.length();
    int time = size/PAGSIZE + 1;
    for(int i = 0; i < time; i++){
        int length = PAGSIZE;
        if(i == time - 1){
            length = size%PAGSIZE;
        }
        PagTCP pag;
        pag.label = string(label);
        pag.src = TCPSOURCE_SERVER;
        pag.type = TCPPACKAGE_MSG;
        pag.num = i+1;
        pag.allnum = time;
        pag.content = msg.substr(i * PAGSIZE, length);

        if(send_package(client_fd, pag) < 0){
            m_tcp_mutex.unlockMutex();
            return;
        }
    }

    m_tcp_mutex.unlockMutex();
}

void TCPServer::send_file(int client_fd, const char *label, const char *filePath){
    m_tcp_mutex.lockMutex();
    char * buffer;
    int size;
    fstream in;
    in.open(filePath, ios::in|ios::binary|ios::ate);
    if(!in){
        m_tcp_mutex.unlockMutex();
        printf("Open File failed\n");
        return;
    }
    size = in.tellg();
    in.seekg (0, ios::beg);
    buffer = new char [size];
    in.read (buffer, size);
    in.close();
    string fileBuf(buffer);
    delete[] buffer;

    int time = size/PAGSIZE + 1;
    printf(">>> start to send file %s\n", label);
    for(int i = 0; i < time; i++){
        int length = PAGSIZE;
        if(i == time - 1){
            length = size%PAGSIZE;
        }
        PagTCP pag;
        pag.label = string(label);
        pag.src = TCPSOURCE_SERVER;
        pag.type = TCPPACKAGE_FILE;
        pag.num = i+1;
        pag.allnum = time;
        pag.content = fileBuf.substr(i * PAGSIZE, length);

        if(send_package(client_fd, pag) < 0){
            m_tcp_mutex.unlockMutex();
            return;
        }
    }
    printf(">>> finish sending file\n");
    m_tcp_mutex.unlockMutex();
}

void TCPServer::send_msg_type(TCPSOURCETYPE src, const char *label, string content){
    for(map<int, TCPSOURCETYPE>::iterator it = m_mapClientType.begin(); it != m_mapClientType.end(); it++){
        if(src == it->second){
            send_msg(it->first, label, content);
        }
    }
}

void TCPServer::send_file_type(TCPSOURCETYPE src, const char *label, const char *filePath){
    for(map<int, TCPSOURCETYPE>::iterator it = m_mapClientType.begin(); it != m_mapClientType.end(); it++){
        if(src == it->second){
            send_file(it->first, label, filePath);
        }
    }
}

void TCPServer::setControlFd(int fd){
    if(-1==m_control_fd)
        m_control_fd = fd;
}

bool TCPServer::isControlFd(int fd){
    if(fd == m_control_fd){
        return true;
    }
    return false;
}

int TCPServer::controlFd(){
    return m_control_fd;
}

int TCPServer::setFirstConnect(int fd, TCPSOURCETYPE src){
    if(isFirstConnect(fd)){
        tcpServer->m_mapClientType[fd] = src;
        tcpServer->m_mapClientNum[src]++;

        if(tcpServer->m_mapClientNum[TCPSOURCE_CLIENT] > CLIENT_MAXCONNECT || tcpServer->m_mapClientNum[TCPSOURCE_OTHER] > OTHER_MAXCONNECT){
            close(fd);
            return 1;
        }
    }
    return 0;
}

bool TCPServer::isFirstConnect(int fd){
    if(tcpServer->m_mapClientType[fd] == TCPSOURCE_UNDEFINE){ // first connect
        return true;
    }
    else{
        return false;
    }
}

void TCPServer::excuteCommand(int fd, TCPSOURCETYPE src, string command, string content){
    printf("deal new command\n");
}

void TCPServer::excuteFile(int fd, TCPSOURCETYPE src, string command){
    printf("deal new file\n");
}

int string_to_int(const string &p_content){
    int m_res;
    if(p_content == "TRUE" || p_content == "ON"){
        m_res = 1;
    }
    else if(p_content == "FALSE" || p_content == "OFF"){
        m_res = 0;
    }
    else{
        stringstream ss;
        ss << p_content;
        ss >> m_res;
    }
    return m_res;
}

string num_to_string(int p_int){
    stringstream ss;
    ss << p_int;
    return ss.str();
}
