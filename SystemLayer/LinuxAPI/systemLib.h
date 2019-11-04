#ifndef SYSTEMLIB_H
#define SYSTEMLIB_H

#include <unistd.h>
#include <pthread.h>
#define  DBG_SERVER     0x6000      //服务端调试专用
#define  DBG_CLIENT     0x6001      //客户端调试专用

#define DECLSPEC_DLLEXPORT  
#define MY_CLASS  class

namespace rclib{

void sleep_ms(unsigned int time);

class Timer{
public:
    Timer();
    ~Timer();

    void start(double periodTime);   // 开始计时
    void wait();    // 到下一个始终周期唤醒

private:
    timespec timespec_add(timespec time1, timespec time2);

private:
    timespec m_periodTime;
    timespec m_wakeupTime;
};

class Mutex{
public:
    Mutex();
    ~Mutex();
    void lockMutex();
    void unlockMutex();
private:
    pthread_mutex_t m_mutex;
};

#define THREAD_ROUTINE_RETURN void*
class Thread{
public:
    Thread();
    Thread(THREAD_ROUTINE_RETURN (*start_routine)(void*), void* arg);
    ~Thread();
    int create(THREAD_ROUTINE_RETURN (*start_routine)(void*), void* arg);
    void join();
    void cancel();

private:
    pthread_t m_id;
};

void thread_detach();
void thread_exit();
void thread_name(const char* str);

void dos_to_unix(const char* str);

}

#endif
