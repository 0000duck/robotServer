#include "systemLib.h"
#include <sys/prctl.h>
#include <iostream>
#include <string>

using namespace std;
using namespace rclib;

namespace rclib{

void sleep_ms(unsigned int time){
    usleep(time * 1000);
}

}

#define NSEC_PER_SEC (1000000000L)

Timer::Timer(){

}

Timer::~Timer(){

}

void Timer::start(double periodTime){
    m_periodTime.tv_sec = 0;
    m_periodTime.tv_nsec = periodTime * 1000000000;

    clock_gettime(CLOCK_REALTIME, &m_wakeupTime);
}

void Timer::wait(){
    m_wakeupTime = timespec_add(m_wakeupTime, m_periodTime);
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &m_wakeupTime, NULL);
}

timespec Timer::timespec_add(timespec time1, timespec time2){
    timespec result;

    if((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC){
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else{
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

Mutex::Mutex(){
    pthread_mutex_init(&m_mutex, NULL);
}

Mutex::~Mutex(){
    pthread_mutex_destroy(&m_mutex);
}

void Mutex::lockMutex(){
    pthread_mutex_lock(&m_mutex);
}

void Mutex::unlockMutex(){
    pthread_mutex_unlock(&m_mutex);
}

Thread::Thread(){

}

Thread::Thread(void *(*start_routine)(void *), void *arg){

}

Thread::~Thread(){

}

int Thread::create(void *(*start_routine)(void *), void *arg){
    int ret = pthread_create(&m_id, NULL, start_routine, arg);
    return ret;
}

void Thread::join(){
    pthread_join(m_id, NULL);
}

void Thread::cancel(){
    pthread_cancel(m_id);
}

namespace rclib {

void thread_detach(){
    pthread_detach(pthread_self());
}

void thread_exit(){
    pthread_exit(nullptr);
}

void thread_name(const char *str){
    prctl(PR_SET_NAME, str);
}

void dos_to_unix(const char* fileName){
    string convert_file;
    convert_file = string("sudo dos2unix ") + string(fileName);
    cout << "convert file: " << convert_file << endl;
    FILE *fp;
    do{
        fp = popen(convert_file.c_str(), "r");
        if(!fp){
            printf("fail to convert file\n");
            usleep(100000);
        }
    }
    while(!fp);
    pclose(fp);
    printf("success to convert file\n");
}

}
