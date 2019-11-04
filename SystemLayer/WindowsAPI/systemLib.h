#ifndef SYSTEMLIB_H
#define SYSTEMLIB_H

#include <windows.h>
#ifndef _WINSOCK2API_
#define _WINSOCK2API_
#define _WINSOCKAPI_
#endif

#include <Winsock2.h>

#include <time.h>
#define DECLSPEC_DLLEXPORT  _declspec(dllexport)
#ifndef  DLL_EXPORT
#define MY_CLASS	class  _declspec(dllimport)
#else
#define MY_CLASS	class  _declspec(dllexport)
#endif

void sleep_ms(unsigned int time);

namespace rclib{

class Timer{
public:
	Timer();
	 ~Timer();

	void start(double periodTime);   // 开始计时
	void wait();    // 到下一个始终周期唤
private:
	struct timeval  timespec_add(struct timeval  time1, struct timeval  time2);
	void getCurrentTime(struct timeval&);
	struct timeval   m_periodTime;	
	struct timeval   m_preTime;
};

class Mutex{
public:
    Mutex();
	_declspec(dllexport)  ~Mutex();
    void lockMutex();
    void unlockMutex();
private:
    void* m_mutex;
};

#define THREAD_ROUTINE_RETURN unsigned int
typedef struct tagTHREADNAME_INFO
{
	DWORD dwType; // Must be 0x1000.
	LPCSTR szName; // Pointer to name (in user addr space).
	DWORD dwThreadID; // Thread ID (-1=caller thread).
	DWORD dwFlags; // Reserved for future use, must be zero.
} THREADNAME_INFO;

MY_CLASS Thread{
public:
    Thread();
    Thread(THREAD_ROUTINE_RETURN (_stdcall* start_routine)(void*), void* arg);
	inline  ~Thread();
    int create(THREAD_ROUTINE_RETURN (_stdcall *start_routine)(void*), void* arg);
    void join();
    void cancel();
	HANDLE getHandle(){ return m_lpHandle; }
private:
	HANDLE m_lpHandle;
    UINT32 m_id;
	unsigned m_stackSize;
	void thread_name(const char* str);
};

void thread_detach();
void thread_exit(); //windows存在
void thread_name(const char* str);
void dos_to_unix(const char* str);


}

#endif
