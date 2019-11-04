#include "../systemLib.h"
#include <process.h>
#include <stdio.h>

using namespace rclib;

void sleep_ms(unsigned int time){
    Sleep(time);
}

#define NSEC_PER_SEC (1000000000L)

Timer::Timer(){

}

Timer::~Timer(){

}

void Timer::getCurrentTime(struct timeval&cTm)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	cTm.tv_sec = clock;
	cTm.tv_usec = wtm.wMilliseconds * 1000;
};
void Timer::start(double periodTime){
	m_periodTime.tv_usec = periodTime*1000000;
	m_periodTime.tv_sec = 0;
	getCurrentTime(m_preTime);
}

struct timeval  Timer::timespec_add(struct timeval  time1, struct timeval  time2){
	struct timeval  result;

	if ((time1.tv_usec + time2.tv_usec) >= NSEC_PER_SEC/1000){
		result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		result.tv_usec = time1.tv_usec + time2.tv_usec - NSEC_PER_SEC/1000;
	}
	else{
		result.tv_sec = time1.tv_sec + time2.tv_sec;
		result.tv_usec = time1.tv_usec + time2.tv_usec;
	}

	return result;
}

void Timer::wait(){
        Sleep(m_periodTime.tv_sec*1000+m_periodTime.tv_usec/1000);
}

Mutex::Mutex(){
    m_mutex = CreateMutex(NULL, FALSE, NULL);
}

Mutex::~Mutex(){
    CloseHandle(m_mutex);
}

void Mutex::lockMutex(){
    WaitForSingleObject(m_mutex, INFINITE);
}

void Mutex::unlockMutex(){
    ReleaseMutex(m_mutex);
}



Thread::Thread(){

}

Thread::Thread(THREAD_ROUTINE_RETURN(_stdcall*start_routine)(void*), void *arg){

}

Thread::~Thread(){
	//cancel();
}

int Thread::create(THREAD_ROUTINE_RETURN(_stdcall*start_routine)(void*), void *arg){
	//int ret = pthread_create(&m_id, NULL, start_routine, arg);
	//m_lpHandle = (HANDLE)::_beginthreadex(NULL, m_stackSize, start_routine, (Thread*)this, NULL, &m_id);
	m_lpHandle = (HANDLE)::_beginthreadex(NULL, 0, start_routine, arg, 0, NULL);
	if (NULL == m_lpHandle)
	{
		int n = errno;
		printf("_beginthreadex, error=%s\n",GetLastError());
	}
	else
	{
		//DBGPRINT(DBG_THREAD, _T("创建新线程:") << _lpThreadId[i]);
	}
	return 0;
}

void Thread::join(){
	//pthread_join(m_id, NULL);
	INT32 iStatus = -1;

	if (::WaitForSingleObject(m_lpHandle, INFINITE) == WAIT_TIMEOUT)
	{
		//DBGPRINT(DBG_ERROR, _T("线程超时dwMilliseconds:") << dwMilliseconds);
		::TerminateThread(m_lpHandle, 0);
	}
	::GetExitCodeThread(m_lpHandle, (LPDWORD)&iStatus);
}

void Thread::cancel(){
	//pthread_cancel(m_id);
	if (m_lpHandle)
	{
		::TerminateThread(m_lpHandle, 0);
		::CloseHandle(m_lpHandle);
	}
}

void rclib::Thread::thread_name(const char* str)
{
#if 0
	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = str;
	info.dwThreadID = m_id;
	info.dwFlags = 0;

	__try
	{
		RaiseException(0x406D1388, 0, sizeof(info) / sizeof(DWORD), (DWORD*)&info);
	}
	__except (EXCEPTION_CONTINUE_EXECUTION)
	{
	}
#endif
}

void rclib::thread_detach()
{

}

void rclib::thread_exit()
{

}
void rclib::thread_name(const char* str)
{

}
void rclib::dos_to_unix(const char* str)
{

}

