//
// Created by root on 2020/11/15.
//

#ifndef TEST_THREAD_H
#define TEST_THREAD_H

#include <pthread.h>
#include "Sem.h"
#include "scope_mutex.h"

/**
 * Calling thread sleeps for some milliseconds.
 */
inline void uSleep(unsigned int ms)
{
    struct timespec req;
    struct timespec rem;
    req.tv_sec = ms / 1000;
    req.tv_nsec = (ms - req.tv_sec * 1000) * 1000 * 1000;
    nanosleep (&req, &rem);
}

/**
 * Calling thread sleeps for some microseconds.
 */
inline void uSleepMicro(unsigned int us)
{
    struct timespec req;
    struct timespec rem;
    req.tv_sec = us / 1000000;
    req.tv_nsec = (us - req.tv_sec * 1000000) * 1000;
    nanosleep (&req, &rem);
}

/**
 * Calling thread sleeps for some nanoseconds.
 */
inline void uSleepNano(unsigned int ns)
{
    struct timespec req;
    struct timespec rem;
    req.tv_sec = ns / 1000000000;
    req.tv_nsec = (ns - req.tv_sec * 1000000000);
    nanosleep (&req, &rem);
}


#define InvalidHandle 0
#define THREAD_HANDLE pthread_t

typedef void *( * pthread_fn )( void * );

template
        <
                typename Thread_T
        >
class ThreadC
{
private:
    struct Instance;

public:
    typedef Thread_T              & Thread_R;
    typedef const Thread_T        & Thread_C_R;

    typedef THREAD_HANDLE Handle;
    typedef void ( *Handler)( Thread_R );

    virtual ~ThreadC() {}

protected:
    ThreadC() {}

    virtual void ThreadMain( Thread_R ) = 0;

    static void Exit()
    { pthread_exit(0); }
    static void TestCancel()
    { pthread_testcancel(); }

    static Handle Self()
    { return (Handle)pthread_self(); }

public:

    static int Create(
            const Handler       & Function,
            Thread_C_R            Param,
            Handle  * const     & H               = 0,
            const bool          & CreateDetached  = false,
            const unsigned int  & StackSize       = 0,
            const bool          & CancelEnable    = false,
            const bool          & CancelAsync     = false
    )
    {
        M_Create().lock();
        pthread_attr_t attr;
        pthread_attr_init(&attr);

        if ( CreateDetached )
            pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

        if ( StackSize )
            pthread_attr_setstacksize(&attr,StackSize);

        Instance I(Param,0,Function,CancelEnable,CancelAsync);

        Handle h=InvalidHandle;
        int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

        pthread_attr_destroy(&attr);

        if(H) *H = h;
        if ( !R ) S_Create().acquire();

        M_Create().unlock();
        return R;
    }

    int Create(
            Thread_C_R            Param,
            Handle  * const     & H               = 0,
            const bool          & CreateDetached  = false,
            const unsigned int  & StackSize       = 0,
            const bool          & CancelEnable    = false,
            const bool          & CancelAsync     = false
    ) const
    {
        M_Create().lock();
        pthread_attr_t attr;
        pthread_attr_init(&attr);

        if ( CreateDetached )
            pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

        if ( StackSize )
            pthread_attr_setstacksize(&attr,StackSize);

        Instance I(Param,const_cast<ThreadC *>(this),0,CancelEnable,CancelAsync);

        Handle h=InvalidHandle;
        int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

        pthread_attr_destroy(&attr);

        if(H) *H = h;
        if ( !R ) S_Create().acquire();

        M_Create().unlock();
        return R;
    }

    static int Join( Handle H )
    { return pthread_join(H,0); }

    static int Kill( Handle H )
    { return pthread_cancel(H); }

    static int Detach( Handle H )
    { return pthread_detach(H); }

private:

    static const Mutex &M_Create()      { static Mutex M; return M; }
    static Semaphore &S_Create()  { static Semaphore S; return S; }

    static void *ThreadMainHandler( Instance *Param )
    {
        Instance  I(*Param);
        Thread_T  Data(I.Data);
        S_Create().release();

        if ( I.Flags & 1 /*CancelEnable*/ )
        {
            pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);

            if ( I.Flags & 2 /*CancelAsync*/ )
                pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
            else
                pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);
        }
        else
        {
            pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL);
        }

        if ( I.Owner )
            I.Owner->ThreadMain(Data);
        else
            I.pFN(Data);

        return 0;
    }

    struct Instance
    {
        Instance( Thread_C_R P, ThreadC<Thread_T> *const &O, const ThreadC<Thread_T>::Handler &pH = 0, const bool &CE=false, const bool &CA=false )
                : Data(P), Owner(O), pFN(pH), Flags(0) { if ( CE ) Flags|=1; if ( CA ) Flags|=2; }

        Thread_C_R      Data;
        ThreadC<Thread_T>                * Owner;
        Handler         pFN;
        unsigned char                     Flags;
    };
};

/////////////////////////////////////////////////////////////////////
//  Explicit specialization, no thread parameters
//
template<>
class ThreadC<void>
{
private:
    struct Instance;

public:
    typedef THREAD_HANDLE Handle;
    typedef void ( *Handler)();

    virtual ~ThreadC<void>() {}

protected:
    ThreadC<void>() {}

    virtual void ThreadMain() = 0;

    static void Exit()
    { pthread_exit(0); }

    static void TestCancel()
    { pthread_testcancel(); }


    static Handle Self()
    { return (Handle)pthread_self(); }

public:

    static int Create(
            const Handler       & Function,
            Handle  * const     & H               = 0,
            const bool          & CreateDetached  = false,
            const unsigned int  & StackSize       = 0,
            const bool          & CancelEnable    = false,
            const bool          & CancelAsync     = false
    )
    {
        M_Create().lock();
        pthread_attr_t attr;
        pthread_attr_init(&attr);

        if ( CreateDetached )
            pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

        if ( StackSize )
            pthread_attr_setstacksize(&attr,StackSize);

        Instance I(0,Function,CancelEnable,CancelAsync);

        Handle h=InvalidHandle;
        int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

        pthread_attr_destroy(&attr);

        if(H) *H = h;
        if ( !R ) S_Create().acquire();

        M_Create().unlock();
        return R;
    }

    int Create(
            Handle  * const     & H               = 0,
            const bool          & CreateDetached  = false,
            const unsigned int  & StackSize       = 0,
            const bool          & CancelEnable    = false,
            const bool          & CancelAsync     = false
    ) const
    {
        M_Create().lock();
        pthread_attr_t attr;
        pthread_attr_init(&attr);

        if ( CreateDetached )
            pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

        if ( StackSize )
            pthread_attr_setstacksize(&attr,StackSize);

        Instance I(const_cast<ThreadC *>(this),0,CancelEnable,CancelAsync);

        Handle h=InvalidHandle;
        int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

        pthread_attr_destroy(&attr);

        if(H) *H = h;
        if ( !R ) S_Create().acquire();

        M_Create().unlock();
        return R;
    }

    int Create(
            unsigned long        & ThreadId,
            Handle  * const     & H               = 0,
            const bool          & CreateDetached  = false,
            const unsigned int  & StackSize       = 0,
            const bool          & CancelEnable    = false,
            const bool          & CancelAsync     = false
    ) const
    {
        M_Create().lock();
        pthread_attr_t attr;
        pthread_attr_init(&attr);

        if ( CreateDetached )
            pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

        if ( StackSize )
            pthread_attr_setstacksize(&attr,StackSize);

        Instance I(const_cast<ThreadC *>(this),0,CancelEnable,CancelAsync);

        *H = InvalidHandle;
        int R = pthread_create((pthread_t *)&(*H),&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

        ThreadId = (unsigned long)*H;

        pthread_attr_destroy(&attr);

        if ( !R ) S_Create().acquire();

        M_Create().unlock();
        return R;
    }

    static int Join( Handle H )
    { return pthread_join(H,0); }

#ifndef ANDROID
    static int Kill( Handle H )
    { return pthread_cancel(H); }
#endif

    static int Detach( Handle H )
    { return pthread_detach(H); }

private:

    static const Mutex &M_Create()      { static Mutex M; return M; }
    static Semaphore &S_Create()  { static Semaphore S; return S; }

    static void *ThreadMainHandler( Instance *Param )
    {
        Instance  I(*Param);
        S_Create().release();

#ifndef ANDROID
        if ( I.Flags & 1 /*CancelEnable*/ )
        {
            pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);

            if ( I.Flags & 2 /*CancelAsync*/ )
                pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
            else
                pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);
        }
        else
        {
            pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL);
        }
#endif

        if ( I.Owner )
            I.Owner->ThreadMain();
        else
            I.pFN();

        return 0;
    }

    struct Instance
    {
        Instance( ThreadC<void> *const &O, const ThreadC<void>::Handler &pH = 0, const bool &CE=false, const bool &CA=false )
                : pFN(pH), Owner(O), Flags(0) { if ( CE ) Flags|=1; if ( CA ) Flags|=2; }

        ThreadC<void>::Handler             pFN;
        ThreadC<void>                *     Owner;
        unsigned char                     Flags;

    };
};

class  Thread : public ThreadC<void>
{
public:
/**
 * Enum of priorities : kPLow, kPBelowNormal, kPNormal, kPAboveNormal, kPRealTime.
 */
    enum Priority{kPLow, kPBelowNormal, kPNormal, kPAboveNormal, kPRealTime};

public:
//return caller thread id
    static unsigned long currentThreadId() {return (unsigned long)ThreadC<void>::Self();}

public:
/**
 * The constructor.
 * @see Priority
 * @param priority the thread priority
 */
    Thread(Priority priority = kPNormal);

/**
 * The destructor. Inherited classes must call join(true) inside their destructor
 * to avoid memory leaks where the underlying c-thread is still running.
 *
 * Note: not safe to delete a thread while other threads are joining it.
 */
    virtual ~Thread();

/**
 * Start the thread. Once the thread is started, subsequent calls
 * to start() are ignored until the thread is killed.
 * @see kill()
 */
    void start();

/**
 * Kill the thread.
 * This functions does nothing if the thread is not started or is killed.
 *
 * Note : not a blocking call
 */
    void kill();

/**
 * The caller thread will wait until the thread has finished.
 *
 * Note : blocking call
 * @param killFirst if you want kill() to be called before joining (default false), otherwise not.
 */
    void join(bool killFirst = false);

/**
 * Set the thread priority.
 * @param priority the priority
 */
    void setPriority(Priority priority);

/**
 * Set the thread affinity. This is applied during start of the thread.
 *
 * MAC OS X : http://developer.apple.com/library/mac/#releasenotes/Performance/RN-AffinityAPI/_index.html.
 * @param cpu the cpu id (start at 1), 0 means no affinity (default).
 */
    void setAffinity(int cpu = 0);

/**
 * @return if the state of the thread is kSCreating (after start() is called but before entering the mainLoop()).
 */
    bool isCreating() const;

/**
 * @return if the state of the thread is kSRunning (it is executing the mainLoop()) or kSCreating.
 */
    bool isRunning() const;

/**
 * @return if the state of the thread is kSIdle (before start() is called and after the thread is totally killed (or after join(true))).
 */
    bool isIdle() const;

/**
 * @return if the state of the thread is kSKilled (after kill() is called and before the thread is totally killed).
 */
    bool isKilled() const;

    Handle getThreadHandle() const {return handle_;}
    unsigned long getThreadId() const {return threadId_;}

protected:

private:
/**
 * Virtual method mainLoopBegin().
 * User can implement this function to add a behavior
 * before the main loop is started. It is
 * called once (before entering mainLoop()).
 */
    virtual void mainLoopBegin() {}

/**
 * Pure virtual method mainLoop().
 * The inner loop of the thread. This method is called repetitively
 * until the thread is killed. Note that if kill() is called in mainLoopBegin(),
 * mainLoop() is not called, terminating immediately the thread.
 *
 * @see mainLoop()
 * @see kill()
 */
    virtual void mainLoop() = 0;

/**
 * Virtual method mainLoopKill().
 * User can implement this function to add a behavior
 * before the thread is killed. When this
 * function is called, the state of the thread is set to kSKilled. It is useful to
 * wake up a sleeping thread to finish his loop and to avoid a deadlock.
 */
    virtual void mainLoopKill() {}

/**
 * Virtual method mainLoopEnd().
 * User can implement this function to add a behavior
 * after the thread is killed (after exiting the mainLoop(), work is
 * still done in the thread before exiting).
 */
    virtual void mainLoopEnd() {}

/*
 * Inherited method ThreadMain() from Thread.
 * @see Thread<void>
 */
    void ThreadMain();

/*
 * Apply thread priority. This is called when starting the thread.
 * *@todo : Support pthread
 */
    void applyPriority();

/*
 * Apply cpu affinity. This is called when starting the thread.
 * *@todo : Support Windows
 */
    void applyAffinity();

/*
 * Inherited method Create() from Thread.
 * Here we force this function to be private so the
 * inherited class can't have access to it.
 * @see Thread<void>
 */
    int Create(
            Handle  * const     & H               = 0,
            const bool          & CreateDetached  = false,
            const unsigned int  & StackSize       = 0,
            const bool          & CancelEnable    = false,   // UNUSED
            const bool          & CancelAsync     = false    // UNUSED
    ) const;

//Methods from UThread<void> class hided
    static int Join( Handle H )
    { return ThreadC<void>::Join(H); }
#ifndef ANDROID
    static int Kill( Handle H )
    { return ThreadC<void>::Kill(H); }
#endif
    static int Detach( Handle H )
    { return ThreadC<void>::Detach(H); }

private:
    void operator=(Thread &) {}
    Thread( const Thread &) : state_(kSIdle) {}

private:
    enum State{kSIdle, kSCreating, kSRunning, kSKilled}; /* Enum of states. */
    State state_; 			/* The thread state. */
    Priority priority_; 	/* The thread priority. */
    Handle handle_; 	/* The thread handle. */
    unsigned long threadId_; /* The thread id. */
    int cpuAffinity_; /* The cpu affinity. */
    Mutex killSafelyMutex_;	/* Mutex used to protect the kill() method. */
    Mutex runningMutex_;	    /* Mutex used to notify the join method when the thread has finished. */
};




#endif //TEST_THREAD_H
