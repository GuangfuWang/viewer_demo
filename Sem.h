//
// Created by root on 2020/11/15.
//

#ifndef TEST_SEM_H
#define TEST_SEM_H

#include <bits/pthreadtypes.h>
#include <pthread.h>
#include <sys/time.h>

class Semaphore
{
public:
    /**
     * The constructor. The semaphore waits on acquire() when its value is <= 0.
     * @param n number to initialize
     */
    Semaphore( int initValue = 0 )
    {
        _available = initValue;
        pthread_mutex_init(&_waitMutex, NULL);
        pthread_cond_init(&_cond, NULL);

    }

    virtual ~Semaphore()
    {
        pthread_cond_destroy(&_cond);
        pthread_mutex_destroy(&_waitMutex);
    }

    /**
     * Acquire the semaphore. If semaphore's value is <=0, the
     * calling thread will wait until the count acquired is released.
     * @see release()
     * @param n number to acquire
     * @param t time to wait (ms), a value <=0 means infinite
     * @return true on success, false on error/timeout
     */
    bool acquire(int n = 1, int ms = 0)
    {
        int rt = 0;
        pthread_mutex_lock(&_waitMutex);
        while (n > _available && rt == 0)
        {
            if(ms > 0)
            {
                struct timespec timeToWait;
                struct timeval now;

                gettimeofday(&now,NULL);

                timeToWait.tv_sec = now.tv_sec + ms/1000;
                timeToWait.tv_nsec = (now.tv_usec+1000UL*(ms%1000))*1000UL;

                rt = pthread_cond_timedwait(&_cond, &_waitMutex, &timeToWait);
            }
            else
            {
                rt = pthread_cond_wait(&_cond, &_waitMutex);
            }
        }
        if(rt == 0)
        {
            // only remove them if waiting did not fail
            _available -= n;
        }
        pthread_mutex_unlock(&_waitMutex);
        return rt == 0;
    }

    /*
     * Try to acquire the semaphore, not a blocking call.
     * @return false if the semaphore can't be taken without waiting (value <= 0), true otherwise
     */

    int acquireTry(int n)
    {
        pthread_mutex_lock(&_waitMutex);
        if(n > _available)
        {
            pthread_mutex_unlock(&_waitMutex);
            return false;
        }
        _available -= n;
        pthread_mutex_unlock(&_waitMutex);
        return true;
    }


    /**
     * Release the semaphore, increasing its value by 1 and
     * signaling waiting threads (which called acquire()).
     */

    void release(int n = 1)
    {
        pthread_mutex_lock(&_waitMutex);
        _available += n;
        pthread_cond_broadcast(&_cond);
        pthread_mutex_unlock(&_waitMutex);
    }

    /**
     * Get the USempahore's value.
     * @return the semaphore's value
     */

    int value()
    {
        int value = 0;
        pthread_mutex_lock(&_waitMutex);
        value = _available;
        pthread_mutex_unlock(&_waitMutex);
        return value;
    }


private:
    void operator=(const Semaphore &){}

    Semaphore(const Semaphore &):_available(0){}
    pthread_mutex_t _waitMutex;
    pthread_cond_t _cond;
    int _available;
};

#endif //TEST_SEM_H
