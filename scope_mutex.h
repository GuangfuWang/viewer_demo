//
// Created by root on 2020/11/15.
//

#ifndef TEST_SCOPE_MUTEX_H
#define TEST_SCOPE_MUTEX_H



#include <pthread.h>

class Mutex
{
public:
    /**
     * The constructor.
     */
    Mutex()
    {

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
        pthread_mutex_init(&M,&attr);
        pthread_mutexattr_destroy(&attr);
    }
    virtual ~Mutex()
    {
        pthread_mutex_unlock(&M); pthread_mutex_destroy(&M);
    }
    /**
     * Lock the mutex.
     */
    int lock() const
    {
        return pthread_mutex_lock(&M);
    }
    int lockTry() const
    {
        return pthread_mutex_trylock(&M);
    }
    /**
     * Unlock the mutex.
     */
    int unlock() const
    {
        return pthread_mutex_unlock(&M);
    }
private:
    mutable pthread_mutex_t M;

    void operator=(Mutex &) {}
    Mutex( const Mutex & ) {}
};

class ScopeMutex
{
public:
    ScopeMutex(const Mutex & mutex) :
            mutex_(mutex)
    {
        mutex_.lock();
    }
    // backward compatibility
    ScopeMutex(Mutex * mutex) :
            mutex_(*mutex)
    {
        mutex_.lock();
    }
    ~ScopeMutex()
    {
        mutex_.unlock();
    }
private:
    const Mutex & mutex_;
};



#endif //TEST_SCOPE_MUTEX_H
