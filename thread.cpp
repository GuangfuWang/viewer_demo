//
// Created by root on 2020/11/15.
//

#include <cstdio>
#include "thread.h"

Thread::Thread(Priority priority) :
        state_(kSIdle),
        priority_(priority),
        handle_(0),
        threadId_(0),
        cpuAffinity_(-1)
{}

Thread::~Thread()
{
}

void Thread::kill()
{
    killSafelyMutex_.lock();
    {
        if(this->isRunning())
        {
            // Thread is creating
            while(state_ == kSCreating)
            {
                uSleep(1);
            }

            if(state_ == kSRunning)
            {
                state_ = kSKilled;

                // Call function to do something before wait
                mainLoopKill();
            }
            else
            {
                printf("thread (%d) is supposed to be running...", threadId_);
            }
        }
        else
        {
        }
    }
    killSafelyMutex_.unlock();
}

void Thread::join(bool killFirst)
{
    //make sure the thread is created
    while(this->isCreating())
    {
        uSleep(1);
    }
    if(ThreadC<void>::Self() == handle_)
    {
        printf("Thread cannot join itself");
        return;
    }
    if(killFirst)
    {
        this->kill();
    }
    runningMutex_.lock();
    runningMutex_.unlock();
}

void Thread::start()
{
    if(state_ == kSIdle || state_ == kSKilled)
    {
        if(state_ == kSKilled)
        {
            // make sure it is finished
            runningMutex_.lock();
            runningMutex_.unlock();
        }
        state_ = kSCreating;
        int r = ThreadC<void>::Create(threadId_, &handle_, true); // Create detached
        if(r)
        {
            printf("Failed to create a thread! errno=%d", r);
            threadId_=0;
            handle_=0;
            state_ = kSIdle;
        }
        else
        {
        }
    }
}

void Thread::setPriority(Priority priority)
{
    priority_ = priority;
}

void Thread::applyPriority()
{
    if(handle_)
    {
    }
}

void Thread::setAffinity(int cpu)
{
    cpuAffinity_ = cpu;
    if(cpuAffinity_<0)
    {
        cpuAffinity_ = 0;
    }
}

void Thread::applyAffinity()
{
    if(cpuAffinity_>0)
    {
    }
}

bool Thread::isCreating() const
{
    return state_ == kSCreating;
}

bool Thread::isRunning() const
{
    return state_ == kSRunning || state_ == kSCreating;
}

bool Thread::isIdle() const
{
    return state_ == kSIdle;
}

bool Thread::isKilled() const
{
    return state_ == kSKilled;
}

void Thread::ThreadMain()
{
    runningMutex_.lock();
    applyPriority();
    applyAffinity();

    state_ = kSRunning;
    mainLoopBegin();
    while(state_ == kSRunning)
    {
        mainLoop();
    }
    mainLoopEnd();
    handle_ = 0;
    threadId_ = 0;
    state_ = kSIdle;
    runningMutex_.unlock();
}

