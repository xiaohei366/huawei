#pragma once

#include <mutex>
#include <condition_variable>
#include <queue>
#include <thread>
#include <functional>
#include <iostream>

class ThreadPool {
public:
    explicit ThreadPool(size_t threadCount = 1) {
            for(size_t i = 0; i < threadCount; ++i) {
                //在创建对象的同时把线程池执行的任务用lambda写好
                std::thread([this] {
                    std::unique_lock<std::mutex> lock(mtx);
                    while(true) {
                        if(!tasks.empty()) {
                            auto task = std::move(tasks.front());
                            tasks.pop();
                            //这一步解锁，是为了线程在执行工作时，其他线程也能接取任务
                            lock.unlock();
                            task();
                            //因为初始加锁在循环外面，因此应再次上锁，以模拟取任务之前的上锁操作。
                            lock.lock();
                        } 
                        //第一个线程解锁的一刻，其余线程都来这等信号量了
                        else cond.wait(lock);
                    }
                }).detach();
            }
    }
    ~ThreadPool() = default;

    template<class F>
    void AddTask(F&& task) {
        //注意要单线程加操作
        tasks.emplace(std::forward<F>(task));
        cond.notify_one();
    }
private:
    int cnt = 0;
    std::mutex mtx;
    std::condition_variable cond;
    std::queue<std::function<void()>> tasks;
};
