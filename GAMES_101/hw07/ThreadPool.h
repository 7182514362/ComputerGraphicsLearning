#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

// struct Task {
//     std::function<void()> func;
//     void operator()() { func(); }
// };

class ThreadPool
{
public:
    using Task = std::function<void()>;

private:
    std::queue<Task> m_tasks;
    std::vector<std::thread> m_threads;
    std::mutex m_mutex;
    std::condition_variable m_condVar;
    std::atomic_bool m_stop{false};

public:
    ThreadPool()
    {
        for (int i = 0; i < std::thread::hardware_concurrency() - 1; ++i) {
            m_threads.emplace_back([this]() {
                while (!m_stop.load()) {
                    Task task = popTask();
                    if (task) {
                        task();
                    }
                }
            });
        }
    }
    ~ThreadPool()
    {
        stop();
        wait_all();
    }

    void submit(Task task)
    {
        auto lock = std::lock_guard(m_mutex);
        m_tasks.push(std::move(task));
    }

    Task popTask()
    {
        std::unique_lock<std::mutex> lck(m_mutex);
        while (m_tasks.empty() && !m_stop.load()) {
            m_condVar.wait(lck);
        }
        if (m_tasks.empty()) {
            return {};
        }
        Task task = std::move(m_tasks.front());
        m_tasks.pop();
        return std::move(task);
    }

    void stop()
    {
        m_stop.store(true);
        m_condVar.notify_all();
        std::cout << "\nstop" << std::endl;
    }

    void wait_all()
    {
        for (auto& t : m_threads) {
            if (t.joinable()) {
                t.join();
            }
        }
        std::cout << "\nwait" << std::endl;
    }

    void run()
    {
        m_stop.store(false);
        m_condVar.notify_all();
        std::cout << "\nrun" << std::endl;
    }
};