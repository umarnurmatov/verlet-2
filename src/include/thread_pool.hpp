#pragma once
#include <queue>
#include <thread>
#include <atomic>
#include <mutex>
#include <future>
#include <unordered_set>

class ThreadPool
{
public:
    ThreadPool(uint32_t num_threads)
    : end_work{false}
    , next_id{0}
    {
        thread_pool.reserve(num_threads);
        for(uint32_t i = 0; i < num_threads; i++)
            thread_pool.emplace_back(&ThreadPool::run, this);
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;


    void run(); 
    void wait(int64_t task_id);
    void wait_all();
    bool is_completed(int64_t task_id);

    template<typename Func, typename ...Args>
    int64_t add_task(const Func& task_func, Args&... args);

    // Waits until all threads finish their current task
    // and joins them (queue could be not empty)
    void terminate();


    // Waits until queue is empty
    // and joins threads
    ~ThreadPool();

private:
    std::queue<std::pair<std::future<void>, int64_t>> task_queue;

    std::mutex queue_mtx;
    std::condition_variable queue_cv;

    std::unordered_set<int64_t> completed_task_id;
    std::mutex completed_task_id_mtx;
    std::condition_variable completed_task_id_cv;

    std::vector<std::thread> thread_pool;

    std::atomic<bool> end_work; // флаг завершения работы ThreadPool
    std::atomic<int64_t> next_id; // ID следующего потока

};

template <typename Func, typename... Args>
inline int64_t ThreadPool::add_task(const Func &task_func, Args &...args)
{
    int64_t task_id = next_id++;

    std::lock_guard<std::mutex> queue_lock(queue_mtx);
    task_queue.emplace(std::async(std::launch::deferred, task_func, args...), task_id);

    queue_cv.notify_one();
    return task_id;
}
