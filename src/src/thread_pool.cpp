#include <thread_pool.hpp>

void ThreadPool::run()
{
    while(true)
    {
        std::unique_lock<std::mutex> queue_lock(queue_mtx);

        queue_cv.wait(queue_lock, [this]()->bool { return !task_queue.empty() || end_work; });
        if(end_work) break; // смотри в ~ThreadPool()

        auto elem = std::move(task_queue.front());
        task_queue.pop();
        queue_lock.unlock();

        // запускаем функцию
        elem.first.get();

        std::lock_guard<std::mutex> completed_task_lock(completed_task_id_mtx);
        completed_task_id.insert(elem.second);
        completed_task_id_cv.notify_all();
    }
}

void ThreadPool::wait(int64_t task_id)
{
    std::unique_lock<std::mutex> completed_task_lock(completed_task_id_mtx);

    // ожидание вызова completed_task_id_cv.notify_all() в run()
    completed_task_id_cv.wait(
        completed_task_lock, 
        [this, task_id]()->bool { 
            return completed_task_id.find(task_id) != completed_task_id.end(); 
        });
}

void ThreadPool::wait_all()
{
    std::unique_lock<std::mutex> queue_lock(queue_mtx);


    completed_task_id_cv.wait(
        queue_lock, 
        [this]()->bool {
            std::lock_guard<std::mutex> completed_task_lock(completed_task_id_mtx);
            return task_queue.empty() && next_id == completed_task_id.size();
        }
    );
}

bool ThreadPool::is_completed(int64_t task_id)
{
    std::unique_lock<std::mutex> completed_task_lock(completed_task_id_mtx);
    if(completed_task_id.end() != completed_task_id.find(task_id)) return true;
    return false;
}

void ThreadPool::terminate()
{
    end_work = true;
    for(auto& thread : thread_pool)
    {
        queue_cv.notify_all(); // trigger queue_cv.wait(...) in ThreadPool::run()
        thread.join();
    }
}

ThreadPool::~ThreadPool()
{
    wait_all();
    end_work = true;
    for(auto& thread : thread_pool)
    {
        queue_cv.notify_all(); // trigger queue_cv.wait(...) in ThreadPool::run()
        thread.join();
    }
}
