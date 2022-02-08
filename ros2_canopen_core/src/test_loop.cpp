#include <thread>
#include <iostream>
#include <thread>
#include <mutex>
#include "lely/ev/loop.hpp"
#include "lely/ev/co_task.hpp"

using namespace lely::ev;
using namespace std::chrono_literals;

#define NUM_OP (16 * 1024 * 1024)

class MyOp : public CoTask
{
public:
    virtual void
    operator()() noexcept override
    {
        co_reenter(*this)
        {
            for (; n < NUM_OP; n++)
            {
                std::cout << "World" << std::endl;
                std::this_thread::sleep_for(100ms);
                co_yield get_executor().post((ev_task &)(*this));
            }
        }
    }

private:
    ::std::size_t n{0};
};

int main()
{

    Loop loop;
    std::shared_ptr<std::mutex> mtx = std::make_shared<std::mutex>();
    MyOp op;
    loop.get_executor().post(op);
    mtx->lock();

    auto func = [&loop]{
        std::this_thread::sleep_for(100ms);
        loop.get_executor().post(
            []{
                while (true)
                {
                    std::this_thread::sleep_for(100ms);
                    std::cout << "Hello" << std::endl;
                }
            }
        );
        };


    auto t = std::thread(func);

    auto t1 = ::std::chrono::high_resolution_clock::now();
    auto nop = loop.run();
    auto t2 = ::std::chrono::high_resolution_clock::now();

    auto ns = ::std::chrono::nanoseconds(t2 - t1).count();

    return 0;
}
