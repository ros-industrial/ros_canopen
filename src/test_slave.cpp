#include <lely/ev/loop.hpp>

#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>

#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>

using namespace lely;

int
main() {
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl("vcan0");
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    canopen::BasicSlave slave(timer, chan, "/home/christoph/ws_ros2/src/ros2_canopen/ressources/simple.eds", "", 2);
    io::SignalSet sigset(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    // Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/) {
    // If the signal is raised again, terminate immediately.
    sigset.clear();
    // Perform a clean shutdown.
    ctx.shutdown();
  });
  slave.Reset();
  loop.run();

  return 0;
}