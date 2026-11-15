// ======================================================================
// \title  Main.cpp
// \brief  PadMcu SIL 主程式（Linux，發射架板）。用法：./PadMcu [-a GDS位址 -p 埠]
//         Ctrl-C／SIGTERM 結束。ESP32 版的 app_main 跟 MCU 組 ESP32 平台 PR 一起。
// ======================================================================
#include "Deployments/PadMcu/Top/PadMcuTopology.hpp"

#include <Fw/Logger/Logger.hpp>
#include <Os/Os.hpp>
#include <getopt.h>
#include <signal.h>
#include <cstdlib>

static void print_usage(const char* app) {
  Fw::Logger::log("Usage: ./%s [options]\n-a\thostname/IP address\n-p\tport_number\n", app);
}

static void signalHandler(int) { PadMcu::stopRateGroups(); }

int main(int argc, char* argv[]) {
  I32 option = 0;
  CHAR* hostname = nullptr;
  U16 port_number = 0;

  Os::init();
  while ((option = getopt(argc, argv, "hp:a:")) != -1) {
    switch (option) {
      case 'a':
        hostname = optarg;
        break;
      case 'p':
        port_number = static_cast<U16>(atoi(optarg));
        break;
      case 'h':
      case '?':
      default:
        print_usage(argv[0]);
        return (option == 'h') ? 0 : 1;
    }
  }
  PadMcu::TopologyState inputs;
  inputs.hostname = hostname;
  inputs.port = port_number;

  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  Fw::Logger::log("PadMcu SIL — Ctrl-C to quit\n");

  PadMcu::setupTopology(inputs);
  PadMcu::startRateGroups();
  PadMcu::teardownTopology(inputs);
  Fw::Logger::log("Exiting...\n");
  return 0;
}
