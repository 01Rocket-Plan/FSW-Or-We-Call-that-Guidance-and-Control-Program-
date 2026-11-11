// ======================================================================
// \title  Main.cpp
// \brief  UsMcu SIL 主程式（Linux）。用法：./UsMcu [-a GDS位址 -p 埠]
//         Ctrl-C／SIGTERM 結束。ESP32 版的 app_main 在下一個 PR。
// ======================================================================
#include "Deployments/UsMcu/Top/UsMcuTopology.hpp"

#include <Fw/Logger/Logger.hpp>
#include <Os/Os.hpp>
#include <getopt.h>
#include <signal.h>
#include <cstdlib>

static void print_usage(const char* app) {
  Fw::Logger::log("Usage: ./%s [options]\n-a\thostname/IP address\n-p\tport_number\n", app);
}

static void signalHandler(int) { UsMcu::stopRateGroups(); }

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
  UsMcu::TopologyState inputs;
  inputs.hostname = hostname;
  inputs.port = port_number;

  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  Fw::Logger::log("UsMcu SIL — Ctrl-C to quit\n");

  UsMcu::setupTopology(inputs);
  UsMcu::startRateGroups();
  UsMcu::teardownTopology(inputs);
  Fw::Logger::log("Exiting...\n");
  return 0;
}
