
#include <iostream>
#include "command.hpp"
#include <gecko2.hpp>
#include <string>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>

using namespace std;

static volatile sig_atomic_t timeout = 0;

void handler(int sig) {
    (void) sig;
    timeout = 1;
}

// int main() {
//   // while (true) {
//   //   cout << "navigator" << endl;
//   //   gecko::sleep(1);
//   // }
//   struct sigaction act;
//   memset(&act, 0, sizeof(act));
//   act.sa_handler = handler;
//   if(sigaction(SIGALRM, &act, NULL) < 0) {
//       // handle error
//   }
//   alarm(5); // seconds
//   int cnt = 0;
//   while(!timeout){
//       // do something, handle error return codes and errno (EINTR)
//       // check terminate flag as necessary
//       sleep(1);
//       cout << "ping: " << ++cnt << endl;
//   }
// }

// #include <cstdio>
// #include <iostream>
// #include <memory>
// #include <stdexcept>
// #include <string>
// #include <array>

// std::string exec(const char* cmd) {
//     std::array<char, 128> buffer;
//     std::string result;
//     std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
//     if (!pipe) {
//         throw std::runtime_error("popen() failed!");
//     }
//     while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
//         result += buffer.data();
//     }
//     return result;
// }

int main() {
  string ser;
  bool ok = exec("ls /dev/tty.usbmodem*", ser);
  if (!ok) cout << "crap" << endl;
  cout << ser << endl;
}