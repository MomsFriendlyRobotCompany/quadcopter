
#include <iostream>
// #include <serialcomm.hpp>
#include <gecko2.hpp>

#include <signal.h>
#include <unistd.h>
#include <stdlib.h>

using namespace std;

static volatile sig_atomic_t timeout = 0;

void handler(int sig) {
    (void) sig;
    timeout = 1;
}

int main() {
  // while (true) {
  //   cout << "navigator" << endl;
  //   gecko::sleep(1);
  // }
  struct sigaction act;
  memset(&act, 0, sizeof(act));
  act.sa_handler = handler;
  if(sigaction(SIGALRM, &act, NULL) < 0) {
      // handle error
  }
  alarm(5); // seconds
  int cnt = 0;
  while(!timeout){
      // do something, handle error return codes and errno (EINTR)
      // check terminate flag as necessary
      sleep(1);
      cout << "ping: " << ++cnt << endl;
  }
}