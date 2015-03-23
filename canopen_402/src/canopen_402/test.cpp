#include <canopen_402/canopen_402.h>

using namespace can;
using namespace canopen;

using canopen::Node_402;

int main()
{
  std::string name = "1234";
  boost::shared_ptr<Node_402> motor( new Node_402(name));
  motor->init();

  motor->enterModeAndWait(OperationMode(7), true);
  while(true)
  {
    motor->read();
    sleep(1);
    motor->write();
    sleep(1);
  }

  return 0;
}

