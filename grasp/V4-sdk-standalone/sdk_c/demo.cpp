#include "../sdk_common/RMAxis.h"
#include <thread>
#include <chrono>
#include <iostream>


int main()
{
	printf("test");

	auto axis = RMAxis::create_rmaxis_modbus_rtu("/dev/tty17", 115200, 0);

	auto version = axis->get_version();

	std::cout << version.build << std::endl;

	return 0;
}