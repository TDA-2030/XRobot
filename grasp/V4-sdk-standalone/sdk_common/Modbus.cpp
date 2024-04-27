#include "Modbus.h"
#include "Exception.h"

#include <map>
#include <mutex>
#include <thread>

extern "C"
{
#include "libmodbus/modbus.h"
}

std::map<std::string, std::mutex> modbus_device_lock;

using namespace std::chrono_literals;

#define SWITCH_GAP_TIME 10ms

Modbus::Modbus(std::string device_name, void* ctx) :
	ctx((modbus_t*)ctx), 
	device(device_name),
	timeout_ms(100), 
	max_retry(10)
{
	set_slave(1);
	modbus_set_response_timeout((modbus_t*)ctx, this->timeout_ms/1000, (this->timeout_ms%1000) * 1000);
}

Modbus::~Modbus()
{

}

void Modbus::set_slave(int slave)
{
	if (modbus_get_slave((modbus_t*)ctx) != slave)
	{
		modbus_set_slave((modbus_t*)ctx, slave);
		std::this_thread::sleep_for(SWITCH_GAP_TIME);
	}
	
}

void Modbus::set_timing(uint32_t timeout_ms, uint32_t max_retry)
{
	modbus_set_response_timeout((modbus_t*)ctx, this->timeout_ms / 1000, (this->timeout_ms % 1000) * 1000);
	this->max_retry = max_retry;
}

std::vector<uint8_t> Modbus::read_inputs(int slave, uint16_t address, uint16_t quantity)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	std::vector<uint8_t> buffer(quantity);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_read_input_bits((modbus_t*)ctx, address, quantity, (uint8_t*)&buffer[0]) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d read inputs address %d quantity %d failed %s",
			device.c_str(), slave, address, quantity, modbus_strerror(errno));

	return buffer;
}

std::vector<uint8_t> Modbus::read_coils(int slave, uint16_t address, uint16_t quantity)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	std::vector<uint8_t> buffer(quantity);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_read_bits((modbus_t*)ctx, address, quantity, (uint8_t*)&buffer[0]) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d read coils address %d quantity %d failed %s",
			device.c_str(), slave, address, quantity, modbus_strerror(errno));

	return buffer;
}

void Modbus::write_coils(int slave, uint16_t address, std::vector<uint8_t> data)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_write_bits((modbus_t*)ctx, address, (int)data.size(), &data[0]) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d write coils address %d quantity %d failed %s",
			device.c_str(), slave, address, data.size(), modbus_strerror(errno));
}

void Modbus::write_coil(int slave, uint16_t address, uint8_t data)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_write_bit((modbus_t*)ctx, address, (int)data) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d write coil address %d failed %s",
			device.c_str(), slave, address, modbus_strerror(errno));
}


std::vector<uint16_t> Modbus::read_input_registers(int slave, uint16_t address, uint16_t quantity)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	std::vector<uint16_t> buffer;
	buffer.resize(quantity);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_read_input_registers((modbus_t*)ctx, address, quantity, &buffer[0]) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d read input registers address %d quantity %d failed %s",
			device.c_str(), slave, address, quantity, modbus_strerror(errno));

	return buffer;
}

std::vector<uint16_t> Modbus::read_holding_registers(int slave, uint16_t address, uint16_t quantity)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	std::vector<uint16_t> buffer;
	buffer.resize(quantity);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_read_registers((modbus_t*)ctx, address, quantity, &buffer[0]) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d read holding registers address %d quantity %d failed %s",
			device.c_str(), slave, address, quantity, modbus_strerror(errno));

	return buffer;
}

void Modbus::write_registers(int slave, uint16_t address, std::vector<uint16_t> data)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_write_registers((modbus_t*)ctx, address, (int)data.size(), &data[0]) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d write coils address %d quantity %d failed %s",
			device.c_str(), slave, address, data.size(), modbus_strerror(errno));
}

void Modbus::write_register(int slave, uint16_t address, uint16_t data)
{
	std::lock_guard<std::mutex> lock_guard(modbus_device_lock[device]);

	set_slave(slave);

	uint32_t retry_count = 0;
	bool succeed = false;

	do
	{
		succeed = modbus_write_register((modbus_t*)ctx, address, data) != -1;
		if (succeed) break;
		retry_count++;
	} while (retry_count < this->max_retry);

	if (!succeed)
		NewException("Modbus %s:%d write register address %d failed %s",
			device.c_str(), slave, address, modbus_strerror(errno));
}
