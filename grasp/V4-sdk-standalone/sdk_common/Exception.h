#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_

#include <stdint.h>
#include <string>
#include <string.h>
#include <stdarg.h>
#include <exception>
#include <stdexcept>

inline void New_Exception(int line_number, std::string src_file, std::string format, ...)
{
	std::string message;
	va_list args, args_copy;
	va_start(args, format);
	va_copy(args_copy, args);
	const auto size = std::vsnprintf(nullptr, 0, format.c_str(), args) + 1;
	message.resize(size, ' ');
	std::vsnprintf(&message.front(), size, format.c_str(), args_copy);
	va_end(args_copy);
	va_end(args);

	throw std::runtime_error(message.c_str());
}

#define NewException(...) New_Exception(__LINE__, __FILE__, __VA_ARGS__)

#endif