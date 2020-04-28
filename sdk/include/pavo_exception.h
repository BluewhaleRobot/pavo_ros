#pragma once
#include <exception>
#include <string>
#include <sstream>

#define PAVO_EXCEPTION_BAD_IP_FORMAT 0x1001
#define PAVO_EXCEPTION_IP_NO_MATCH   0x1002
#define PAVO_EXCEPTION_FAIL_CREATE_RECVSOCKET        0x1003
#define PAVO_EXCEPTION_BOOST         0x3009
#define PAVO_EXCEPTION_SYSTEM        0x4009
#define PAVO_EXCEPTION_UNKNOWN       0x6009


class pavo_exception : public std::exception
{
public:
	pavo_exception(int err_code, std::string& cause)
		:err_code_(err_code)
	{
		std::ostringstream oss;
		if (err_code_ == PAVO_EXCEPTION_BAD_IP_FORMAT)
			oss << "Format of the IP [" << cause << "] is in correct!";
		else if (err_code_ == PAVO_EXCEPTION_IP_NO_MATCH)
			oss << "The address [" << cause << "] is could not be binded!";
		else if (err_code_ == PAVO_EXCEPTION_FAIL_CREATE_RECVSOCKET)
			oss << "Fail to create The receive socket [" << cause << "],the scoket may be used!";
		else if(err_code_ == PAVO_EXCEPTION_UNKNOWN)
		{
			oss << "Unknown error!";
		}
		else
		{
			oss << cause << ":" << std::exception::what();
		}
		what_ = oss.str();
	}

	int error_code(void)
	{
		return err_code_;
	}

	virtual const char* what(void) const noexcept override
	{
		return what_.c_str();
	}



private:
	int err_code_;
	std::string what_;
};
