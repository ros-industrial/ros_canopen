#ifndef H_EXCEPTIONS
#define H_EXCEPTIONS

#include <exception>
#include <boost/exception/all.hpp>
#include <boost/format.hpp>

namespace canopen{

class Exception : public std::runtime_error {
public:
    Exception(const std::string &w) : std::runtime_error(w) {}
};

class PointerInvalid : public Exception{
public:
    PointerInvalid(const std::string &w) : Exception("Pointer invalid") {}
};

class ParseException : public Exception{
public:
    ParseException(const std::string &w) : Exception(w) {}
};

class TimeoutException : public Exception{
public:
    TimeoutException(const std::string &w) : Exception(w) {}
};


} // canopen

#endif // !H_EXCEPTIONS
