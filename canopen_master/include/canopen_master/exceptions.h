#ifndef H_EXCEPTIONS
#define H_EXCEPTIONS

#include <exception>
#include <boost/exception/all.hpp>
#include <boost/format.hpp>

namespace canopen{

class Exception : public std::exception {};

class PointerInvalid : public Exception{};
class TimeoutException : public Exception{};
class ParseException : public Exception{};

} // canopen

#endif // !H_EXCEPTIONS
