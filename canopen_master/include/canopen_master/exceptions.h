#ifndef H_IPA_EXCEPTIONS
#define H_IPA_EXCEPTIONS

#include <exception>
#include <boost/exception/all.hpp>
#include <boost/format.hpp>

namespace ipa_canopen{

class Exception : public std::exception {};

class PointerInvalid : public Exception{};
class TimeoutException : public Exception{};
class ParseException : public Exception{};

} // ipa_canopen

#endif // !H_IPA_EXCEPTIONS
