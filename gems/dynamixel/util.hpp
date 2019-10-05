#include <sstream>

template <typename T>
    std::string NumberToString ( T Number ) {
        std::ostringstream ss;
        ss << Number;
        return ss.str();
    }

template <typename T>
    T clamp(const T& value, const T& low, const T& high) {
        return std::max(std::min(value, high), low);
    }
