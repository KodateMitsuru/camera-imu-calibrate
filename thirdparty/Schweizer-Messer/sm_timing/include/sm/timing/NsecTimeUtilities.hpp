/**
 * @file   NsecTimeUtilities.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Sat Jul 20 12:39:54 2013
 *
 * @brief  Functions to support the use of nanosecond epoch time.
 *
 */

#include <chrono>
#include <cstdint>

#if (defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE < 15) || \
    (defined(__GNUC__) && __GNUC__ < 15)
#include <format>
namespace std {
// signed __int128
template <typename CharT>
struct formatter<__int128, CharT> : formatter<long long, CharT> {
  template <typename FormatContext>
  auto format(__int128 v, FormatContext& ctx) {
    return formatter<long long, CharT>::format(static_cast<long long>(v), ctx);
  }
};

template <typename CharT>
struct formatter<unsigned __int128, CharT>
    : formatter<unsigned long long, CharT> {
  template <typename FormatContext>
  auto format(unsigned __int128 v, FormatContext& ctx) {
    return formatter<unsigned long long, CharT>::format(
        static_cast<unsigned long long>(v), ctx);
  }
};
}  // namespace std
#endif

namespace sm {
namespace timing {

/// \brief Nanoseconds since the epoch.
typedef int64_t NsecTime;

/// \brief Convert nanoseconds since the epoch to std::chrono
std::chrono::system_clock::time_point nsecToChrono(const NsecTime& time);

/// \brief Convert std::chrono to nanoseconds since the epoch.
NsecTime chronoToNsec(const std::chrono::system_clock::time_point& time);

/// \brief Get the epoch time as nanoseconds since the epoch.
NsecTime nsecNow();

/// \brief Convert the time (in integer nanoseconds) to decimal seconds.
double nsecToSec(const NsecTime& time);

/// \brief Convert the time (in seconds) to integer nanoseconds
NsecTime secToNsec(const double& time);

}  // namespace timing
}  // namespace sm
