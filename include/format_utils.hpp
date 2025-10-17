#include <Eigen/Eigen>
#include <format>
#include <iostream>
#include <string>

template <typename EigenExprTypeT>
concept EigenTypeMatExpr = requires(const EigenExprTypeT t) {
  std::remove_cvref_t<EigenExprTypeT>::RowsAtCompileTime;
  std::remove_cvref_t<EigenExprTypeT>::ColsAtCompileTime;
  typename std::remove_cvref_t<EigenExprTypeT>::Scalar;
  { t.size() } -> std::same_as<typename Eigen::Index>;
  { t.rows() } -> std::same_as<typename Eigen::Index>;
  { t.cols() } -> std::same_as<typename Eigen::Index>;
};

enum class EigenCustomFormats {
  Default,              //
  CleanFormat,          // cf
  HeavyFormat,          // hf
  SingleLineFormat,     // sfl
  HighPrecisionFormat,  // hpf
  DebuggingFormat       // df
};

static const auto defaultFormat = Eigen::IOFormat();
static const auto cleanFormat = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");
static const auto heavyFormat =
    Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
static const auto singleLineFormat = Eigen::IOFormat(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
static const auto highPrecisionFormat = Eigen::IOFormat(
    Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");
static const auto debuggingFormat = Eigen::IOFormat(
    Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "\n", "");

template <EigenTypeMatExpr MatT>
struct std::formatter<MatT> {
  constexpr auto parse(format_parse_context& ctx) {
    const std::string_view fmt(ctx.begin(), ctx.end());
    if (fmt.starts_with("cf")) _format = EigenCustomFormats::CleanFormat;
    if (fmt.starts_with("hf")) _format = EigenCustomFormats::HeavyFormat;
    if (fmt.starts_with("sfl")) _format = EigenCustomFormats::SingleLineFormat;
    if (fmt.starts_with("hpf"))
      _format = EigenCustomFormats::HighPrecisionFormat;
    if (fmt.starts_with("df")) _format = EigenCustomFormats::DebuggingFormat;
    return ctx.begin() + fmt.find_first_of('}');
  }

  // Format the type for output
  template <typename FormatContext>
  auto format(const MatT& m, FormatContext& ctx) const {
    switch (_format) {
      case EigenCustomFormats::CleanFormat:
        return std::format_to(
            ctx.out(), "{}",
            (std::stringstream{} << std::fixed << m.format(cleanFormat)).str());
      case EigenCustomFormats::HeavyFormat:
        return std::format_to(
            ctx.out(), "{}",
            (std::stringstream{} << std::fixed << m.format(heavyFormat)).str());
      case EigenCustomFormats::SingleLineFormat:
        return std::format_to(
            ctx.out(), "{}",
            (std::stringstream{} << std::fixed << m.format(singleLineFormat))
                .str());
      case EigenCustomFormats::HighPrecisionFormat:
        return std::format_to(
            ctx.out(), "{}",
            (std::stringstream{} << std::fixed << m.format(highPrecisionFormat))
                .str());
      case EigenCustomFormats::DebuggingFormat:
        return std::format_to(
            ctx.out(), "{}",
            (std::stringstream{} << std::fixed << m.format(debuggingFormat))
                .str());
      default:
        return std::format_to(
            ctx.out(), "{}",
            (std::stringstream{} << std::fixed << m.format(defaultFormat))
                .str());
    }
  }

 private:
  EigenCustomFormats _format{EigenCustomFormats::Default};
};

template <EigenTypeMatExpr MatT>
std::ostream& operator<<(std::ostream& os, const MatT& mat) {
  return os << std::format("{:hf}", mat);
}

// Provide a small compatibility formatter for __int128 on some libstdc++
// versions where std::format's selection can be ambiguous. This casts to
// 64-bit for formatting purposes (the library consistently uses 64-bit
// timestamps elsewhere). Guarded to only compile with GNU's libstdc++.
#if (defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE < 15) || (defined(__GNUC__) && __GNUC__ < 15)
namespace std {
// signed __int128
template <typename CharT>
struct formatter<__int128, CharT> : formatter<long long, CharT> {
  template <typename FormatContext>
  auto format(__int128 v, FormatContext& ctx) {
    return formatter<long long, CharT>::format(static_cast<long long>(v), ctx);
  }
};

// unsigned __int128
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