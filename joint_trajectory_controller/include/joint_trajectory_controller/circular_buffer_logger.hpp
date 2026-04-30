#include <algorithm>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

inline std::string make_timestamped_filename(
  const std::string & prefix, const std::string & extension)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm tm{};

#if defined(_WIN32)
  localtime_s(&tm, &now_time);
#else
  localtime_r(&now_time, &tm);
#endif

  std::ostringstream oss;
  oss << prefix << std::put_time(&tm, "%y-%m-%d-%H-%M-%S") << extension;

  return oss.str();
}

class CircularVectorLogBuffer
{
public:
  CircularVectorLogBuffer(std::size_t capacity, std::size_t sample_dim, std::string filename)
  : filename_(std::move(filename)), capacity_(capacity), sample_dim_(sample_dim)
  {
    if (capacity == 0 || sample_dim == 0)
    {
      throw std::invalid_argument("capacity and sample_dim must be > 0");
    }

    // Reserve storage for exactly `capacity` samples, each with `sample_dim` values.
    buffer_.resize(capacity_ * sample_dim_);
  }

  ~CircularVectorLogBuffer() noexcept
  {
    try
    {
      flush();
    }
    catch (...)
    {
      // Never throw from a destructor.
    }
  }

  void add(const double * sample, std::size_t n) noexcept
  {
    if (n != sample_dim_)
    {
      return;
    }

    double * dst = &buffer_[write_index_ * sample_dim_];

    std::copy(sample, sample + sample_dim_, dst);

    write_index_ = (write_index_ + 1) % capacity_;

    if (size_ < capacity_)
    {
      ++size_;
    }
    else
    {
      full_ = true;
    }
  }

  void flush()
  {
    std::ofstream file(filename_);

    if (!file)
    {
      throw std::runtime_error("failed to open file: " + filename_);
    }

    for (std::size_t i = 0; i < size_; ++i)
    {
      const double * sample = get_chronological(i);

      for (std::size_t j = 0; j < sample_dim_; ++j)
      {
        file << sample[j];

        if (j + 1 < sample_dim_)
        {
          file << ",";
        }
      }

      file << "\n";
    }
  }

private:
  const double * get_chronological(std::size_t i) const noexcept
  {
    std::size_t start = full_ ? write_index_ : 0;
    std::size_t sample_index = (start + i) % capacity_;
    return &buffer_[sample_index * sample_dim_];
  }

  std::vector<double> buffer_;

  std::string filename_;

  std::size_t capacity_;
  std::size_t sample_dim_;

  std::size_t write_index_ = 0;
  std::size_t size_ = 0;
  bool full_ = false;
};