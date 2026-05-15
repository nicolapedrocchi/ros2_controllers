#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iostream>
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
  : filename_(std::move(filename)), capacity_(capacity), sample_dim_and_time_(sample_dim+1), start_(std::chrono::system_clock::now())
  {
    if (capacity == 0 || sample_dim == 0)
    {
      throw std::invalid_argument("capacity and sample_dim must be > 0");
    }

    // Reserve storage for exactly `capacity` samples, each with `sample_dim` values.
    buffer_.resize(capacity_ * sample_dim_and_time_);
  }

  ~CircularVectorLogBuffer() noexcept
  {
    std::cerr << "[CircularVectorLogBuffer] destructor called for: " << filename_ << std::endl;
    try
    {
      flush();
    }
    catch (const std::exception & e)
    {
      std::cerr << "[CircularVectorLogBuffer] flush() threw in destructor: " << e.what() << std::endl;
    }
    catch (...)
    {
      std::cerr << "[CircularVectorLogBuffer] flush() threw unknown exception in destructor" << std::endl;
    }
  }

  void add(const double * sample, std::size_t n) noexcept
  {
    if (n != sample_dim_and_time_-1)
    {
      return;
    }

    auto now = std::chrono::system_clock::now();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_).count();

    double * dst = &buffer_[write_index_ * sample_dim_and_time_];
    dst[0] = static_cast<double>(millis);
    std::copy(sample, sample + (sample_dim_and_time_-1), dst + 1);

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
    if (flushed_)
    {
      std::cerr << "[CircularVectorLogBuffer] flush() skipped (already flushed): " << filename_ << std::endl;
      return;
    }

    std::cerr << "[CircularVectorLogBuffer] flush() entered for: " << filename_
              << "  (size=" << size_ << ", capacity=" << capacity_ << ")" << std::endl;

    std::ofstream file(filename_);

    if (!file)
    {
      std::cerr << "[CircularVectorLogBuffer] ERROR: failed to open file: " << filename_ << std::endl;
      throw std::runtime_error("failed to open file: " + filename_);
    }

    std::cerr << "[CircularVectorLogBuffer] file opened successfully, writing " << size_ << " rows..." << std::endl;

    file << std::fixed << std::setprecision(6);

    for (std::size_t i = 0; i < size_; ++i)
    {
      const double * sample = get_chronological(i);

      for (std::size_t j = 0; j < sample_dim_and_time_; ++j)
      {
        if (j == 0)
        {
          // timestamp column: integer milliseconds, no decimal needed
          file << std::fixed << std::setprecision(0) << sample[j];
        }
        else
        {
          file << std::fixed << std::setprecision(9) << sample[j];
        }

        if (j + 1 < sample_dim_and_time_)
        {
          file << ",";
        }
      }

      file << "\n";
    }

    file.flush();
    flushed_ = true;
    std::cerr << "[CircularVectorLogBuffer] flush() completed for: " << filename_ << std::endl;
  }

private:
  const double * get_chronological(std::size_t i) const noexcept
  {
    std::size_t start = full_ ? write_index_ : 0;
    std::size_t sample_index = (start + i) % capacity_;
    return &buffer_[sample_index * sample_dim_and_time_];
  }

  std::vector<double> buffer_;

  std::string filename_;

  std::size_t capacity_;
  std::size_t sample_dim_and_time_;

  std::size_t write_index_ = 0;
  std::size_t size_ = 0;
  bool full_ = false;
  bool flushed_ = false;

  std::chrono::system_clock::time_point start_;
};