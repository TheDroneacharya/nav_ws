#ifndef WALL_DISTANCE_CONTROLLER__MOVING_AVERAGE_FILTER_HPP_
#define WALL_DISTANCE_CONTROLLER__MOVING_AVERAGE_FILTER_HPP_

#include <vector>
#include <numeric>
#include <cmath>
#include <limits>
#include <algorithm>

namespace wall_distance_controller
{

/**
 * @brief Moving Average Filter with configurable window size
 * 
 * Uses a circular buffer to compute rolling average of depth measurements.
 * Supports outlier rejection and weighted averaging.
 */
class MovingAverageFilter
{
public:
    /**
     * @brief Construct a new Moving Average Filter
     * 
     * @param window_size Number of samples to average over
     * @param reject_outliers If true, reject values outside 2 std deviations
     */
    explicit MovingAverageFilter(size_t window_size = 10, bool reject_outliers = true)
        : window_size_(window_size)
        , reject_outliers_(reject_outliers)
        , buffer_(window_size, std::numeric_limits<double>::quiet_NaN())
        , head_(0)
        , count_(0)
        , last_valid_value_(0.0)
    {
    }

    /**
     * @brief Add a new sample to the filter
     * 
     * @param value New depth measurement value
     */
    void addSample(double value)
    {
        if (std::isnan(value) || std::isinf(value)) {
            return;  // Ignore invalid values
        }

        // Store the value in circular buffer
        buffer_[head_] = value;
        head_ = (head_ + 1) % window_size_;
        
        if (count_ < window_size_) {
            count_++;
        }
    }

    /**
     * @brief Get the current filtered (averaged) value
     * 
     * @return double Filtered depth value
     */
    double getFilteredValue() const
    {
        if (count_ == 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        std::vector<double> valid_values;
        valid_values.reserve(count_);

        // Collect valid values from buffer
        for (size_t i = 0; i < count_; ++i) {
            if (!std::isnan(buffer_[i]) && !std::isinf(buffer_[i])) {
                valid_values.push_back(buffer_[i]);
            }
        }

        if (valid_values.empty()) {
            return last_valid_value_;
        }

        if (reject_outliers_ && valid_values.size() > 3) {
            // Calculate mean and standard deviation
            double sum = std::accumulate(valid_values.begin(), valid_values.end(), 0.0);
            double mean = sum / valid_values.size();
            
            double sq_sum = 0.0;
            for (double val : valid_values) {
                sq_sum += (val - mean) * (val - mean);
            }
            double std_dev = std::sqrt(sq_sum / valid_values.size());

            // Remove outliers (values outside 2 standard deviations)
            std::vector<double> filtered_values;
            for (double val : valid_values) {
                if (std::abs(val - mean) <= 2.0 * std_dev) {
                    filtered_values.push_back(val);
                }
            }

            if (!filtered_values.empty()) {
                double filtered_sum = std::accumulate(filtered_values.begin(), filtered_values.end(), 0.0);
                last_valid_value_ = filtered_sum / filtered_values.size();
            }
        } else {
            // Simple average without outlier rejection
            double sum = std::accumulate(valid_values.begin(), valid_values.end(), 0.0);
            last_valid_value_ = sum / valid_values.size();
        }

        return last_valid_value_;
    }

    /**
     * @brief Get the median value from the buffer (more robust to outliers)
     * 
     * @return double Median depth value
     */
    double getMedianValue() const
    {
        if (count_ == 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        std::vector<double> valid_values;
        for (size_t i = 0; i < count_; ++i) {
            if (!std::isnan(buffer_[i]) && !std::isinf(buffer_[i])) {
                valid_values.push_back(buffer_[i]);
            }
        }

        if (valid_values.empty()) {
            return last_valid_value_;
        }

        std::sort(valid_values.begin(), valid_values.end());
        size_t mid = valid_values.size() / 2;
        
        if (valid_values.size() % 2 == 0) {
            return (valid_values[mid - 1] + valid_values[mid]) / 2.0;
        } else {
            return valid_values[mid];
        }
    }

    /**
     * @brief Get the minimum value from the buffer
     * 
     * @return double Minimum depth value (closest object)
     */
    double getMinValue() const
    {
        double min_val = std::numeric_limits<double>::max();
        bool found = false;
        
        for (size_t i = 0; i < count_; ++i) {
            if (!std::isnan(buffer_[i]) && !std::isinf(buffer_[i])) {
                if (buffer_[i] < min_val && buffer_[i] > 0.01) {  // Ignore very small values (sensor noise)
                    min_val = buffer_[i];
                    found = true;
                }
            }
        }
        
        return found ? min_val : std::numeric_limits<double>::quiet_NaN();
    }

    /**
     * @brief Check if the filter has enough samples for a reliable estimate
     * 
     * @return true if at least half the window is filled
     */
    bool isReady() const
    {
        return count_ >= (window_size_ / 2);
    }

    /**
     * @brief Reset the filter to initial state
     */
    void reset()
    {
        std::fill(buffer_.begin(), buffer_.end(), std::numeric_limits<double>::quiet_NaN());
        head_ = 0;
        count_ = 0;
        last_valid_value_ = 0.0;
    }

    /**
     * @brief Get number of valid samples currently in buffer
     */
    size_t getSampleCount() const
    {
        return count_;
    }

    /**
     * @brief Get the variance of values in the buffer
     * 
     * @return double Variance (useful for determining measurement stability)
     */
    double getVariance() const
    {
        if (count_ < 2) {
            return 0.0;
        }

        std::vector<double> valid_values;
        for (size_t i = 0; i < count_; ++i) {
            if (!std::isnan(buffer_[i]) && !std::isinf(buffer_[i])) {
                valid_values.push_back(buffer_[i]);
            }
        }

        if (valid_values.size() < 2) {
            return 0.0;
        }

        double sum = std::accumulate(valid_values.begin(), valid_values.end(), 0.0);
        double mean = sum / valid_values.size();
        
        double sq_sum = 0.0;
        for (double val : valid_values) {
            sq_sum += (val - mean) * (val - mean);
        }
        
        return sq_sum / (valid_values.size() - 1);
    }

private:
    size_t window_size_;
    bool reject_outliers_;
    std::vector<double> buffer_;
    size_t head_;
    size_t count_;
    mutable double last_valid_value_;
};

}  // namespace wall_distance_controller

#endif  // WALL_DISTANCE_CONTROLLER__MOVING_AVERAGE_FILTER_HPP_
