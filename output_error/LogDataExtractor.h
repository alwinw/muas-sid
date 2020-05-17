#pragma once

#include <string>
#include <vector>
#include <array>
#include <exception>
#include <fstream>
#include <set>


template <unsigned int N> // Number of columns of data
class LogDataExtractor
{
public:
    LogDataExtractor()
    {
        static_assert(N > 0);
    }

    const std::vector<std::array<double, N>> &get_data() const { return _data; }
    

protected:
    void extract_data(const std::string &file_name, const double &start_time, const double &duration, const std::set<unsigned int> &data_columns)
    {
        std::ifstream file;
        file.open(file_name);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file");
        }

        // skip the headers
        std::string line;
        std::getline(file, line);

        // iterate over the data
        unsigned int col_idx = 0, time_idx = 0, data_idx = 0;
        bool is_first_line = true;
        double t0;
        while (std::getline(file, line, ',')) {
            // get the initial timestamp to shift the time
            if (is_first_line) {
                t0 = std::stod(line);
                is_first_line = false;
            }

            // only extract data in the specified period of time
            if (col_idx == 0) {
                const double t = std::stod(line);
                if (t - t0 < start_time) {
                    // skip remaining data in this row
                    std::getline(file, line);
                    col_idx = 0;
                    data_idx = 0;
                    continue;
                } else if (t - t0 - start_time > duration) {
                    // no point continuing
                    file.close();
                    return;
                } else {
                    // since this is an acceptable row of data, allocate memory
                    _data.push_back(std::array<double, N>());
                }
            }

            // only extract data for the columns specified by data_columns
            if (data_columns.find(col_idx) == data_columns.end()) {
                if (col_idx > *(data_columns.end())) {
                    // no point continuing to read this row of data since we have already past the largest data column index
                    std::getline(file, line);
                    col_idx = 0;
                    data_idx = 0;
                } else {
                    col_idx++;
                }
                continue;
            }

            double val = std::stod(line);
            _data[time_idx][data_idx] = val;
            col_idx++;
            data_idx++;
            if (data_idx == N) {
                col_idx = 0;
                data_idx = 0;
                time_idx++;
            }
        }
        file.close();
    }

    std::vector<std::array<double, N>> _data;
};


class LogDataExtractor_IMU : public LogDataExtractor<7>
{
public:
    LogDataExtractor_IMU(const std::string &file_name, const double &start_time, const double &duration) : 
        LogDataExtractor<7>()
    {
        extract_data(file_name, start_time, duration, _data_columns);
    }

private:
    const std::set<unsigned int> _data_columns = {0, 2, 3, 4, 5, 6, 7};
};


class LogDataExtractor_ARSP : public LogDataExtractor<2>
{
public:
    LogDataExtractor_ARSP(const std::string &file_name, const double &start_time, const double &duration) : 
        LogDataExtractor<2>()
    {
        extract_data(file_name, start_time, duration, _data_columns);
    }

private:
    const std::set<unsigned int> _data_columns = {0, 2};
};