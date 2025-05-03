#include "pybind11_example/data_processor.hpp"

#include <numeric>

void DataProcessor::setData(std::vector<int> input_data)
{
  data_ = std::move(input_data);
}

int DataProcessor::process()
{
  return std::accumulate(data_.begin(), data_.end(), 0);
}
