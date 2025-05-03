#include <random>
#include <vector>

class DataProcessor
{
public:
  DataProcessor() = default;
  void setData(std::vector<int> input_data);
  int process();

private:
  std::vector<int> data_;
};
