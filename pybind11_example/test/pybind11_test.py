
import sys
import os
print(sys.path)
lib_path = os.path.join(os.path.dirname(__file__), "..", "..", "install/pybind11_example/lib")
print(f"lib_path is {lib_path}")
sys.path.append(lib_path)

import pybind11_example.pybind_cpp_data_processor as pybind_cpp_data_processor

input_data = list(range(10))

cpp_data_processor = pybind_cpp_data_processor.DataProcessor()
cpp_data_processor.setData(input_data)
print("C++ data processor output: {}".format(cpp_data_processor.process()))


import pybind11_example.pybind_simple_function as pybind_simple_function

# 调用函数
result = pybind_simple_function.add(1, 2)
print(f"1 + 2 = {result}")