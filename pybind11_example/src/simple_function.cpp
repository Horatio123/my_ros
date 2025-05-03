#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// 定义一个简单的求和函数
int add(int a, int b) {
    return a + b;
}

// 使用 PYBIND11_MODULE 宏暴露函数
PYBIND11_MODULE(pybind_simple_function, m) {
    m.doc() = "pybind11 simple function example"; 
    m.def("add", &add, "A function that adds two numbers"); 
}