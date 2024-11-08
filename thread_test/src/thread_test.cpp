#include <iostream>
#include <thread>
#include <chrono>

// 定义一个函数，该函数将在新线程中运行
void print_numbers(int start, int end) {
    std::cout << "Son thread is running..." << std::endl;
    for (int i = start; i <= end; ++i) {
        std::cout << "Son Number: " << i << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 模拟延迟
    }
    std::cout << "Son thread is finishing..." << std::endl;
}

int main() {
    // 创建一个线程，调用 print_numbers 函数
    std::thread t(print_numbers, 1, 10);

    // 让新线程独立运行
    t.detach();
    // std::thread{print_numbers, 1, 10}.detach();

    // 打印一些信息，表示主线程仍在运行
    std::cout << "Main thread is running..." << std::endl;
    size_t count = 10;
    for (size_t i = 0; i < count; i++)
    {
        std::cout << "Main count is "<<  i << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 模拟主线程的其他任务
    std::cout << "Main thread is finishing..." << std::endl;

    return 0;
}