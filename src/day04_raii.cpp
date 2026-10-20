#include <iostream>
#include <memory>

class Trajectory {
public:
    Trajectory() {
        std::cout << "轨迹对象创建\n";
    }

    ~Trajectory() {
        std::cout << "轨迹对象销毁\n";
    }

    void print() const {
        std::cout << "正在使用轨迹\n";
    }
};

int main() {
    std::cout << "程序开始\n";

    {
        auto trajectory = std::make_unique<Trajectory>();

        trajectory->print();

        std::cout << "即将离开作用域\n";
    }

    std::cout << "程序结束\n";

    return 0;
}
