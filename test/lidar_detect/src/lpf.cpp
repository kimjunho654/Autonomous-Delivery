#include <iostream>
#include "matplotlibcpp.h" 
#include <vector>

namespace plt = matplotlibcpp;

class LowPassFilter {
public:
    LowPassFilter(double alpha) : alpha_(alpha), initialized_(false), prev_output_(0) {}

    double filter(double input) {
        if (!initialized_) {
            prev_output_ = 0;
            initialized_ = true;
        }

        double output = alpha_ * prev_output_ + input * (1 - alpha_);
        prev_output_ = output;

        return output;
    }

private:
    double alpha_;
    bool initialized_;
    double prev_output_;
};

class HighPassFilter {
public:
    HighPassFilter(double alpha) : alpha_(alpha), initialized_(false), prev_input_(0), prev_output_(0) {}

    double filter(double input) {
        if (!initialized_) {
            prev_input_ = 0;
            prev_output_ = 0;
            initialized_ = true;
        }

        double output = alpha_ * prev_input_ + alpha_ * ( input - prev_output_);
        prev_input_ = output;
        prev_output_ = input;

        return output;
    }

private:
    double alpha_;
    bool initialized_;
    double prev_input_;
    double prev_output_;
};

int main() {
    // Low-Pass Filter 객체 생성 (alpha 값은 필터 강도를 조절)
    LowPassFilter lpf(0.9);  // alpha 값은 필터 강도를 조절

    // High-Pass Filter 객체 생성 (alpha 값은 필터 강도를 조절)
    HighPassFilter hpf(0.98);  // alpha 값은 필터 강도를 조절

    // 입력 데이터 생성 (상수 값 0.66)
    double constant_data = 0.66;

    std::vector<double> x_data;
    std::vector<double> y_data;


    for (int i = 0; i < 41; i++) {
        x_data.push_back(i);
    }

    // LPF와 HPF를 적용하여 필터링된 데이터 생성
    for (int i = 0; i < 41; i++) {
        double lpf_filtered_data = lpf.filter(constant_data);
        double hpf_filtered_data = hpf.filter(0.66);
        y_data.push_back(lpf_filtered_data);

    }

    //plt::plot(x_data, y_lpf_data);
    plt::plot(x_data, y_data);
    plt::xlabel("position x");
    plt::ylabel("filtered value");
    plt::title("Low-Pass and High-Pass Filtered Data");

    plt::show();

    return 0;
}


