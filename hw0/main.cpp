#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    float theta = 45.0 / 180.0 * std::acos(-1);
    Eigen::Vector3f P(2.0f, 1.0f, 1.0f);
    Eigen::Matrix3f R, T;
    R << std::cos(theta), -std::sin(theta), 0.0, std::sin(theta), std::cos(theta), 0.0, 0.0, 0.0, 1.0;
    T << 1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0;
    std::cout << T * R * P << std::endl;
    return 0;
}