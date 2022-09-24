#include<cmath>
#include<Eigen/Core>
#include<iostream>

// Problem: 给定一个点 P =(2,1), 将该点绕原点先逆时针旋转 45◦，
// 再平移 (1,2), 计算出 变换后点的坐标(要求用齐次坐标进行计算)。

// Ansewer: 1.70711, 4.12132 , 1
int main(){

    float TO_DEGREE = M_PI/180.0f;
    float rad = 45.0f * TO_DEGREE;

    Eigen::Vector3f P(2,1,1);
    // Translation
    Eigen::Matrix3f M {
        {1, 0, 1},
        {0, 1, 2},
        {0, 0, 1},
    };
    // Rotation
    Eigen::Matrix3f R {
        {cos(rad), -sin(rad), 0},
        {sin(rad), cos(rad), 0},
        {0, 0, 1},
    };

    Eigen::Vector3f P2 = M * R * P;
    std::cout << P2 << std::endl;

    return 0;
}