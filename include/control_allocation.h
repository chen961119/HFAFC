#pragma once

#include <ArduinoEigenDense.h>

// 控制器读取的分配结果；矩阵构造参数和临时矩阵留在 control_allocation.cpp 内。
extern Eigen::MatrixXf Bplusminismall_pinv;
extern Eigen::MatrixXf Bplusfullsmall_pinv;
extern Eigen::VectorXf dw_config, dw_att, de_att, de_config;

void initializeControlAllocation(); // 初始化惯量矩阵和舵效矩阵。
void getpinvBplusmini();
