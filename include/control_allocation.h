#pragma once

#include <ArduinoEigenDense.h>

extern Eigen::MatrixXf Bplusminismall_pinv;
extern Eigen::MatrixXf Bplusfullsmall_pinv;
extern Eigen::VectorXf dw_config, dw_att, de_att, de_config;

void initializeControlAllocation();
void getpinvBplusmini();
