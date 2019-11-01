#ifndef PARAMETERS_H
#define PARAMETERS_H

#define PARAM_SIZE 27

// for debug
#include <stdio.h>
char loginfo_buffer[100];

bool parameters[PARAM_SIZE];

bool p_calibration_mode = false;

float mag_offsets[3]            = {0.0, 0.0, 0.0};
float mag_softiron_matrix[3][3] = {{0.0, 0.0, 0.0},
                                   {0.0, 0.0, 0.0},
                                   {0.0, 0.0, 0.0}};

int p_sensor_read_rate = 400;   // valid values are 50, 100, 200, 400
int p_output_rate_divider = 4;  // value values are 2, 4, 8, 16

bool p_adaptive_gain = true;
bool p_bias_estimation = true;

float p_gain_acc = 0.02;
float p_gain_mag = 0.01;
float p_bias_alpha = 0.25;

float p_kAngularVelocityThreshold = 0.2;
float p_kAccelerationThreshold = 0.4;
float p_kDeltaAngularVelocityThreshold = 0.05;

char imu_link[16] = "base_imu_link";
char mag_link[16] = "base_mag_link";

char *imu_link_ptr[1] = {imu_link};
char *mag_link_ptr[1] = {mag_link};

int p_gfsr = 2; // GFSR_500PS
int p_afsr = 1; // AFSR_4G

void handle_parameters(ros::NodeHandle &nh) {

    // set all to false
    for(int i=0; i<PARAM_SIZE; i++) {
        parameters[i] = false;
    }

    parameters[0] = nh.getParam("/params/imu/mag_offset_x", (float*) &mag_offsets[0], 1, 1000);
    parameters[1] = nh.getParam("/params/imu/mag_offset_y", (float*) &mag_offsets[1], 1, 1000);
    parameters[2] = nh.getParam("/params/imu/mag_offset_z", (float*) &mag_offsets[2], 1, 1000);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[3] = nh.getParam("/params/imu/mag_soft_iron_ix", (float*) &mag_softiron_matrix[0][0], 1, 1000);
    parameters[4] = nh.getParam("/params/imu/mag_soft_iron_iy", (float*) &mag_softiron_matrix[0][1], 1, 1000);
    parameters[5] = nh.getParam("/params/imu/mag_soft_iron_iz", (float*) &mag_softiron_matrix[0][2], 1, 1000);
    parameters[6] = nh.getParam("/params/imu/mag_soft_iron_jx", (float*) &mag_softiron_matrix[1][0], 1, 1000);
    parameters[7] = nh.getParam("/params/imu/mag_soft_iron_jy", (float*) &mag_softiron_matrix[1][1], 1, 1000);
    parameters[8] = nh.getParam("/params/imu/mag_soft_iron_jz", (float*) &mag_softiron_matrix[1][2], 1, 1000);
    parameters[9] = nh.getParam("/params/imu/mag_soft_iron_kx", (float*) &mag_softiron_matrix[2][0], 1, 1000);
    parameters[10] = nh.getParam("/params/imu/mag_soft_iron_ky", (float*) &mag_softiron_matrix[2][1], 1, 1000);
    parameters[11] = nh.getParam("/params/imu/mag_soft_iron_kz", (float*) &mag_softiron_matrix[2][2], 1, 1000);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[12] = nh.getParam("/params/imu/sensor_read_rate", (int*) &p_sensor_read_rate, 1, 1000);
    parameters[13] = nh.getParam("/params/imu/output_rate_divider", (int*) &p_output_rate_divider, 1, 1000);

    sprintf(loginfo_buffer, "sensor_read_rate: %d", p_sensor_read_rate);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "output_rate_divider: %d", p_output_rate_divider);
    nh.loginfo(loginfo_buffer);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[14] = nh.getParam("/params/imu/adaptive_gain", (bool*) &p_adaptive_gain, 1, 1000);
    parameters[15] = nh.getParam("/params/imu/bias_estimation", (bool*) &p_bias_estimation, 1, 1000);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[16] = nh.getParam("/params/imu/gain_mag", (float*) &p_gain_mag, 1, 1000);
    parameters[17] = nh.getParam("/params/imu/gain_acc", (float*) &p_gain_acc, 1, 1000);
    parameters[18] = nh.getParam("/params/imu/bias_alpha", (float*) &p_bias_alpha, 1, 1000);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[19] = nh.getParam("/params/imu/kAngularVelocityThreshold", (float*) &p_kAngularVelocityThreshold, 1, 1000);
    parameters[20] = nh.getParam("/params/imu/kAccelerationThreshold", (float*) &p_kAccelerationThreshold, 1, 1000);
    parameters[21] = nh.getParam("/params/imu/kDeltaAngularVelocityThreshold", (float*) &p_kDeltaAngularVelocityThreshold, 1, 1000);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[22] = nh.getParam("/params/imu/imu_frame_id", imu_link_ptr, 1, 1000);
    parameters[23] = nh.getParam("/params/imu/mag_frame_id", mag_link_ptr, 1, 1000);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[24] = nh.getParam("/params/imu/gfsr", (int*) &p_gfsr, 1, 1000);
    parameters[25] = nh.getParam("/params/imu/afsr", (int*) &p_afsr, 1, 1000);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    parameters[26] = nh.getParam("/params/imu/calibration_mode", (bool*) &p_calibration_mode, 1, 1000);

    sprintf(loginfo_buffer, "calibration mode: %s", p_calibration_mode ? "true" : "false");
    nh.loginfo(loginfo_buffer);

    nh.spinOnce();
    nh.getHardware()->delay(10);

    bool success = true;
    for(int i=0; i<PARAM_SIZE; i++) {
        if(!parameters[i]) {
            sprintf(loginfo_buffer, "fximu parameter for index: %d failed", i);
            nh.loginfo(loginfo_buffer);
            success = false;
        }
    }

    if(success) {
        sprintf(loginfo_buffer, "fximu parameters loaded with success");
        nh.loginfo(loginfo_buffer);
    }

}

#endif