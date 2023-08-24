#pragma once

void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void getQuaternion(float &qw, float &qx, float &qy, float &qz);
