#ifndef MADGWICK_H
#define MADGWICK_H

void Madgwick_Init(float sampleFreq, float beta);
void Madgwick_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Madgwick_GetEuler(float* roll, float* pitch, float* yaw);

#endif
