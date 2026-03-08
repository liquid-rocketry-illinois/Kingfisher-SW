//
// Created by dyrel on 3/6/2026.
//

#include "IMUs.h"

//PRIVATE:

uint8_t IMU::TMR_Compute() {
    Vector3D<float> BMI_acc = tmrComp.Vote(Raw_BMI.BMIA_d.accel_linear,
                                    Raw_BMI.BMIB_d.accel_linear,
                                    Raw_BMI.BMIC_d.accel_linear);
    Vector3D<float> BMI_angVel = tmrComp.Vote( Raw_BMI.BMIA_d.ang_vel,
                                        Raw_BMI.BMIB_d.ang_vel,
                                        Raw_BMI.BMIC_d.ang_vel);

    Quaternion_BMI.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_BMI, BMI_acc, BMI_angVel);
    Quaternion_ISM.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_ISM, Data_ISM.acceleration, Data_ISM.angular_velocity);
}
uint8_t IMU::Normal_Compute() {
    BMI_Data Data = BMI323_A.getRawData();
    Quaternion_BMI.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_BMI, Data.accel_linear, Data.ang_vel);
    Quaternion_ISM.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_ISM, Data_ISM.acceleration, Data_ISM.angular_velocity);
}

//PUBLIC:

// Also initialize quaternions for BMI and ISM imus.
IMU::IMU(bool tmr_on) : Quaternion_BMI(), Quaternion_ISM() {
    TMR_STATE = tmr_on;
}

IMU::Status IMU::Init() {
    sensorStatus.A = BMI323_A.Init();
    sensorStatus.A = BMI323_B.Init();
    sensorStatus.A = BMI323_C.Init();
    sensorStatus.A = ISM6HGx.Init();

    return sensorStatus;
}
IMU::Status IMU::Update() {
    if (BMI323_A.Update() != 0) sensorStatus.A = false;
    else {
        Raw_BMI.BMIA_d = BMI323_A.getRawData();
        sensorStatus.A = true;
    }

    if (TMR_STATE) {
        if (BMI323_B.Update() != 0) sensorStatus.B = false;
        else {
            Raw_BMI.BMIB_d = BMI323_B.getRawData();
            sensorStatus.B = true;
        }


        if (BMI323_C.Update() != 0) sensorStatus.C = false;
        else {
            Raw_BMI.BMIC_d = BMI323_C.getRawData();
            sensorStatus.C = true;
        }
    }
    ISM6HGx.Update(); // Doesn't return status
    Data_ISM = ISM6HGx.GetData();

    return sensorStatus;
}

// Index 0-2 for sensors A, B, and C. Sensor A is always on.
BMI_Data IMU::getRawBMI(uint8_t index) {
    switch (index) {
        case 0: return Raw_BMI.BMIA_d;
        case 1: return Raw_BMI.BMIB_d;
        case 2: return Raw_BMI.BMIC_d;
    }
}

ISM_Data IMU::getRawISM() {
    return Data_ISM;
}