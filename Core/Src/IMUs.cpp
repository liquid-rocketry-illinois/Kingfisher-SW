//
// Created by dyrel on 3/6/2026.
//

#include "IMUs.h"

//PRIVATE:

uint8_t IMUs::TMR_Compute() {
    Voted_BMI.accel_linear = tmrComp.Vote(Raw_BMI.BMIA_d.accel_linear,
                                          Raw_BMI.BMIB_d.accel_linear,
                                          Raw_BMI.BMIC_d.accel_linear);
    Voted_BMI.ang_vel      = tmrComp.Vote(Raw_BMI.BMIA_d.ang_vel,
                                          Raw_BMI.BMIB_d.ang_vel,
                                          Raw_BMI.BMIC_d.ang_vel);

    Quaternion_BMI.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_BMI, Voted_BMI.accel_linear, Voted_BMI.ang_vel);
    // Quaternion_ISM.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_ISM, Data_ISM.acceleration, Data_ISM.angular_velocity);
    return 0;
}
uint8_t IMUs::Normal_Compute() {
    Voted_BMI = BMI323_A.getRawData();
    Quaternion_BMI.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_BMI, Voted_BMI.accel_linear, Voted_BMI.ang_vel);
    // Quaternion_ISM.q = MATHEMATICS::Quaternion_Madgwick(&Quaternion_ISM, Data_ISM.acceleration, Data_ISM.angular_velocity);
    return 0;
}

//PUBLIC:

// Also initialize quaternions for BMI and ISM imus.
IMUs::IMUs() : Quaternion_BMI(), Quaternion_ISM(), Data_ISM() {}

IMUsStatus IMUs::Init(bool tmr_on) {
    TMR_STATE = tmr_on;

    sensorStatus.A = BMI323_A.Init(SENSOR1_I);
    if (TMR_STATE) {
        sensorStatus.B = BMI323_B.Init(SENSOR2_I);
        sensorStatus.C = BMI323_C.Init(SENSOR3_I);
    }

    // sensorStatus.ISM = ISM6HGx.Init();  // ISM6HGx not used this flight

    return sensorStatus;
}

IMUsStatus IMUs::Update() {
    if (BMI323_A.Update() != 0) sensorStatus.A = 1;
    else {
        Raw_BMI.BMIA_d = BMI323_A.getRawData();
        sensorStatus.A = 0;
    }

    if (TMR_STATE) {
        if (BMI323_B.Update() != 0) sensorStatus.B = 1;
        else {
            Raw_BMI.BMIB_d = BMI323_B.getRawData();
            sensorStatus.B = 0;
        }


        if (BMI323_C.Update() != 0) sensorStatus.C = 1;
        else {
            Raw_BMI.BMIC_d = BMI323_C.getRawData();
            sensorStatus.C = 0;
        }
    }

    // sensorStatus.ISM = ISM6HGx.Update();  // ISM6HGx not used this flight
    // Data_ISM = ISM6HGx.GetData();

    if (TMR_STATE) TMR_Compute();
    else           Normal_Compute();

    return sensorStatus;
}

BMI_Data IMUs::getVotedBMI() {
    return Voted_BMI;
}

// Index 0-2 for sensors A, B, and C. Sensor A is always on.
BMI_Data IMUs::getRawBMI(uint8_t index) {
    switch (index) {
        case 0:  return Raw_BMI.BMIA_d;
        case 1:  return Raw_BMI.BMIB_d;
        case 2:  return Raw_BMI.BMIC_d;
        default: return {};
    }
}

ISM_Data IMUs::getRawISM() {
    return Data_ISM;
}