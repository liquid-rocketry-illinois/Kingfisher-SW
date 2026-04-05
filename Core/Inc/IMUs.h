//
// Created by dyrel on 3/6/2026.
//

#ifndef KINGFISHER_SW_IMUS_H
#define KINGFISHER_SW_IMUS_H

#include "SIMU_BMI323.h"
#include "IMU_ISM6HGx.h"
#include "Math/TripleModRedundancy.h"

struct IMUsStatus {
    int8_t A = 0;
    int8_t B = 0;
    int8_t C = 0;
    int8_t ISM = 0;
};

class IMUs {
private:
    SIMU_BMI323 BMI323_A;
    SIMU_BMI323 BMI323_B;
    SIMU_BMI323 BMI323_C;
    IMU_ISM6HGx ISM6HGx;
    TMR tmrComp;

    struct {
        BMI_Data BMIA_d;
        BMI_Data BMIB_d;
        BMI_Data BMIC_d;
    } Raw_BMI;

    IMUsStatus sensorStatus;

    bool TMR_STATE = false;

    Quaternion Quaternion_BMI;
    Quaternion Quaternion_ISM;

    ISM_Data Data_ISM;

    uint8_t TMR_Compute();
    uint8_t Normal_Compute();

public:
    IMUs(bool tmr_on);

    IMUsStatus Init();
    IMUsStatus Update();
    BMI_Data getRawBMI(uint8_t index);
    ISM_Data getRawISM();
};

#endif //KINGFISHER_SW_IMUS_H