// Created by admin on 5/10/2026.

#include "CTRLS_ControlAlgorithm.h"
#include <cstdlib>  // for abs

namespace {

static constexpr float kThrustT[] = {
    0.07f, 0.211f, 0.353f, 0.495f, 0.636f, 0.778f, 0.92f, 1.061f,
    1.203f, 1.345f, 1.486f, 1.628f, 1.771f, 1.914f, 2.055f, 2.197f,
    2.339f, 2.48f, 2.622f, 2.764f, 2.905f, 3.047f, 3.189f, 3.33f,
    3.473f, 3.616f
};

static constexpr float kThrustN[] = {
    2441.945f, 2495.46f, 2556.133f, 2601.596f, 2637.66f, 2660.804f, 2676.486f, 2687.081f,
    2695.807f, 2694.493f, 2684.268f, 2667.289f, 2629.961f, 2578.923f, 2522.074f, 2461.704f,
    2393.518f, 2303.939f, 2201.61f, 2097.461f, 2010.409f, 1275.776f, 418.836f, 17.586f,
    3.669f, 0.0f
};

static constexpr float kDragMach[] = {
    0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f,
    0.09f, 0.1f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f,
    0.17f, 0.18f, 0.19f, 0.2f, 0.21f, 0.22f, 0.23f, 0.24f,
    0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.3f, 0.31f, 0.32f,
    0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.4f,
    0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f,
    0.49f, 0.5f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f,
    0.57f, 0.58f, 0.59f, 0.6f, 0.61f, 0.62f, 0.63f, 0.64f,
    0.65f, 0.66f, 0.67f, 0.68f, 0.69f, 0.7f, 0.71f, 0.72f,
    0.73f, 0.74f, 0.75f, 0.76f, 0.77f, 0.78f, 0.79f, 0.8f,
    0.81f, 0.82f, 0.83f, 0.84f, 0.85f, 0.86f, 0.87f, 0.88f,
    0.89f, 0.9f, 0.91f, 0.92f, 0.93f, 0.94f, 0.95f, 0.96f,
    0.97f, 0.98f, 0.99f, 1.0f
};

static constexpr float kDragCd[] = {
    0.569051589f, 0.52605377f, 0.525832698f, 0.525615889f, 0.5254033f, 0.52519502f, 0.524991009f, 0.524791277f,
    0.524595889f, 0.524404882f, 0.524218202f, 0.524035931f, 0.523858126f, 0.523684782f, 0.523515978f, 0.52335172f,
    0.523192083f, 0.523037128f, 0.522886922f, 0.522741522f, 0.522600976f, 0.522465341f, 0.522334793f, 0.522209327f,
    0.522089082f, 0.522456875f, 0.522829131f, 0.523205929f, 0.523587459f, 0.523973811f, 0.524365069f, 0.524761467f,
    0.525163147f, 0.525570251f, 0.525982943f, 0.526401418f, 0.526825879f, 0.527256541f, 0.527693567f, 0.528137297f,
    0.528587922f, 0.529045713f, 0.529510946f, 0.529983978f, 0.530465078f, 0.530954593f, 0.531452951f, 0.531960482f,
    0.532477693f, 0.533004953f, 0.533542788f, 0.534091723f, 0.534652347f, 0.535225235f, 0.535811092f, 0.536410567f,
    0.53702454f, 0.537653713f, 0.538299096f, 0.538961663f, 0.540487449f, 0.542000834f, 0.543533427f, 0.545086685f,
    0.546662174f, 0.548261658f, 0.54988696f, 0.551540284f, 0.553223938f, 0.554940606f, 0.555828201f, 0.556660936f,
    0.557492007f, 0.55832138f, 0.559149071f, 0.559975082f, 0.560799396f, 0.561622012f, 0.562443031f, 0.563262365f,
    0.566191044f, 0.569118026f, 0.572043456f, 0.574967195f, 0.577889305f, 0.580809862f, 0.583728762f, 0.586646034f,
    0.589561727f, 0.59247585f, 0.596504685f, 0.608591193f, 0.638876354f, 0.681316916f, 0.723757477f, 0.766198039f,
    0.808638601f, 0.851079162f, 0.893519724f, 0.935960286f
};

struct WeatherPoint {
    float altitude_agl_m;
    float rho;
    float temperature_K;
    float g;
    float wind_x;
    float wind_y;
};

static constexpr WeatherPoint kWeatherProfile[] = {
    {0.000f, 1.1927284f, 286.248204f, 9.80137498f, 8.02070887f, -4.76881510f},
    {50.000f, 1.18727276f, 285.862390f, 9.80122070f, 8.59885029f, -5.19170412f},
    {100.000f, 1.18181711f, 285.476577f, 9.80106643f, 9.17699171f, -5.61459314f},
    {150.000f, 1.17636147f, 285.090763f, 9.80091216f, 9.75513313f, -6.03748215f},
    {200.000f, 1.17090582f, 284.704949f, 9.80075789f, 10.33327460f, -6.46037117f},
    {250.000f, 1.16545017f, 284.319135f, 9.80060362f, 10.91141600f, -6.88326018f},
    {300.000f, 1.16017964f, 283.909141f, 9.80044936f, 11.25534170f, -7.27045563f},
    {350.000f, 1.15495687f, 283.492908f, 9.80029510f, 11.53883050f, -7.64844071f},
    {400.000f, 1.14973409f, 283.076674f, 9.80014084f, 11.82231940f, -8.02642580f},
    {450.000f, 1.14451132f, 282.660440f, 9.79998659f, 12.10580820f, -8.40441089f},
    {500.000f, 1.13890699f, 282.347592f, 9.79983233f, 12.35553890f, -8.77259637f},
    {550.000f, 1.13259489f, 282.226521f, 9.79967809f, 12.54264990f, -9.12260399f},
    {600.000f, 1.12628279f, 282.105449f, 9.79952385f, 12.72976090f, -9.47261160f},
    {650.000f, 1.11997068f, 281.984377f, 9.79936961f, 12.91687190f, -9.82261922f},
    {700.000f, 1.11365858f, 281.863305f, 9.79921537f, 13.10398290f, -10.17262680f},
    {750.000f, 1.10724125f, 281.822096f, 9.79906115f, 13.15960910f, -10.30735320f},
    {800.000f, 1.10079884f, 281.799924f, 9.79890692f, 13.18389260f, -10.39076170f},
    {850.000f, 1.09435643f, 281.777752f, 9.79875270f, 13.20817610f, -10.47417020f},
    {900.000f, 1.08791402f, 281.755580f, 9.79859848f, 13.23245960f, -10.55757870f},
    {950.000f, 1.08147162f, 281.733408f, 9.79844427f, 13.25674310f, -10.64098720f},
    {1000.000f, 1.07502921f, 281.711237f, 9.79829007f, 13.28102660f, -10.72439570f},
    {1050.000f, 1.0685868f, 281.689065f, 9.79813586f, 13.30531010f, -10.80780420f},
    {1100.000f, 1.06214439f, 281.666893f, 9.79798166f, 13.32959360f, -10.89121270f},
    {1150.000f, 1.05570198f, 281.644721f, 9.79782746f, 13.35387700f, -10.97462120f},
    {1200.000f, 1.04955059f, 281.572947f, 9.79767327f, 13.41799130f, -10.98972760f},
    {1250.000f, 1.04397416f, 281.403175f, 9.79751908f, 13.56080020f, -10.86988800f},
    {1300.000f, 1.03839774f, 281.233402f, 9.79736490f, 13.70360900f, -10.75004840f},
    {1350.000f, 1.03282132f, 281.063629f, 9.79721072f, 13.84641790f, -10.63020880f},
    {1400.000f, 1.0272449f, 280.893856f, 9.79705654f, 13.98922670f, -10.51036920f},
    {1450.000f, 1.02166848f, 280.724084f, 9.79690236f, 14.13203560f, -10.39052960f},
    {1500.000f, 1.01609205f, 280.554311f, 9.79674819f, 14.27484440f, -10.27069000f},
    {1550.000f, 1.01051563f, 280.384538f, 9.79659402f, 14.41765330f, -10.15085040f},
    {1600.000f, 1.00493921f, 280.214765f, 9.79643986f, 14.56046210f, -10.03101080f},
    {1650.000f, 0.999362788f, 280.044993f, 9.79628570f, 14.70327100f, -9.91117120f},
    {1700.000f, 0.993813937f, 279.909188f, 9.79613154f, 14.88383680f, -9.71354365f},
    {1750.000f, 0.988322963f, 279.844690f, 9.79597738f, 15.14366040f, -9.35262669f},
    {1800.000f, 0.982831989f, 279.780191f, 9.79582323f, 15.40348400f, -8.99170973f},
    {1850.000f, 0.977341014f, 279.715693f, 9.79566909f, 15.66330760f, -8.63079277f},
    {1900.000f, 0.97185004f, 279.651195f, 9.79551494f, 15.92313120f, -8.26987581f},
    {1950.000f, 0.966359066f, 279.586696f, 9.79536080f, 16.18295480f, -7.90895886f},
    {2000.000f, 0.960868092f, 279.522198f, 9.79520667f, 16.44277850f, -7.54804190f},
    {2050.000f, 0.955377117f, 279.457699f, 9.79505253f, 16.70260210f, -7.18712494f},
    {2100.000f, 0.949886143f, 279.393201f, 9.79489840f, 16.96242570f, -6.82620798f},
    {2150.000f, 0.944395169f, 279.328702f, 9.79474428f, 17.22224930f, -6.46529102f},
    {2200.000f, 0.938904195f, 279.264204f, 9.79459015f, 17.48207290f, -6.10437406f},
    {2250.000f, 0.93341322f, 279.199705f, 9.79443604f, 17.74189650f, -5.74345710f},
    {2300.000f, 0.927922246f, 279.135207f, 9.79428192f, 18.00172020f, -5.38254014f},
    {2350.000f, 0.922431272f, 279.070708f, 9.79412781f, 18.26154380f, -5.02162318f},
    {2400.000f, 0.916940297f, 279.006210f, 9.79397370f, 18.52136740f, -4.66070622f},
    {2450.000f, 0.911449323f, 278.941711f, 9.79381960f, 18.78119100f, -4.29978926f},
    {2500.000f, 0.905958349f, 278.877213f, 9.79366550f, 19.04101460f, -3.93887230f},
    {2550.000f, 0.900467375f, 278.812714f, 9.79351140f, 19.30083820f, -3.57795534f},
    {2600.000f, 0.8949764f, 278.748216f, 9.79335731f, 19.56066190f, -3.21703838f},
    {2650.000f, 0.889485426f, 278.683718f, 9.79320322f, 19.82048550f, -2.85612142f},
    {2700.000f, 0.883994452f, 278.619219f, 9.79304913f, 20.08030910f, -2.49520446f},
    {2750.000f, 0.878503478f, 278.554721f, 9.79289505f, 20.34013270f, -2.13428750f},
    {2800.000f, 0.873561471f, 278.399511f, 9.79274097f, 20.57064450f, -1.79448684f},
    {2850.000f, 0.869259552f, 278.138535f, 9.79258690f, 20.76697920f, -1.47930749f},
    {2900.000f, 0.864957634f, 277.877558f, 9.79243283f, 20.96331380f, -1.16412813f},
    {2950.000f, 0.860655715f, 277.616582f, 9.79227876f, 21.15964850f, -0.84894877f},
    {3000.000f, 0.856353797f, 277.355605f, 9.79212470f, 21.35598310f, -0.53376942f},
    {3050.000f, 0.852051878f, 277.094629f, 9.79197064f, 21.55231780f, -0.21859006f},
    {3100.000f, 0.847749959f, 276.833652f, 9.79181658f, 21.74865240f, 0.09658929f},
    {3150.000f, 0.843448041f, 276.572676f, 9.79166253f, 21.94498710f, 0.41176865f},
    {3200.000f, 0.839146122f, 276.311699f, 9.79150848f, 22.14132180f, 0.72694801f},
    {3250.000f, 0.834844203f, 276.050723f, 9.79135443f, 22.33765640f, 1.04212736f},
    {3300.000f, 0.830542285f, 275.789746f, 9.79120039f, 22.53399110f, 1.35730672f},
    {3350.000f, 0.826240366f, 275.528769f, 9.79104635f, 22.73032570f, 1.67248608f},
    {3400.000f, 0.821938448f, 275.267793f, 9.79089232f, 22.92666040f, 1.98766543f},
    {3450.000f, 0.817636529f, 275.006816f, 9.79073828f, 23.12299500f, 2.30284479f},
    {3500.000f, 0.81333461f, 274.745840f, 9.79058426f, 23.31932970f, 2.61802414f},
    {3550.000f, 0.809032692f, 274.484863f, 9.79043023f, 23.51566430f, 2.93320350f},
    {3600.000f, 0.804730773f, 274.223887f, 9.79027621f, 23.71199900f, 3.24838286f},
    {3650.000f, 0.800428855f, 273.962910f, 9.79012219f, 23.90833370f, 3.56356221f},
    {3700.000f, 0.796126936f, 273.701934f, 9.78996818f, 24.10466830f, 3.87874157f},
    {3750.000f, 0.791825017f, 273.440957f, 9.78981417f, 24.30100300f, 4.19392093f},
    {3800.000f, 0.787523099f, 273.179981f, 9.78966016f, 24.49733760f, 4.50910028f},
    {3850.000f, 0.78322118f, 272.919004f, 9.78950616f, 24.69367230f, 4.82427964f},
    {3900.000f, 0.778919262f, 272.658028f, 9.78935216f, 24.89000690f, 5.13945900f},
    {3950.000f, 0.774617343f, 272.397051f, 9.78919817f, 25.08634160f, 5.45463835f},
    {4000.000f, 0.770315424f, 272.136075f, 9.78904418f, 25.28267630f, 5.76981771f},
};

static void copyTable(float* dst, const float* src, int n) {
    for (int i = 0; i < n; ++i) dst[i] = src[i];
}

} // namespace

void loadControlFreakRocketConfig(RocketConfig& cfg) {
    // Values copied from FV-Controls/rockets/Control Freak/Rocketpy/rocketpy.ipynb.
    cfg.m_0    = 21.8520008f;
    cfg.m_f    = 18.2000000f;
    cfg.m_prop = 3.65200077f;

    cfg.I_0   = 12.7651156f;
    cfg.I_f   = 10.1818760f;
    cfg.I_3_0 = 0.0648822543f;
    cfg.I_3_f = 0.0613600000f;

    cfg.x_CG_0 = 1.88045662f;
    cfg.x_CG_f = 1.74179228f;

    cfg.d    = 0.13081f;
    cfg.L_ne = 2.870f;

    cfg.t_rail    = 0.309f;
    cfg.t_burnout = 3.28f;
    cfg.t_apogee  = 25.678f;

    cfg.base_cn   = 10.581f;
    cfg.base_cp   = 2.17842618f;
    cfg.canard_cn = 0.857f;
    cfg.canard_cp = 1.049f;
    cfg.canard_plane_angle_deg = 0.0f;

    cfg.N_fins      = 4;
    cfg.Cr          = 0.305f;
    cfg.Ct          = 0.152f;
    cfg.s           = 0.133f;
    cfg.delta_deg   = 0.1139f;
    cfg.Cnalpha_fin = 2.34275f;

    cfg.canard_moment_coeff_per_deg = -2.23e-6f;
    cfg.roll_damping_lambda = 30.0f;
    cfg.max_deflection_rad  = 9.0f * (M_PI / 180.0f);
    cfg.max_deflection_rate = 428.571428571f * (M_PI / 180.0f);
    cfg.min_control_speed   = 30.0f;
    cfg.irec_compliant      = true;

    cfg.Q_angular[0] = 2.0e-4f;
    cfg.Q_angular[1] = 2.0e-4f;
    cfg.Q_angular[2] = 3.0e-4f;
    cfg.Q_velocity[0] = 2.5e-3f;
    cfg.Q_velocity[1] = 2.5e-3f;
    cfg.Q_velocity[2] = 2.5e-3f;
    for (int i = 0; i < 4; ++i) cfg.Q_quat[i] = 1.0e-5f;

    cfg.accel_noise_std_g = 0.5f;
    cfg.gyro_noise_std    = 0.05f;
    cfg.accel_model_std_g = 0.5f;
    cfg.accel_burn_std_g  = 5.0f;
    cfg.P0_scale          = 0.01f;

    cfg.rho_0    = 1.213f;
    cfg.g_0      = 9.8014f;
    cfg.v_wind_x = 0.0f;
    cfg.v_wind_y = 0.0f;

    for (int i = 0; i < 10; ++i) cfg.x0[i] = 0.0f;
    // Matches FV-Controls setSimParamsFromRailAngle(45 deg, nose_to_tail):
    // launch roll angle = 360 - 45 = 315 deg.
    cfg.x0[6] = -0.923879533f;
    cfg.x0[9] =  0.382683432f;

    cfg.rem_enabled             = true;
    cfg.rem_post_burn_delay_s   = 0.25f;
    cfg.rem_min_cmd_rad         = 2.0f * (M_PI / 180.0f);
    cfg.rem_min_expected_accel  = 0.5f;
    cfg.rem_min_measured_accel  = 0.5f;
    cfg.rem_required_mismatches = 5;
    cfg.rem_allow_flip_back     = false;

    cfg.thrust_n = static_cast<int>(sizeof(kThrustT) / sizeof(kThrustT[0]));
    copyTable(cfg.thrust_t, kThrustT, cfg.thrust_n);
    copyTable(cfg.thrust_N, kThrustN, cfg.thrust_n);

    cfg.drag_on_n = static_cast<int>(sizeof(kDragMach) / sizeof(kDragMach[0]));
    copyTable(cfg.drag_on_mach, kDragMach, cfg.drag_on_n);
    copyTable(cfg.drag_on_cd, kDragCd, cfg.drag_on_n);

    cfg.drag_off_n = static_cast<int>(sizeof(kDragMach) / sizeof(kDragMach[0]));
    copyTable(cfg.drag_off_mach, kDragMach, cfg.drag_off_n);
    copyTable(cfg.drag_off_cd, kDragCd, cfg.drag_off_n);
}

// ─── inv6: LU decomposition with partial pivoting ────────────────────────────
Mat<6,6> inv6(const Mat<6,6>& A) {
    Mat<6,6> LU = A;
    int piv[6] = {0,1,2,3,4,5};

    for (int k = 0; k < 6; k++) {
        // partial pivot
        int   mx  = k;
        float mxv = fabsf(LU(k,k));
        for (int i = k+1; i < 6; i++) {
            if (fabsf(LU(i,k)) > mxv) { mxv = fabsf(LU(i,k)); mx = i; }
        }
        if (mx != k) {
            for (int j = 0; j < 6; j++) { float t = LU(k,j); LU(k,j)=LU(mx,j); LU(mx,j)=t; }
            int  tp = piv[k]; piv[k]=piv[mx]; piv[mx]=tp;
        }
        if (fabsf(LU(k,k)) < 1e-12f) return eye<6>(); // singular fallback
        for (int i = k+1; i < 6; i++) {
            LU(i,k) /= LU(k,k);
            for (int j = k+1; j < 6; j++) LU(i,j) -= LU(i,k)*LU(k,j);
        }
    }

    Mat<6,6> inv;
    for (int col = 0; col < 6; col++) {
        float b[6] = {};
        b[col] = 1.0f;
        // apply row permutation
        for (int k = 0; k < 6; k++) {
            if (piv[k] != k) { float t = b[piv[k]]; b[piv[k]]=b[k]; b[k]=t; }
        }
        // forward substitution (unit lower triangular L)
        for (int i = 1; i < 6; i++)
            for (int j = 0; j < i; j++) b[i] -= LU(i,j)*b[j];
        // back substitution (upper triangular U)
        for (int i = 5; i >= 0; i--) {
            for (int j = i+1; j < 6; j++) b[i] -= LU(i,j)*b[j];
            b[i] /= LU(i,i);
        }
        for (int i = 0; i < 6; i++) inv(i,col) = b[i];
    }
    return inv;
}

// ─── lerp ─────────────────────────────────────────────────────────────────────
float lerp(const float* xs, const float* ys, int n, float xi) {
    if (n <= 0) return 0.0f;
    if (xi <= xs[0]) return ys[0];
    if (xi >= xs[n-1]) return ys[n-1];
    // binary search for bracket
    int lo = 0, hi = n-1;
    while (hi - lo > 1) {
        int mid = (lo+hi) >> 1;
        if (xs[mid] <= xi) lo = mid; else hi = mid;
    }
    float t = (xi - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + t * (ys[hi] - ys[lo]);
}

// ─── Physics constructor ──────────────────────────────────────────────────────
Physics::Physics(const RocketConfig& c) : cfg(c) {
    // Match FV-Controls/src/dynamics/momentsforces.py roll EOM.
    const float gamma = (cfg.Cr > 0.0f) ? (cfg.Ct / cfg.Cr) : 0.0f;
    r_t = cfg.d * 0.5f;
    tau = (r_t > 0.0f) ? ((cfg.s + r_t) / r_t) : 1.0f;

    y_ma = (cfg.s / 3.0f) * (1.0f + 2.0f*gamma) / (1.0f + gamma);

    const float tau2 = tau * tau;
    const float tau_minus_1 = tau - 1.0f;
    const float asin_arg = (tau2 - 1.0f) / (tau2 + 1.0f);
    const float asin_term = asinf(fmaxf(-1.0f, fminf(1.0f, asin_arg)));
    const float shape_f =
        (1.0f / (M_PI * M_PI)) *
        ((M_PI*M_PI / 4.0f) * ((tau + 1.0f)*(tau + 1.0f) / tau2)
       + (M_PI * (tau2 + 1.0f)*(tau2 + 1.0f) / (tau2 * tau_minus_1*tau_minus_1)) * asin_term
       - (2.0f * M_PI * (tau + 1.0f)) / (tau * tau_minus_1)
       + ((tau2 + 1.0f)*(tau2 + 1.0f) / (tau2 * tau_minus_1*tau_minus_1)) * asin_term*asin_term
       - (4.0f * (tau + 1.0f) / (tau * tau_minus_1)) * asin_term
       + (8.0f / (tau_minus_1*tau_minus_1)) * logf((tau2 + 1.0f) / (2.0f * tau)));

    // Runtime uses M_f_roll = K_f * q_dyn.
    K_f = shape_f * (float)cfg.N_fins * (y_ma + r_t) * cfg.Cnalpha_fin
          * cfg.delta_rad * cfg.A_ref;

    const float trap_integral = cfg.s / 12.0f *
        ((cfg.Cr + 3.0f*cfg.Ct) * cfg.s*cfg.s
       + 4.0f * (cfg.Cr + 2.0f*cfg.Ct) * cfg.s * r_t
       + 6.0f * (cfg.Cr + cfg.Ct) * r_t*r_t);
    C_ldw = 2.0f * (float)cfg.N_fins * cfg.Cnalpha_fin
            / (cfg.A_ref * cfg.d*cfg.d) * cosf(cfg.delta_rad) * trap_integral;
    const float shape_d = 1.0f +
        (((tau - gamma) / tau) - ((1.0f - gamma) / tau_minus_1) * logf(tau)) /
        (((tau + 1.0f) * (tau - gamma) / 2.0f)
       - ((1.0f - gamma) * (tau*tau*tau - 1.0f) / (3.0f * tau_minus_1)));

    // Runtime uses M_d_roll = 0.5*rho*v_mag*K_d*w3.
    K_d = shape_d * cfg.A_ref * cfg.d * C_ldw * (cfg.d * 0.5f);
}

// ─── Time-varying parameters ──────────────────────────────────────────────────
float Physics::getMass(float t) const {
    if (t >= cfg.t_burnout) return cfg.m_f;
    return cfg.m_0 - cfg.m_prop * (t / cfg.t_burnout);
}

float Physics::getInertiaPitch(float t) const {
    if (t >= cfg.t_burnout) return cfg.I_f;
    return cfg.I_0 + (cfg.I_f - cfg.I_0) * (t / cfg.t_burnout);
}

float Physics::getInertiaRoll(float t) const {
    if (t >= cfg.t_burnout) return cfg.I_3_f;
    return cfg.I_3_0 + (cfg.I_3_f - cfg.I_3_0) * (t / cfg.t_burnout);
}

float Physics::getInertiaDotPitch(float t) const {
    if (t >= cfg.t_burnout) return 0.0f;
    return (cfg.I_f - cfg.I_0) / cfg.t_burnout;
}

float Physics::getInertiaDotRoll(float t) const {
    if (t >= cfg.t_burnout) return 0.0f;
    return (cfg.I_3_f - cfg.I_3_0) / cfg.t_burnout;
}

float Physics::getCG(float t) const {
    if (t >= cfg.t_burnout) return cfg.x_CG_f;
    return cfg.x_CG_0 + (cfg.x_CG_f - cfg.x_CG_0) * (t / cfg.t_burnout);
}

float Physics::getThrust(float t) const {
    if (t < 0.0f || t > cfg.t_burnout || cfg.thrust_n == 0) return 0.0f;
    if (t < cfg.thrust_t[0] || t > cfg.thrust_t[cfg.thrust_n - 1]) return 0.0f;
    return lerp(cfg.thrust_t, cfg.thrust_N, cfg.thrust_n, t);
}

float Physics::getDragCoeff(float mach, float t) const {
    if (t < cfg.t_burnout && cfg.drag_on_n > 0)
        return lerp(cfg.drag_on_mach, cfg.drag_on_cd, cfg.drag_on_n, mach);
    if (cfg.drag_off_n > 0)
        return lerp(cfg.drag_off_mach, cfg.drag_off_cd, cfg.drag_off_n, mach);
    return 0.45f; // fallback
}

float Physics::speedOfSound(float T_kelvin) {
    return sqrtf(1.4f * 287.05f * T_kelvin);
}

WeatherSample Physics::weatherAtAltitude(float alt_m) const {
    static constexpr int n = static_cast<int>(sizeof(kWeatherProfile) / sizeof(kWeatherProfile[0]));
    if (n <= 0) {
        WeatherSample fallback;
        fallback.rho = cfg.rho_0;
        fallback.temperature_K = 288.15f;
        fallback.g = cfg.g_0;
        fallback.wind_x = cfg.v_wind_x;
        fallback.wind_y = cfg.v_wind_y;
        return fallback;
    }

    auto sampleFrom = [](const WeatherPoint& p) {
        WeatherSample s;
        s.rho = p.rho;
        s.temperature_K = p.temperature_K;
        s.g = p.g;
        s.wind_x = p.wind_x;
        s.wind_y = p.wind_y;
        return s;
    };

    if (alt_m <= kWeatherProfile[0].altitude_agl_m) return sampleFrom(kWeatherProfile[0]);
    if (alt_m >= kWeatherProfile[n - 1].altitude_agl_m) return sampleFrom(kWeatherProfile[n - 1]);

    int lo = 0;
    int hi = n - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) >> 1;
        if (kWeatherProfile[mid].altitude_agl_m <= alt_m) lo = mid;
        else hi = mid;
    }

    const WeatherPoint& a = kWeatherProfile[lo];
    const WeatherPoint& b = kWeatherProfile[hi];
    const float u = (alt_m - a.altitude_agl_m) / (b.altitude_agl_m - a.altitude_agl_m);

    WeatherSample s;
    s.rho           = a.rho           + u * (b.rho           - a.rho);
    s.temperature_K = a.temperature_K + u * (b.temperature_K - a.temperature_K);
    s.g             = a.g             + u * (b.g             - a.g);
    s.wind_x        = a.wind_x        + u * (b.wind_x        - a.wind_x);
    s.wind_y        = a.wind_y        + u * (b.wind_y        - a.wind_y);
    return s;
}

// ─── R_BW: world-to-body DCM from unit quaternion ────────────────────────────
void Physics::R_BW(float qw, float qx, float qy, float qz, float R[3][3]) {
    const float qx2=qx*qx, qy2=qy*qy, qz2=qz*qz;
    R[0][0] = 1.0f - 2.0f*(qy2+qz2);   R[0][1] = 2.0f*(qx*qy+qw*qz);    R[0][2] = 2.0f*(qx*qz-qw*qy);
    R[1][0] = 2.0f*(qx*qy-qw*qz);      R[1][1] = 1.0f - 2.0f*(qx2+qz2); R[1][2] = 2.0f*(qy*qz+qw*qx);
    R[2][0] = 2.0f*(qx*qz+qw*qy);      R[2][1] = 2.0f*(qy*qz-qw*qx);    R[2][2] = 1.0f - 2.0f*(qx2+qy2);
}

// ─── Air-relative velocity in body frame ─────────────────────────────────────
void Physics::airRelativeVelocity(const StateVec& x, float v_air[3]) const {
    airRelativeVelocity(x, cfg.v_wind_x, cfg.v_wind_y, v_air);
}

void Physics::airRelativeVelocity(const StateVec& x, float wind_x, float wind_y,
                                  float v_air[3]) const {
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);
    float R[3][3];
    R_BW(qw, qx, qy, qz, R);
    // rotate to body frame
    float w_body[3] = {
        R[0][0]*wind_x + R[0][1]*wind_y,
        R[1][0]*wind_x + R[1][1]*wind_y,
        R[2][0]*wind_x + R[2][1]*wind_y
    };
    v_air[0] = x(3,0) - w_body[0];
    v_air[1] = x(4,0) - w_body[1];
    v_air[2] = x(5,0) - w_body[2];
}

// ─── Canard moment (CFD numeric) ──────────────────────────────────────────────
float Physics::canardMoment(float v_mag, float zeta_rad) const {
    return cfg.canard_moment_coeff_per_deg * v_mag * v_mag
           * (zeta_rad * (180.0f / M_PI));
}

float Physics::canardMomentJacobian(float v_mag) const {
    return cfg.canard_moment_coeff_per_deg * v_mag * v_mag * (180.0f / M_PI);
}

// ─── Full nonlinear EOM ───────────────────────────────────────────────────────
StateVec Physics::eom(float t, const StateVec& x, float u_zeta, float alt) const {
    // Extract state
    float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);

    // Normalize quaternion
    float qn = sqrtf(qw*qw+qx*qx+qy*qy+qz*qz);
    if (qn > 1e-8f) { qw/=qn; qx/=qn; qy/=qn; qz/=qn; }

    // Time-varying params
    const float m    = getMass(t);
    const float I1   = getInertiaPitch(t);
    const float I3   = getInertiaRoll(t);
    const float I1d  = getInertiaDotPitch(t);
    const float I3d  = getInertiaDotRoll(t);
    const float xCG  = getCG(t);
    const float Thr  = getThrust(t);
    const WeatherSample wx = weatherAtAltitude(alt);

    // Rotation matrix
    float R[3][3];
    R_BW(qw, qx, qy, qz, R);

    // Air-relative velocity
    float va[3];
    airRelativeVelocity(x, wx.wind_x, wx.wind_y, va);

    // Airspeed
    const float vmag2 = va[0]*va[0]+va[1]*va[1]+va[2]*va[2];
    const float vmag  = sqrtf(vmag2);
    static constexpr float EPS_AOA = 1.0e-9f;
    const float vmag_reg = sqrtf(vmag2 + EPS_AOA*EPS_AOA);
    const float v_xy_mag = sqrtf(va[0]*va[0] + va[1]*va[1] + EPS_AOA*EPS_AOA);

    // Angle of attack (from axial z-body), clamped to ±15 deg
    static constexpr float AoA_MAX = 15.0f * (M_PI / 180.0f);
    float alpha = 0.0f;
    if (vmag > 0.1f)
        alpha = atan2f(v_xy_mag, va[2] + EPS_AOA);
    if (alpha >  AoA_MAX) alpha =  AoA_MAX;
    if (alpha < -AoA_MAX) alpha = -AoA_MAX;

    // Directional canard-plane AoA, matching FV-Controls momentsforces.py.
    const float canard_lat = va[0]*cfg.canard_dir_1 + va[1]*cfg.canard_dir_2;
    const float canard_lat_mag = sqrtf(canard_lat*canard_lat + EPS_AOA*EPS_AOA);
    const float vcan2 = canard_lat*canard_lat + va[2]*va[2];
    const float vcan_mag = sqrtf(vcan2 + EPS_AOA*EPS_AOA);
    float alpha_can = 0.0f;
    if (vcan_mag > 0.1f)
        alpha_can = atan2f(canard_lat_mag, va[2] + EPS_AOA);
    if (alpha_can >  AoA_MAX) alpha_can =  AoA_MAX;
    if (alpha_can < -AoA_MAX) alpha_can = -AoA_MAX;

    // Mach, Cd, dynamic pressure
    const float c_snd = speedOfSound(wx.temperature_K);
    const float mach  = (c_snd > 0.0f) ? vmag / c_snd : 0.0f;
    const float Cd    = getDragCoeff(mach, t);
    const float rho   = wx.rho;
    const float q_dyn = 0.5f * rho * vmag2;

    // Rail: no lift/moments before release. Thrust, gravity, and drag still act,
    // matching FV-Controls' force model.
    const bool on_rail = (t < cfg.t_rail);

    // ── Forces (body frame) ──────────────────────────────────────────────────
    float F[3] = {0.0f, 0.0f, 0.0f};

    // Gravity: g_world = [0,0,-g]; g_body = R_BW * g_world = -g * R[:,2]
    F[0] += m * (-wx.g * R[0][2]);
    F[1] += m * (-wx.g * R[1][2]);
    F[2] += m * (-wx.g * R[2][2]);

    // Thrust along +z body axis
    F[2] += Thr;

    // Aerodynamic drag (along -v_air)
    if (vmag > 0.1f) {
        const float D = Cd * cfg.A_ref * q_dyn / vmag_reg;
        F[0] -= D * va[0];
        F[1] -= D * va[1];
        F[2] -= D * va[2];
    }

    if (!on_rail && vmag > 0.1f) {
        // Symmetric base-rocket normal force.
        const float L_base = q_dyn * cfg.base_cn * alpha * cfg.A_ref;
        const float cos_a = cosf(alpha);
        F[0] += L_base * (-cos_a * va[0] / v_xy_mag);
        F[1] += L_base * (-cos_a * va[1] / v_xy_mag);
        F[2] += L_base * sinf(alpha);

        // Two-canard normal force acts only in the configured canard plane.
        const float q_can = 0.5f * rho * vcan2;
        const float L_can = q_can * cfg.canard_cn * alpha_can * cfg.A_ref;
        const float cos_a_can = cosf(alpha_can);
        const float canard_lat_sign = canard_lat / canard_lat_mag;
        F[0] += L_can * (-cos_a_can * canard_lat_sign * cfg.canard_dir_1);
        F[1] += L_can * (-cos_a_can * canard_lat_sign * cfg.canard_dir_2);
        F[2] += L_can * sinf(alpha_can);
    }

    // ── Moments (body frame) ─────────────────────────────────────────────────
    float M[3] = {0.0f, 0.0f, 0.0f};

    if (!on_rail) {
        // Corrective moments: symmetric base plus directional canard plane.
        if (vmag > 0.1f) {
            const float cos_a = va[2] / vmag_reg;
            const float C_raw_base = vmag2 * cfg.A_ref * cfg.base_cn * alpha
                                   * (cfg.base_cp - xCG) * rho * 0.5f;
            M[0] += -C_raw_base * cos_a * va[1] / v_xy_mag;
            M[1] +=  C_raw_base * cos_a * va[0] / v_xy_mag;

            const float cos_a_can = va[2] / vcan_mag;
            const float C_raw_can = vcan2 * cfg.A_ref * cfg.canard_cn * alpha_can
                                  * (cfg.canard_cp - xCG) * rho * 0.5f;
            const float canard_lat_sign = canard_lat / canard_lat_mag;
            M[0] += -C_raw_can * cos_a_can * canard_lat_sign * cfg.canard_dir_2;
            M[1] +=  C_raw_can * cos_a_can * canard_lat_sign * cfg.canard_dir_1;
        }

        // Propulsive and aerodynamic pitch/yaw damping from FV-Controls.
        const float mdot = (cfg.t_burnout > 0.0f) ? (cfg.m_prop / cfg.t_burnout) : 0.0f;
        const float Cdp = (t < cfg.t_burnout)
                        ? mdot * (cfg.L_ne - xCG) * (cfg.L_ne - xCG)
                        : 0.0f;
        const float Cda_base = 0.5f * rho * vmag_reg * cfg.A_ref
                             * cfg.base_cn * (cfg.base_cp - xCG) * (cfg.base_cp - xCG);
        const float Cda_can = 0.5f * rho * vcan_mag * cfg.A_ref
                            * cfg.canard_cn * (cfg.canard_cp - xCG) * (cfg.canard_cp - xCG);
        M[0] -= (Cdp + Cda_base + Cda_can * cfg.canard_dir_2 * cfg.canard_dir_2) * w1;
        M[1] -= (Cdp + Cda_base + Cda_can * cfg.canard_dir_1 * cfg.canard_dir_1) * w2;

        // Fin cant roll forcing (proportional to dynamic pressure)
        M[2] += K_f * q_dyn;

        // Roll aerodynamic damping: M = -0.5 * rho * v * K_d * w3
        M[2] -= 0.5f * rho * vmag * K_d * w3;

        // Canard roll moment (CFD numeric)
        M[2] += canardMoment(vmag, u_zeta);
    }

    // ── Euler rigid-body (I1=I2 for symmetric rocket) ────────────────────────
    const float I2 = I1;
    StateVec xdot;
    xdot(0,0) = ((I2-I3)*w2*w3 + M[0] - I1d*w1) / I1;
    xdot(1,0) = ((I3-I1)*w3*w1 + M[1] - I1d*w2) / I1;
    xdot(2,0) = ((I1-I2)*w1*w2 + M[2] - I3d*w3) / I3;

    // ── Velocity: vdot = F/m - omega×v ───────────────────────────────────────
    xdot(3,0) = F[0]/m - (w2*v3 - w3*v2);
    xdot(4,0) = F[1]/m - (w3*v1 - w1*v3);
    xdot(5,0) = F[2]/m - (w1*v2 - w2*v1);

    // ── Quaternion kinematics: qdot = 0.5 * Omega(w) * q ─────────────────────
    xdot(6,0) = 0.5f*(-w1*qx - w2*qy - w3*qz);
    xdot(7,0) = 0.5f*( w1*qw + w3*qy - w2*qz);
    xdot(8,0) = 0.5f*( w2*qw - w3*qx + w1*qz);
    xdot(9,0) = 0.5f*( w3*qw + w2*qx - w1*qy);

    return xdot;
}

// ─── Numerical Jacobian A = df/dx (central finite differences) ───────────────
StateMat Physics::jacobianA(float t, const StateVec& x, float u, float alt) const {
    static constexpr float EPS = 1e-5f;
    StateMat A;
    for (int j = 0; j < 10; j++) {
        StateVec xp = x, xm = x;
        xp(j,0) += EPS;
        xm(j,0) -= EPS;
        StateVec fp = eom(t, xp, u, alt);
        StateVec fm = eom(t, xm, u, alt);
        for (int i = 0; i < 10; i++)
            A(i,j) = (fp(i,0) - fm(i,0)) / (2.0f * EPS);
    }
    return A;
}

// ─── Analytic B = df/du ───────────────────────────────────────────────────────
// Only roll row (index 2) is nonzero: dw3dot/du = dM_canard/du / I3
InputMat Physics::jacobianB(float t, const StateVec& x) const {
    float va[3];
    airRelativeVelocity(x, va);
    const float vmag = sqrtf(va[0]*va[0]+va[1]*va[1]+va[2]*va[2]);
    const float I3   = getInertiaRoll(t);
    InputMat B;
    B(2,0) = (I3 > 1e-10f) ? (canardMomentJacobian(vmag) / I3) : 0.0f;
    return B;
}

// ─── Measurement Jacobian C = dy/dx (6×10) ───────────────────────────────────
MeasMat Physics::jacobianC(float t, const StateVec& x, float u,
                            const StateMat& A, float alt) const {
    const float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    const float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);
    const WeatherSample wx = weatherAtAltitude(alt);
    const float g_body_mag = wx.g;
    const float accel_scale = cfg.g_0;

    // J_cross = ∂(ω×v)/∂x  [3×10]
    // ω×v = [w2*v3-w3*v2, w3*v1-w1*v3, w1*v2-w2*v1]
    Mat<3,10> Jcross;
    // ∂/∂w1: [0, -v3, v2]
    Jcross(0,0)=0.0f;  Jcross(1,0)=-v3;   Jcross(2,0)= v2;
    // ∂/∂w2: [v3, 0, -v1]
    Jcross(0,1)= v3;   Jcross(1,1)=0.0f;  Jcross(2,1)=-v1;
    // ∂/∂w3: [-v2, v1, 0]
    Jcross(0,2)=-v2;   Jcross(1,2)= v1;   Jcross(2,2)=0.0f;
    // ∂/∂v1: [0, w3, -w2]
    Jcross(0,3)=0.0f;  Jcross(1,3)= w3;   Jcross(2,3)=-w2;
    // ∂/∂v2: [-w3, 0, w1]
    Jcross(0,4)=-w3;   Jcross(1,4)=0.0f;  Jcross(2,4)= w1;
    // ∂/∂v3: [w2, -w1, 0]
    Jcross(0,5)= w2;   Jcross(1,5)=-w1;   Jcross(2,5)=0.0f;
    // ∂/∂q = 0 (columns 6-9 already zero)

    // J_g = ∂(g_body)/∂x  [3×10]
    // g_body = R_BW * [0,0,-g]; only q-columns nonzero
    // R_BW col-2: [2(qx*qz-qw*qy), 2(qy*qz+qw*qx), 1-2(qx²+qy²)]
    Mat<3,10> Jg;
    // ∂g_body/∂qw
    // g_body[0] = -g * 2*(qx*qz - qw*qy)
    // ∂g_body[0]/∂qw = -g * 2*(-qy) = 2*g*qy
    Jg(0,6) = 2.0f*g_body_mag*qy;
    Jg(0,7) = -2.0f*g_body_mag*qz;         // ∂/∂qx = -g*2*qz
    Jg(0,8) = 2.0f*g_body_mag*qw;          // ∂/∂qy = -g*2*(-qw) = 2*g*qw
    Jg(0,9) = -2.0f*g_body_mag*qx;         // ∂/∂qz = -g*2*qx

    // g_body[1] = -g * 2*(qy*qz + qw*qx)
    Jg(1,6) = -2.0f*g_body_mag*qx;         // ∂/∂qw = -g*2*qx
    Jg(1,7) = -2.0f*g_body_mag*qw;         // ∂/∂qx = -g*2*qw
    Jg(1,8) = -2.0f*g_body_mag*qz;         // ∂/∂qy = -g*2*qz
    Jg(1,9) = -2.0f*g_body_mag*qy;         // ∂/∂qz = -g*2*qy

    // g_body[2] = -g * (1 - 2*(qx²+qy²))
    // ∂/∂qw = 0
    Jg(2,7) = 4.0f*g_body_mag*qx;          // ∂/∂qx = -g*(-4*qx) = 4*g*qx
    Jg(2,8) = 4.0f*g_body_mag*qy;          // ∂/∂qy = 4*g*qy

    // Build C matrix
    MeasMat C;

    // Accel rows 0-2: C[0:3,:] = (A[3:6,:] + J_cross - J_g) / accel_scale
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 10; j++) {
            C(i, j) = (A(3+i, j) + Jcross(i, j) - Jg(i, j)) / accel_scale;
        }
    }
    // Zero angular-rate columns on accel rows (accel doesn't directly observe w)
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C(i, j) = 0.0f;

    // Gyro rows 3-5: identity on angular-rate columns
    C(3,0) = 1.0f;
    C(4,1) = 1.0f;
    C(5,2) = 1.0f;

    return C;
}

// ─── Measurement prediction ───────────────────────────────────────────────────
// Returns [accel_g (3), gyro_rad_s (3)]
MeasVec Physics::predictMeasurement(float t, const StateVec& x, float u,
                                    float alt) const {
    const float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    const float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);

    const WeatherSample wx = weatherAtAltitude(alt);
    StateVec xdot = eom(t, x, u, alt);

    // vdot (body) = xdot[3:6]
    // Specific force = vdot + omega×v - g_body, divided by g to get g units
    // omega × v
    float oxv[3] = {
        w2*v3 - w3*v2,
        w3*v1 - w1*v3,
        w1*v2 - w2*v1
    };

    // g_body = R_BW * [0,0,-g]
    float R[3][3];
    Physics::R_BW(qw, qx, qy, qz, R);
    float g_body[3] = {
        R[0][2] * (-wx.g),
        R[1][2] * (-wx.g),
        R[2][2] * (-wx.g)
    };

    MeasVec y;
    for (int i = 0; i < 3; i++)
        y(i, 0) = (xdot(3+i, 0) + oxv[i] - g_body[i]) / cfg.g_0;

    // Gyro: direct angular rate observation
    y(3,0) = w1;
    y(4,0) = w2;
    y(5,0) = w3;

    return y;
}
