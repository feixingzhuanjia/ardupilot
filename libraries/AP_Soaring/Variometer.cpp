/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#include "Variometer.h"

#include <AP_Logger/AP_Logger.h>

Variometer::Variometer(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms) :
    _ahrs(ahrs),
    _aparm(parms),
    new_data(false)
{
}

void Variometer::update(const float polar_K, const float polar_B, const float polar_Cd0)
{
    _ahrs.get_relative_position_D_home(alt);
    alt = -alt;

    if (fabsf(alt - _last_alt) > 0.0001f) { // if no change in altitude then there will be no update of ekf buffer
        // Both filtered total energy rates and unfiltered are computed for the thermal switching logic and the EKF
        float aspd = 0;
        float roll = _ahrs.roll;
        if (!_ahrs.airspeed_estimate(&aspd)) {
            aspd = _aparm.airspeed_cruise_cm / 100.0f;//
        }
        _aspd_filt = ASPD_FILT * aspd + (1 - ASPD_FILT) * _aspd_filt;//经历了一阶低通滤波器的空速
        float total_E = alt + 0.5f *_aspd_filt * _aspd_filt / GRAVITY_MSS;   //总比能 但未对时间进行微分                                               // Work out total energy计算总能量
        float sinkrate = correct_netto_rate(0.0f, (roll + _last_roll) / 2, _aspd_filt, polar_K, polar_Cd0, polar_B);       // Compute still-air sinkrate计算静止空气沉降率
        reading = (total_E - _last_total_E) / ((AP_HAL::micros64() - _prev_update_time) * 1e-6) + sinkrate;    // Unfiltered netto rate对总能量求微分得到能量变化率再加上下沉率    = enet
        filtered_reading = TE_FILT * reading + (1 - TE_FILT) * filtered_reading;                       //一阶低通滤波efiltn=Tc*enet+(1-Tc)*efiltn-1; Apply low pass timeconst filter for noise对噪声应用低通时间常数滤波器
        displayed_reading = TE_FILT_DISPLAYED * reading + (1 - TE_FILT_DISPLAYED) * displayed_reading;


        _last_alt = alt;                                       // Store variables
        _last_roll = roll;
        _last_aspd = aspd;
        _last_total_E = total_E;
        _prev_update_time = AP_HAL::micros64();
        new_data = true;

        AP::logger().Write("VAR", "TimeUS,aspd_raw,aspd_filt,alt,roll,raw,filt", "Qffffff",
                                               AP_HAL::micros64(),
                                               (double)aspd,
                                               (double)_aspd_filt,
                                               (double)alt,
                                               (double)roll,
                                               (double)reading,
                                               (double)filtered_reading);
    }
}


float Variometer::correct_netto_rate(float climb_rate,
                                     float phi,
                                     float aspd,
                                     const float polar_K,
                                     const float polar_CD0,
                                     const float polar_B)
{
    // Remove aircraft sink rate
    float CL0;  // CL0 = 2*W/(rho*S*V^2)     CL
    float C1;   // C1 = CD0/CL
    float C2;   // C2 = B*CL
    float netto_rate;
    float cosphi;
    CL0 = polar_K / (aspd * aspd);
    C1 = polar_CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
    C2 = polar_B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank
    //cosx=cos0-sin0*x-cos0*x*x/2!
    cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)麦克劳林展开式前两项
    netto_rate = climb_rate + aspd * (C1 + C2 / (cosphi * cosphi));  // effect of aircraft drag removed下沉率

    // Remove acceleration effect - needs to be tested.
    //float temp_netto = netto_rate;
    //float dVdt = SpdHgt_Controller->get_VXdot();
    //netto_rate = netto_rate + aspd*dVdt/GRAVITY_MSS;
    //gcs().send_text(MAV_SEVERITY_INFO, "%f %f %f %f",temp_netto,dVdt,netto_rate,barometer.get_altitude());
    return netto_rate;
}
