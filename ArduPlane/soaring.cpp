#include "Plane.h"

#if SOARING_ENABLED == ENABLED

/*
*  ArduSoar support function
*
*  Peter Braswell, Samuel Tabor, Andrey Kolobov, and Iain Guilliard
*/
void Plane::update_soaring() {
    
    if (!g2.soaring_controller.is_active()) {//是否激活
        return;
    }
    
    g2.soaring_controller.update_vario();//通过K，Co，B等更新sink

    // Check for throttle suppression change.检查油门抑制是否发生变化
    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO:
        g2.soaring_controller.suppress_throttle();
        break;
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
        if (!g2.soaring_controller.suppress_throttle()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: forcing RTL");
            set_mode(mode_rtl, ModeReason::SOARING_FBW_B_WITH_MOTOR_RUNNING);
        }
        break;
    case Mode::Number::LOITER:
        // Do nothing. We will switch back to auto/rtl before enabling throttle.
        break;
    default:
        // This does not affect the throttle since suppressed is only checked in the above three modes. 
        // It ensures that the soaring always starts with throttle suppressed though.
        g2.soaring_controller.set_throttle_suppressed(true);//默认设置油门抑制为真
        break;
    }

    // Nothing to do if we are in powered flight如果我们在动力飞行中就没什么可做的了，也就是说正常飞行，油门没被抑制
    if (!g2.soaring_controller.get_throttle_suppressed() && aparm.throttle_max > 0) {
        return;
    }

    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO://根据地面站航点飞行
    case Mode::Number::FLY_BY_WIRE_B://手动操作，各通道输出都有限制
    case Mode::Number::CRUISE://自动控制高度，速度，方向，人为可以通过遥控来改方向和高度
        // Test for switch into thermalling mode自己填
        g2.soaring_controller.update_cruising();

        if (g2.soaring_controller.check_thermal_criteria()) {//判断进入热状态条件
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal detected, entering loiter");
            set_mode(mode_loiter, ModeReason::SOARING_THERMAL_DETECTED);
        }
        break;

    case Mode::Number::LOITER://定点绕圈飞行，虽然进了这个模式但是自身动力是怎么取消的？下一步要了解悬停模式   解答：该模式就是盘旋上升获取高度，在上一模式电机就停止工作了？
        // Update thermal estimate and check for switch back to AUTO
        g2.soaring_controller.update_thermalling();  // Update estimate

        if (g2.soaring_controller.check_cruise_criteria()) {
            // Exit as soon as thermal state estimate deteriorates一旦热状态估计恶化，立即退出
            switch (previous_mode->mode_number()) {
            case Mode::Number::FLY_BY_WIRE_B:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, entering RTL");
                set_mode(mode_rtl, ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
                break;

            case Mode::Number::CRUISE: {
                // return to cruise with old ground course
                CruiseState cruise = cruise_state;
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring CRUISE");
                set_mode(mode_cruise, ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
                cruise_state = cruise;
                set_target_altitude_current();
                break;
            }

            case Mode::Number::AUTO:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring AUTO");
                set_mode(mode_auto, ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
                break;

            default:
                break;
            }
        } else {
            // still in thermal - need to update the wp location如果还在热状态中就更新到下一个航点
            g2.soaring_controller.get_target(next_WP_loc);
        }
        break;
    default:
        // nothing to do
        break;
    }
}

#endif // SOARING_ENABLED
