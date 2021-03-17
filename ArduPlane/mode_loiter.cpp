#include "mode.h"
#include "Plane.h"

bool ModeLoiter::_enter()
{
    plane.throttle_allows_nudging = true;//运行依靠空速来混合RC输入和油门
    plane.auto_throttle_mode = true;//需要运行速度高度控制器，（对下面更新pitch角作铺垫）
    plane.auto_navigation_mode = true;//需要运行自动导航控制器，（对下方更新roll角作铺垫）
    plane.do_loiter_at_location();

#if SOARING_ENABLED == ENABLED
    if (plane.g2.soaring_controller.is_active() && plane.g2.soaring_controller.suppress_throttle()) {
        plane.g2.soaring_controller.init_thermalling();
        plane.g2.soaring_controller.get_target(plane.next_WP_loc); // ahead on flight path
    }
#endif

    return true;
}

void ModeLoiter::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

