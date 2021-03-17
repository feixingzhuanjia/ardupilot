#include "mode.h"
#include "Plane.h"

bool ModeLoiter::_enter()
{
    plane.throttle_allows_nudging = true;//�����������������RC���������
    plane.auto_throttle_mode = true;//��Ҫ�����ٶȸ߶ȿ������������������pitch�����̵棩
    plane.auto_navigation_mode = true;//��Ҫ�����Զ������������������·�����roll�����̵棩
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

