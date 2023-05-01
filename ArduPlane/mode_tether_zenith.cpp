#include "mode.h"
#include "Plane.h"
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <SRV_Channel/SRV_Channel.h>

void ModeTetherZenith::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 1500;  // pitch 15 degrees up
    // TODO: calculate the desired pitch above from the height and the tether length. The higher up, the lower the pitch towards horizontal
    
    int32_t home_bearing;
    home_bearing = plane.current_loc.get_bearing_to(plane.home);
    printf("needed bearing: %d\n", home_bearing);
    // TODO: Make the rudder influence the needed bearing

    // In order to point to home, which variable do I need to set?
    // - calc_nav_yaw_coordinated();
    // - calc_nav_yaw_course(void);
    // - calc_nav_yaw_ground(void);
    
    // Benodigdheden
    // plane.current_loc  // huidige locatie
    // plane.home  // home locatie
    // get_bearing_cd()  // richting om vanaf huidige locatie home locatie te bereiken
    // wp_bearing = wrap_180_cd(9000);  // zet de waypoint bearing
    // wp_bearing = wrap_180_cd(get_bearing_cd(plane.current_loc, plane.home));
}


// bool ModeTetherZenith::init(bool ignore_checks)
// {
//     if (!ignore_checks) {
//         if (!AP::ahrs().home_is_set()) {
//             return false;
//         }
//     }

//     return true;
// }
