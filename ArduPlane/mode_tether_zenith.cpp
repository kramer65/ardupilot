#include "mode.h"
#include "Plane.h"
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Common/Location.h>


bool ModeTetherZenith::_enter()
{
    // // Send text to the GCS
    // gcs().send_text(MAV_SEVERITY_INFO, "DIT IS MET DE gcs() INFO");
    // gcs().send_text(MAV_SEVERITY_EMERGENCY, "en DIT is natuurlijk met gcs() EMERGENCY");
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DIT IS MET DE GCS INFO");
    // GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "en DIT is natuurlijk met GCS EMERGENCY");


    // // START CODE COMING FROM ArduPlane/mode_auto.cpp
    // plane.auto_state.vtol_mode = false;

    // plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // // start or resume the mission, based on MIS_AUTORESET
    // plane.mission.start_or_resume();

    // if (hal.util->was_watchdog_armed()) {
    //     if (hal.util->persistent_data.waypoint_num != 0) {
    //         gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
    //         plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
    //         hal.util->persistent_data.waypoint_num = 0;
    //     }
    // }
    // //END CODE COMING FROM ArduPlane/mode_auto.cpp



    // // clear the mission waypoints (tried several things)
    // plane.mission.reset_wp_history();
    // plane.mission.clear();

    // // Set a single new waypoint to a place at the home location but at the current altitude
    // Location destination = plane.home;
    // destination.alt = plane.current_loc.alt;
    // plane.next_WP_loc = destination;
    // plane.mission.start();

    // printf("current location: %d, %d, %d\n", plane.current_loc.lat, plane.current_loc.lng, plane.current_loc.alt);
    // printf("destination: %d, %d, %d\n", destination.lat, destination.lng, destination.alt);

    // // current location: -35.3599089, 149.1611586, 679.05
    // // destination: -35.3632621, 149.1652375, 679.05


    // return true;

    // plane.throttle_allows_nudging = true;
    // plane.auto_throttle_mode = true;
    // plane.auto_navigation_mode = true;
    
    plane.prev_WP_loc = plane.current_loc;
    plane.auto_state.next_wp_crosstrack = false;
    Location destination = plane.home;
    destination.alt = plane.current_loc.alt;
    plane.next_WP_loc = destination;


    return true;
}

void ModeTetherZenith::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();


    // // calculate the desired heading towards home
    // int32_t target_bearing_cd = plane.current_loc.get_bearing_to(plane.next_WP_loc);

    // // altitude is to maintain the current altitude
    // float target_altitude_cm = plane.current_loc.alt;

    // // update navigation and throttle
    // plane.nav_controller->update_heading_hold(target_bearing_cd);
    // plane.altitude_error_cm = plane.current_loc.alt - target_altitude_cm;

    // // force plane to navigate towards the target
    // plane.auto_state.wp_proportion = 1;
    // plane.auto_state.wp_distance = plane.current_loc.get_distance(plane.next_WP_loc);


    // set target altitude and adjust throttle to maintain altitude
    // plane.next_WP_loc.alt = target_altitude_cm;
    // plane.TECS_controller.set_altitude_target()

    // SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    // plane.nav_roll_cd = 0;
    // plane.nav_pitch_cd = 1500;  // pitch 15 degrees up
    // TODO: calculate the desired pitch above from the height and the tether length. The higher up, the lower the pitch towards horizontal
    
    // int32_t home_bearing;
    // home_bearing = plane.current_loc.get_bearing_to(plane.home);
    // printf("needed bearing: %d\n", home_bearing);

    // Set the new waypoint to a place at the current altitude, but at the home bearing
    // Location destination = plane.home;
    // destination.alt = plane.current_loc.alt;
    // plane.next_WP_loc = destination;


    // // Print "Hello world" every 50th time
    // static int counter = 0;
    // counter++;
    // if (counter % 50 == 0) {
    //     printf("Hello world %f\n", float(counter));
    // }


    // // Use L1 controller to point to next waypoint (home)
    // plane.calc_nav_roll();
    // plane.calc_nav_pitch();
    // plane.calc_throttle();
    // // printf("mission cmd id: %d    -   mission state id: %d\n", plane.mission.get_current_nav_cmd().id, plane.mission.state());
    // // prints out "mission cmd id: 22    -   mission state id: 0"


    
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

void ModeTetherZenith::navigate()
{
    
    if (!plane.auto_state.checked_for_autoland) {
        if ((plane.g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START) ||
            (plane.g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START &&
            plane.reached_loiter_target() && 
            labs(plane.altitude_error_cm) < 1000))
            {
                // we've reached the RTL point, see if we have a landing sequence
                if (plane.mission.jump_to_landing_sequence()) {
                    // switch from RTL -> AUTO
                    plane.mission.set_force_resume(true);
                    if (plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND)) {
                        // return here so we don't change the radius and don't run the rtl update_loiter()
                        return;
                    }
                }

                // prevent running the expensive jump_to_landing_sequence
                // on every loop
                plane.auto_state.checked_for_autoland = true;
            }
    }

    uint16_t radius = abs(plane.g.rtl_radius);
    if (radius > 0) {
        plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
    }

    plane.update_loiter(radius);
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
