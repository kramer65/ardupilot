#include "mode.h"
#include "Plane.h"
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Common/Location.h>


bool ModeTetherZenith::_enter()
{
    plane.auto_state.next_wp_crosstrack = false;
    
    plane.prev_WP_loc = plane.current_loc;
    
    Location destination = plane.home;
    destination.alt = plane.current_loc.alt;
    plane.next_WP_loc = destination;

    plane.setup_turn_angle();

    return true;
}

void ModeTetherZenith::update()
{
    plane.calc_nav_roll();
    plane.nav_pitch_cd = 1500;  // pitch 15 degrees up
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
}

void ModeTetherZenith::navigate()
{
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
}
