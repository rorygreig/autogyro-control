// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if(next_WP.lat == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    wp_distance = get_distance(&current_loc, &next_WP);

    if (wp_distance < 0) {
        gcs_send_text_P(SEVERITY_HIGH,PSTR("WP error - distance < 0"));
        //Serial.println(wp_distance,DEC);
        return;
    }

    // target_bearing is where we should be heading
    // --------------------------------------------
    target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);

    // nav_bearing will includes xtrac correction
    // ------------------------------------------
    nav_bearing_cd = target_bearing_cd;

    // check if we have missed the WP
    loiter_delta = (target_bearing_cd - old_target_bearing_cd)/100;

    // reset the old value
    old_target_bearing_cd = target_bearing_cd;

    // wrap values
    if (loiter_delta > 180) loiter_delta -= 360;
    if (loiter_delta < -180) loiter_delta += 360;
    loiter_sum += abs(loiter_delta);

    // control mode specific updates to nav_bearing
    // --------------------------------------------
    update_navigation();
}


#if 0
// Disabled for now
void calc_distance_error()
{
    distance_estimate       += (float)g_gps->ground_speed * .0002 * cos(radians(bearing_error_cd * .01));
    distance_estimate       -= DST_EST_GAIN * (float)(distance_estimate - GPS_wp_distance);
    wp_distance             = max(distance_estimate,10);
}
#endif

static void calc_gndspeed_undershoot()
{
    // Function is overkill, but here in case we want to add filtering
    // later
    if (g_gps && g_gps->status() == GPS::GPS_OK) {
        groundspeed_undershoot = (g.min_gndspeed_cm > 0) ? (g.min_gndspeed_cm - g_gps->ground_speed) : 0;
    }
}

static void calc_bearing_error()
{
    bearing_error_cd = nav_bearing_cd - ahrs.yaw_sensor;
    bearing_error_cd = wrap_180_cd(bearing_error_cd);
}

static void calc_altitude_error()
{	
	target_altitude_cm = next_WP.alt;

	//Rory's code
	altitude_error_cm = target_altitude_cm - current_loc.alt;

}

//Rory's function to calculate climb rate error
static void calc_climb_rate_error()
{
	target_climb_rate = constrain(altitude_error_cm, -125, 125);  //limit climb rate for large altitude errors - RMG

	climb_rate_error = 0.15*target_climb_rate - 0.15*current_climb_rate;

}

static void calc_airspeed_errors()
{
    //The airspeed error should also depend on whether the plane is ascending, descending or gliding - RMG

    float aspeed_cm = airspeed.get_airspeed_cm();

    // Normal airspeed target - should change depending on scenario
    target_airspeed_cm = g.airspeed_cruise_cm;
	
    //Rory's code
    airspeed_error_cm = target_airspeed_cm - aspeed_cm;

}

static int32_t wrap_360_cd(int32_t error)
{
    if (error > 36000) error -= 36000;
    if (error < 0) error += 36000;
    return error;
}

static int32_t wrap_180_cd(int32_t error)
{
    if (error > 18000) error -= 36000;
    if (error < -18000) error += 36000;
    return error;
}

static void update_loiter()
{
    float power;

    if(wp_distance <= g.loiter_radius) {
        power = float(wp_distance) / float(g.loiter_radius);
        power = constrain(power, 0.5, 1);
        nav_bearing_cd += 9000.0 * (2.0 + power);
    } else if(wp_distance < (g.loiter_radius + LOITER_RANGE)) {
        power = -((float)(wp_distance - g.loiter_radius - LOITER_RANGE) / LOITER_RANGE);
        power = constrain(power, 0.5, 1);                               //power = constrain(power, 0, 1);
        nav_bearing_cd -= power * 9000;
    } else{
        update_crosstrack();
        loiter_time_ms = millis();                              // keep start time for loiter updating till we get within LOITER_RANGE of orbit

    }
    nav_bearing_cd = wrap_360_cd(nav_bearing_cd);
}

static void update_crosstrack(void)
{
    // if we are using a compass for navigation, then adjust the
    // heading to account for wind
    if (g.crosstrack_use_wind && compass.use_for_yaw()) {
        Vector3f wind = ahrs.wind_estimate();
        Vector2f wind2d = Vector2f(wind.x, wind.y);
        float speed;
        if (ahrs.airspeed_estimate(&speed)) {
            Vector2f nav_vector = Vector2f(cos(radians(nav_bearing_cd*0.01)), sin(radians(nav_bearing_cd*0.01))) * speed;
            Vector2f nav_adjusted = nav_vector - wind2d;
            nav_bearing_cd = degrees(atan2(nav_adjusted.y, nav_adjusted.x)) * 100;
        }
    }

    // Crosstrack Error
    // ----------------
    // If we are too far off or too close we don't do track following
    if (wp_totalDistance >= g.crosstrack_min_distance && 
        abs(wrap_180_cd(target_bearing_cd - crosstrack_bearing_cd)) < 4500) {
        // Meters we are off track line
        crosstrack_error = sin(radians((target_bearing_cd - crosstrack_bearing_cd) * 0.01)) * wp_distance;               
        nav_bearing_cd += constrain(crosstrack_error * g.crosstrack_gain, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
        nav_bearing_cd = wrap_360_cd(nav_bearing_cd);
    }

}

static void reset_crosstrack()
{
    crosstrack_bearing_cd   = get_bearing_cd(&prev_WP, &next_WP);       // Used for track following
}

