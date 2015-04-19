//Board = Arduino Mega (ATmega1280)
#define ARDUINO 101
#define __AVR_ATmega1280__
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
#define __attribute__(x)
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__
#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {}

//already defined in arduno.h
//already defined in arduno.h
static void fast_loop();
static void medium_loop();
static void slow_loop();
static void one_second_loop();
static void update_GPS(void);
static void update_current_flight_mode(void);
static void update_navigation();
static void update_alt();
static float calc_gps_climb(int32_t new_alt);
static float get_speed_scaler(void);
static bool stick_mixing_enabled(void);
static void stabilize();
static void crash_checker();
static void calc_throttle();
static void calc_nav_pitch();
static void calc_nav_yaw(float speed_scaler, float ch4_inf);
static void calc_nav_roll();
static void throttle_slew_limit();
static bool suppress_throttle(void);
static void set_servos(void);
static void demo_servos(byte i);
static bool alt_control_airspeed(void);
static NOINLINE void send_heartbeat(mavlink_channel_t chan);
static NOINLINE void send_attitude(mavlink_channel_t chan);
static NOINLINE void send_fence_status(mavlink_channel_t chan);
static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops);
static void NOINLINE send_meminfo(mavlink_channel_t chan);
static void NOINLINE send_location(mavlink_channel_t chan);
static void NOINLINE send_nav_controller_output(mavlink_channel_t chan);
static void NOINLINE send_gps_raw(mavlink_channel_t chan);
static void NOINLINE send_servo_out(mavlink_channel_t chan);
static void NOINLINE send_radio_in(mavlink_channel_t chan);
static void NOINLINE send_radio_out(mavlink_channel_t chan);
static void NOINLINE send_vfr_hud(mavlink_channel_t chan);
static void NOINLINE send_raw_imu1(mavlink_channel_t chan);
static void NOINLINE send_raw_imu2(mavlink_channel_t chan);
static void NOINLINE send_raw_imu3(mavlink_channel_t chan);
static void NOINLINE send_ahrs(mavlink_channel_t chan);
static void NOINLINE send_simstate(mavlink_channel_t chan);
static void NOINLINE send_hwstatus(mavlink_channel_t chan);
static void NOINLINE send_wind(mavlink_channel_t chan);
static void NOINLINE send_current_waypoint(mavlink_channel_t chan);
static void NOINLINE send_statustext(mavlink_channel_t chan);
static bool telemetry_delayed(mavlink_channel_t chan);
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str);
static void mavlink_delay(unsigned long t);
static void gcs_send_message(enum ap_message id);
static void gcs_data_stream_send(void);
static void gcs_update(void);
static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);
static bool print_log_menu(void);
static void do_erase_logs(void);
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw);
static void Log_Write_Performance();
static void Log_Write_Cmd(byte num, struct Location *wp);
static void Log_Write_Startup(byte type);
static void Log_Write_Control_Tuning();
static void Log_Write_Nav_Tuning();
static void Log_Write_Mode(byte mode);
static void Log_Write_GPS(      int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,                                 int32_t log_Ground_Speed, int32_t log_Ground_Course, byte log_Fix, byte log_NumSats);
static void Log_Write_Raw();
static void Log_Write_Current();
static void Log_Read_Current();
static void Log_Read_Control_Tuning();
static void Log_Read_Nav_Tuning();
static void Log_Read_Performance();
static void Log_Read_Cmd();
static void Log_Read_Startup();
static void Log_Read_Attitude();
static void Log_Read_Mode();
static void Log_Read_GPS();
static void Log_Read_Raw();
static void Log_Read(int16_t start_page, int16_t end_page);
static int16_t Log_Read_Process(int16_t start_page, int16_t end_page);
static void Log_Write_Mode(byte mode);
static void Log_Write_Startup(byte type);
static void Log_Write_Cmd(byte num, struct Location *wp);
static void Log_Write_Current();
static void Log_Write_Nav_Tuning();
static void Log_Write_GPS(      int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,                                 int32_t log_Ground_Speed, int32_t log_Ground_Course, byte log_Fix, byte log_NumSats);
static void Log_Write_Performance();
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw);
static void Log_Write_Control_Tuning();
static void Log_Write_Raw();
static void load_parameters(void);
void add_altitude_data(unsigned long xl, long y);
static void init_commands();
static void update_auto();
static void reload_commands_airstart();
static struct Location get_cmd_with_index_raw(int16_t i);
static struct Location get_cmd_with_index(int16_t i);
static void set_cmd_with_index(struct Location temp, int16_t i);
static void decrement_cmd_index();
static int32_t read_alt_to_hold();
static void set_next_WP(struct Location *wp);
static void set_guided_WP(void);
void init_home();
static void handle_process_nav_cmd();
static void handle_process_condition_command();
static void handle_process_do_command();
static void handle_no_commands();
static bool verify_nav_command();
static bool verify_condition_command();
static void do_RTL(void);
static void do_takeoff();
static void do_nav_wp();
static void do_land();
static void do_loiter_unlimited();
static void do_loiter_turns();
static void do_loiter_time();
static bool verify_takeoff();
static bool verify_land();
static bool verify_nav_wp();
static bool verify_loiter_unlim();
static bool verify_loiter_time();
static bool verify_loiter_turns();
static bool verify_RTL();
static void do_wait_delay();
static void do_change_alt();
static void do_within_distance();
static bool verify_wait_delay();
static bool verify_change_alt();
static bool verify_within_distance();
static void do_loiter_at_location();
static void do_jump();
static void do_change_speed();
static void do_set_home();
static void do_set_servo();
static void do_set_relay();
static void do_repeat_servo(uint8_t channel, uint16_t servo_value,                             int16_t repeat, uint8_t delay_time);
static void do_repeat_relay();
void change_command(uint8_t cmd_index);
static void update_commands(void);
static void verify_commands(void);
static void process_next_command();
static void process_non_nav_command();
static void read_control_switch();
static byte readSwitch(void);
static void reset_control_switch();
static void failsafe_short_on_event(int16_t fstype);
static void failsafe_long_on_event(int16_t fstype);
static void failsafe_short_off_event();
void low_battery_event(void);
static void update_events(void);
void failsafe_check(uint32_t tnow);
static Vector2l get_fence_point_with_index(unsigned i);
static void set_fence_point_with_index(Vector2l &point, unsigned i);
static void geofence_load(void);
static bool geofence_enabled(void);
static bool geofence_check_minalt(void);
static bool geofence_check_maxalt(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static void geofence_send_status(mavlink_channel_t chan);
bool geofence_breached(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static bool geofence_enabled(void);
static void navigate();
void calc_distance_error();
static void calc_gndspeed_undershoot();
static void calc_bearing_error();
static void calc_altitude_error();
static void calc_climb_rate_error();
static void calc_airspeed_errors();
static int32_t wrap_360_cd(int32_t error);
static int32_t wrap_180_cd(int32_t error);
static void update_loiter();
static void update_crosstrack(void);
static void reset_crosstrack();
static void init_rc_in();
static void init_rc_out();
static void read_radio();
static void control_failsafe(uint16_t pwm);
static void trim_control_surfaces();
static void trim_radio();
static void init_barometer(void);
static int32_t read_barometer(void);
static void read_airspeed(void);
static void zero_airspeed(void);
static void read_battery(void);
void read_receiver_rssi(void);
static int32_t adjusted_altitude_cm(void);
static void report_batt_monitor();
static void report_radio();
static void report_gains();
static void report_xtrack();
static void report_throttle();
static void report_imu();
static void report_compass();
static void report_flight_modes();
static void print_PID(PID * pid);
static void print_radio_values();
static void print_switch(byte p, byte m);
static void print_done();
static void print_blanks(int16_t num);
static void print_divider(void);
static int8_t radio_input_switch(void);
static void zero_eeprom(void);
static void print_enabled(bool b);
static void print_accel_offsets(void);
static void print_gyro_offsets(void);
static void run_cli(void);
static void init_ardupilot();
static void startup_ground(void);
static void set_mode(byte mode);
static void check_long_failsafe();
static void check_short_failsafe();
static void startup_IMU_ground(bool force_accel_level);
static void update_GPS_light(void);
static void resetPerfData(void);
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud);
static void check_usb_mux(void);
void flash_leds(bool on);
uint16_t board_voltage(void);
static void reboot_apm(void);
static void print_flight_mode(uint8_t mode);
static void print_comma(void);
static void print_hit_enter();
static void test_wp_print(struct Location *cmd, byte wp_index);

#include "C:\Users\Rory\Documents\Engineering\L4 Project\arduino-1.0.1\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Users\Rory\Documents\Engineering\L4 Project\arduino-1.0.1\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\ArduPlane.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\APM_Config.h"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\Attitude.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\GCS.h"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\GCS_Mavlink.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\Log.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\Parameters.h"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\Parameters.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\climb_rate.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\commands.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\commands_logic.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\commands_process.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\config.h"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\control_modes.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\defines.h"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\events.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\failsafe.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\geofence.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\navigation.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\planner.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\radio.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\sensors.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\setup.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\system.ino"
#include "C:\Users\Rory\Documents\Engineering\L4 Project\Autogyro Sketchbook\ArduPlane\test.ino"
