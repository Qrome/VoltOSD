/*
 *  VoldOSD, the little brother to QLiteOSD, is an simple OSD for DJI FPV System:
 *  This is an Arduino project that feelds Flight Pack Voltage to DJI FPV Goggles
 *
 * ------------------------------------------------
 *
 * Copyright (C) 2022 David Payne
 * 
 * This software is based on and uses software published by Paul Kurucz (pkuruz):opentelem_to_bst_bridge
 * as well as software d3ngit : djihdfpv_mavlink_to_msp_V2 and crashsalot : VOT_to_DJIFPV
 * 
 * License info: See the LICENSE file at the repo top level
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */

/* 
 *  VoltOSD
 *
 *  XIAO SAMD21 TX to DJI Air unit RX(115200)
 *  Voltage sensor on A0 pin 0
 *  PWM Arm Signal on D1 pin 1
 */


/* ----------------------------------------------------- */

#include <MSP.h>
#include "MSP_OSD.h"
#include "OSD_positions_config.h"

#define VERSION "1.0"
#define MAH_CALIBRATION_FACTOR 1.0f   //used to calibrate mAh reading.
#define SPEED_IN_KILOMETERS_PER_HOUR  //if commented out defaults to m/s
#define IMPERIAL_UNITS                //Altitude in feet, distance to home in miles.
#define FC_FIRMWARE_NAME "Betaflight"
#define FC_FIRMWARE_IDENTIFIER "BTFL"


HardwareSerial &mspSerial = Serial1;
MSP msp;

static const int pwm_arm_pin = 1;
static int triggerValue = 1600;

boolean lightOn = true;

//Voltage and Battery Reading
const float arduinoVCC = 3.3;  //Measured SAMD21 3.3 pin voltage on A0

float ValueR1 = 7500;   //7.5K Resistor
float ValueR2 = 30000;  //30K Resistor
const int alanogPin = A0;
float averageVoltage = 0;
int sampleVoltageCount = 0;
int lastCount = 0;

//Other
char fcVariant[5] = "BTFL";
char craftname[15] = "VoltOSD";
uint32_t previousMillis_MSP = 0;
const uint32_t next_interval_MSP = 100;
uint32_t custom_mode = 0;  //flight mode
uint8_t vbat = 0;
float airspeed = 0;
int16_t groundspeed = 0;
int32_t relative_alt = 0;   // in milimeters
uint32_t altitude_msp = 0;  // EstimatedAltitudeCm
uint16_t rssi = 0;
uint8_t battery_remaining = 0;
uint32_t flightModeFlags = 0x00000002;
int16_t amperage = 0;
uint16_t mAhDrawn = 0;
float f_mAhDrawn = 0.0;
uint8_t numSat = 0;
uint8_t pid_roll[3];
uint8_t pid_pitch[3];
uint8_t pid_yaw[3];
int32_t gps_lon = 0;
int32_t gps_lat = 0;
int32_t gps_alt = 0;
double gps_home_lon = 0;
double gps_home_lat = 0;
int32_t gps_home_alt = 0;
int16_t roll_angle = 0;
int16_t pitch_angle = 0;
uint32_t distanceToHome = 0;  // distance to home in meters
int16_t directionToHome = 0;  // direction to home in degrees
uint8_t fix_type = 0;         // < 0-1: no fix, 2: 2D fix, 3: 3D fix
uint8_t batteryCellCount = 3;
uint16_t batteryCapacity = 2200;
uint8_t legacyBatteryVoltage = 0;
uint8_t batteryState = 0;  // voltage color 0==white, 1==red
uint16_t batteryVoltage = 0;
int16_t heading = 0;
float dt = 0;
#ifdef MAH_CALIBRATION_FACTOR
float mAh_calib_factor = MAH_CALIBRATION_FACTOR;
#else
float mAh_calib_factor = 1;
#endif
uint8_t set_home = 1;
uint32_t general_counter = next_interval_MSP;
uint16_t blink_sats_orig_pos = osd_gps_sats_pos;
uint16_t blink_sats_blank_pos = 234;
uint32_t previousFlightMode = custom_mode;
uint8_t srtCounter = 1;
uint8_t thr_position = 0;
float wind_direction = 0;  // wind direction (degrees)
float wind_speed = 0;      // wind speed in ground plane (m/s)
float relative_wind_direction = 0;
float climb_rate = 0;

msp_battery_state_t battery_state = { 0 };
msp_name_t name = { 0 };
msp_fc_version_t fc_version = { 0 };
msp_fc_variant_t fc_variant = { 0 };
//msp_status_BF_t status_BF = {0};
msp_status_DJI_t status_DJI = { 0 };
msp_analog_t analog = { 0 };
msp_raw_gps_t raw_gps = { 0 };
msp_comp_gps_t comp_gps = { 0 };
msp_attitude_t attitude = { 0 };
msp_altitude_t altitude = { 0 };


/* ----------------------------------------------------- */
void setup() {
#ifdef DEBUG
  SerialUSB.begin(115200);
  while (!SerialUSB);
#endif
  Serial1.begin(115200);
  while (!Serial1);

  analogReadResolution(12); // SAMD21 12 bit resolution 0 - 4096 range on Analog pin

  msp.begin(mspSerial);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(pwm_arm_pin, INPUT_PULLUP);

  delay(1000);

  status_DJI.cycleTime = 0x0080;
  status_DJI.i2cErrorCounter = 0;
  status_DJI.sensor = 0x23;
  status_DJI.configProfileIndex = 0;
  status_DJI.averageSystemLoadPercent = 7;
  status_DJI.accCalibrationAxisFlags = 0;
  status_DJI.DJI_ARMING_DISABLE_FLAGS_COUNT = 20;
  status_DJI.djiPackArmingDisabledFlags = (1 << 24);
  flightModeFlags = 0x00000002;
}

msp_osd_config_t msp_osd_config = { 0 };

void send_osd_config() {

#ifdef IMPERIAL_UNITS
  msp_osd_config.units = 0;
#else
  msp_osd_config.units = 1;
#endif

  msp_osd_config.osd_item_count = 56;
  msp_osd_config.osd_stat_count = 24;
  msp_osd_config.osd_timer_count = 2;
  msp_osd_config.osd_warning_count = 16;  // 16
  msp_osd_config.osd_profile_count = 1;   // 1
  msp_osd_config.osdprofileindex = 1;     // 1
  msp_osd_config.overlay_radio_mode = 0;  //  0

  msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
  msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
  msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
  msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
  msp_osd_config.osd_flymode_pos = osd_flymode_pos;
  msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
  msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
  msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  msp_osd_config.osd_altitude_pos = osd_altitude_pos;
  msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  msp_osd_config.osd_power_pos = osd_power_pos;
  msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  msp_osd_config.osd_warnings_pos = osd_warnings_pos;
  msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  msp_osd_config.osd_debug_pos = osd_debug_pos;
  msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
  msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
  msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
  msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
  msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
  msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
  msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
  msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
  msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
  msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
  msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
  msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
  msp_osd_config.osd_g_force_pos = osd_g_force_pos;
  msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
  msp_osd_config.osd_log_status_pos = osd_log_status_pos;
  msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
  msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
  msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
  msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
  msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
  msp_osd_config.osd_display_name_pos = osd_display_name_pos;
  msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
  msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

  msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}

void invert_pos(uint16_t *pos1, uint16_t *pos2) {
  uint16_t tmp_pos = *pos1;
  *pos1 = *pos2;
  *pos2 = tmp_pos;
}

void set_flight_mode_flags() {
    //USE PWM signal to ARM
    volatile int pwmValue = readChannel(pwm_arm_pin, 1000, 2000, 0);
    if ((flightModeFlags == 0x00000002) && pwmValue >= triggerValue) {
      flightModeFlags = 0x00000003;    // armed to start recording
    } else if ((flightModeFlags == 0x00000003) && pwmValue < triggerValue && general_counter % 3000 == 0) {        
      flightModeFlags = 0x00000002;    // disarm after 3 second delay
    }
}

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void display_flight_mode() {
  show_text(&craftname);
}

void send_msp_to_airunit() {
  
  //MSP_FC_VARIANT
  memcpy(fc_variant.flightControlIdentifier, fcVariant, sizeof(fcVariant));
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));

  //MSP_FC_VERSION
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 1;
  fc_version.versionPatchLevel = 1;
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));

  //MSP_NAME
  memcpy(name.craft_name, craftname, sizeof(craftname));
  msp.send(MSP_NAME, &name, sizeof(name));

  //MSP_STATUS
  status_DJI.flightModeFlags = flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));

  //MSP_ANALOG
  analog.vbat = vbat;
  analog.rssi = rssi;
  analog.amperage = amperage;
  analog.mAhDrawn = mAhDrawn;
  msp.send(MSP_ANALOG, &analog, sizeof(analog));

  //MSP_BATTERY_STATE
  battery_state.amperage = amperage;
  battery_state.batteryVoltage = vbat * 10;
  battery_state.mAhDrawn = mAhDrawn;
  battery_state.batteryCellCount = batteryCellCount;
  battery_state.batteryCapacity = batteryCapacity;
  battery_state.batteryState = batteryState;
  battery_state.legacyBatteryVoltage = vbat;
  msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

  //MSP_RAW_GPS
  raw_gps.lat = gps_lat;
  raw_gps.lon = gps_lon;
  raw_gps.numSat = numSat;
  raw_gps.alt = gps_alt;
  raw_gps.groundSpeed = groundspeed;  //in cm/s
  msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));

  //MSP_COMP_GPS
  comp_gps.distanceToHome = (int16_t)distanceToHome;
  comp_gps.directionToHome = directionToHome - heading;
  msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));

  //MSP_ATTITUDE
  attitude.pitch = pitch_angle * 10;
  attitude.roll = roll_angle * 10;
  msp.send(MSP_ATTITUDE, &attitude, sizeof(attitude));

  //MSP_ALTITUDE
  altitude.estimatedActualPosition = relative_alt;
  altitude.estimatedActualVelocity = (int16_t)(climb_rate);  //m/s to cm/s
  msp.send(MSP_ALTITUDE, &altitude, sizeof(altitude));

  //MSP_OSD_CONFIG
  send_osd_config();
}


void blink_sats() {
  if (general_counter % 900 == 0 && set_home == 1 && blink_sats_orig_pos > 2000) {
    invert_pos(&osd_gps_sats_pos, &blink_sats_blank_pos); 
  } else if (set_home == 0) {
    osd_gps_sats_pos = blink_sats_orig_pos;
  }
}

void show_text(char (*text)[15]) {
  memcpy(craftname, *text, sizeof(craftname));
}

void set_battery_cells_number() {
  if (vbat < 43) batteryCellCount = 1;
  else if (vbat < 85) batteryCellCount = 2;
  else if (vbat < 127) batteryCellCount = 3;
  else if (vbat < 169) batteryCellCount = 4;
  else if (vbat < 211) batteryCellCount = 5;
  else if (vbat < 255) batteryCellCount = 6;
}


void readVoltage() {
  int readValue = analogRead(alanogPin);
  averageVoltage += (readValue * (arduinoVCC / 4096)) * (1 + (ValueR2 / ValueR1));
  //4096
  sampleVoltageCount++;
}

void getVoltageSample() {
  vbat = (int)((averageVoltage / sampleVoltageCount) * 10);
  lastCount = sampleVoltageCount;
  sampleVoltageCount = 0;
  averageVoltage = 0;
}


void loop() {

  uint32_t currentMillis_MSP = millis();

  readVoltage();

  if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
    previousMillis_MSP = currentMillis_MSP;

    if (general_counter % 300 == 0) {  // update the altitude and voltage values every 300ms
      getVoltageSample();
      if (lightOn) {
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      lightOn = !lightOn;
    }
    set_flight_mode_flags();
    blink_sats();

#ifdef DEBUG
    debugPrint();
#endif
    send_msp_to_airunit();
    general_counter += next_interval_MSP;
  }
  if (custom_mode != previousFlightMode) {
    previousFlightMode = custom_mode;
    display_flight_mode();
  }

  if (batteryCellCount == 0 && vbat > 0) {
    set_battery_cells_number();
  }

  //display flight mode every 10s
  if (general_counter % 10000 == 0) {
    display_flight_mode();
  }
}

//*** USED ONLY FOR DEBUG ***
void debugPrint() {
  SerialUSB.println("**********************************");
  SerialUSB.print("Flight Mode: ");
  SerialUSB.println(flightModeFlags);
  SerialUSB.print("Voltage: ");
  SerialUSB.println(((double)vbat / 10), 1);
  SerialUSB.print("Sample Count / transmit: ");
  SerialUSB.println(lastCount);
  SerialUSB.print("Battery Cell Count: ");
  SerialUSB.println(batteryCellCount);
}

