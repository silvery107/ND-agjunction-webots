#include <webots/vehicle/driver.h>
#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/device.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/keyboard.h>
// Add the include directives corresponding to the Robot

// To be used as array indices
enum
{
  X,
  Y,
  Z
};

// ------------Initialize devices------------

// GPS
WbDeviceTag gps;
double gps_coords[3];
double gps_speed = 0.0;

// Inertial unit
WbDeviceTag inertial_unit;
double inertial_unit_coords[3];

// Gyro
WbDeviceTag gyro;
double gyro_coords[3];

// Accelerometer
WbDeviceTag accelerometer;
double accelerometer_coords[3];

// Global variables
int time_step = -1;
double speed = 0.0;
double throttle = 0.0;
double steering_angle = 0.0;
int pre_key = 0;
int g = 1;

// ------------Function------------

void print_help();
void check_keyboard();
void update_gps_speed();
void update_imu_angle();
void update_imu_acceleration();
void update_imu_angular_velocity();
int data_export(FILE *p);
void sensor_init();

void print_help()
{
  printf("You can drive this vehicle!\n");
  printf("Select the 3D window and then use the cursor keys to:\n");
  printf("[LEFT]/[RIGHT] - steer\n");
  printf("[UP]/[DOWN]    - throttle\n");
  printf("[1]-[6]        - change the gear ratio\n");
  printf("[S]            - see the speed of the car\n");
  printf("The gear ratio right now is: %d\n", g);
}

void check_keyboard()
{
  if (pre_key == 0)
  {
    int key = wb_keyboard_get_key();
    pre_key = key > 0 ? key : 0;
  }
  else
  {
    int pre_key_state = wb_keyboard_get_key();
    if (pre_key_state != pre_key)
    {
      switch (pre_key)
      {
      case ' ':
        wbu_driver_set_throttle(0);
        wbu_driver_set_brake_intensity(1);
        break;
      case WB_KEYBOARD_UP:
        wbu_driver_set_brake_intensity(0);
        throttle += 0.1;
        throttle = (throttle > 1) ? 1 : throttle;
        wbu_driver_set_throttle(throttle);
        printf("throttle: %f\n", throttle);
        break;
      case WB_KEYBOARD_DOWN:
        wbu_driver_set_brake_intensity(0);
        throttle -= 0.1;
        throttle = (throttle < 0) ? 0 : throttle;
        wbu_driver_set_throttle(throttle);
        printf("throttle: %f\n", throttle);
        break;
      case WB_KEYBOARD_RIGHT:
        wbu_driver_set_brake_intensity(0);
        steering_angle += 0.05;
        steering_angle = (steering_angle > 0.45) ? 0.45 : steering_angle;
        wbu_driver_set_steering_angle(steering_angle);
        printf("Steering: %f / 9\n", steering_angle * 20);
        break;
      case WB_KEYBOARD_LEFT:
        wbu_driver_set_brake_intensity(0);
        steering_angle -= 0.05;
        steering_angle = (steering_angle < -0.45) ? -0.45 : steering_angle;
        wbu_driver_set_steering_angle(steering_angle);
        printf("Steering: %f / 9\n", steering_angle * 20);
        break;
      case '0':
        g = -1;
        printf("The gear ratio right now is: %d\n", g);
        wbu_driver_set_gear(g);
        break;
      case '1':
        g = 1;
        printf("The gear ratio right now is: %d\n", g);
        wbu_driver_set_gear(g);
        break;
      case '2':
        g = 2;
        printf("The gear ratio right now is: %d\n", g);
        wbu_driver_set_gear(g);
        break;
      case '3':
        g = 3;
        printf("The gear ratio right now is: %d\n", g);
        wbu_driver_set_gear(g);
        break;
      case '4':
        g = 4;
        printf("The gear ratio right now is: %d\n", g);
        wbu_driver_set_gear(g);
        break;
      case '5':
        g = 5;
        printf("The gear ratio right now is: %d\n", g);
        wbu_driver_set_gear(g);
        break;
      case '6':
        g = 6;
        printf("The gear ratio right now is: %d\n", g);
        wbu_driver_set_gear(g);
        break;
      case 'S':
        printf("The gps speed of the car: %f\n", gps_speed);
        break;
      }
    }
    pre_key = pre_key_state;
  }
}

void update_gps_speed()
{
  const double *coords = wb_gps_get_values(gps);
  double vel[3] = {coords[X] - gps_coords[X], coords[Y] - gps_coords[Y], coords[Z] - gps_coords[Z]};
  double dist = sqrt(vel[X] * vel[X] + vel[Y] * vel[Y] + vel[Z] * vel[Z]);
  gps_speed = dist / time_step * 3600.0;
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

void update_imu_angle()
{
  const double *coords = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  memcpy(inertial_unit_coords, coords, sizeof(inertial_unit_coords));
}

void update_imu_acceleration()
{
  const double *coords = wb_accelerometer_get_values(accelerometer);
  memcpy(accelerometer_coords, coords, sizeof(accelerometer_coords));
}

void update_imu_angular_velocity()
{
  const double *coords = wb_gyro_get_values(gyro);
  memcpy(gyro_coords, coords, sizeof(gyro_coords));
}

int data_export(FILE *p)
{
  if (p == NULL)
  {
    printf("open error!\n");
    return -1;
  }
  else
  {
    update_gps_speed();
    update_imu_angle();
    update_imu_acceleration();
    update_imu_angular_velocity();
    fprintf(p, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            gps_coords[X], gps_coords[Y], gps_coords[Z],
            inertial_unit_coords[X], inertial_unit_coords[Y], inertial_unit_coords[Z],
            accelerometer_coords[X], accelerometer_coords[Y], accelerometer_coords[Z],
            gyro_coords[X], gyro_coords[Y], gyro_coords[Z]);
    return 0;
  }
}

void sensor_init()
{
  // initialize inertial unit
  inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);

  // initialize accelerometer
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  // initialize gyro
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, time_step);

  // initialize gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);
}

// ------------Main function------------
int main(int argc, char **argv)
{
  wbu_driver_init(); // Initialize devices (should be called at the very beginning of any controller program)
  time_step = (int)wb_robot_get_basic_time_step();

  // -------------Start engine-------------
  wbu_driver_set_brake_intensity(0); // Brakes the car by increasing the dampingConstant coefficient of the rotational joints of each of the four wheels.
  wbu_driver_set_throttle(0);        // Control the car in torque
  wbu_driver_set_gear(g);
  // ---------Preparation function---------
  sensor_init();                          // Initialize sensors
  print_help();                           // Print help info
  wb_keyboard_enable(time_step);          // Enable keyboard input
  FILE *p = fopen("test_data.txt", "w+"); // Export data to file(function: data_export)

  while (wbu_driver_step() != -1) // --------------Main loop---------------
  {
    check_keyboard(); // Get user input
    data_export(p);   // Update stuff
  }

  fclose(p);            // Close data file
  wbu_driver_cleanup(); // Cleanup the Webots API
  return 0;
}
