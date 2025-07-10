#include "headfile.h"
#include "stm32h750xx.h"

#include "sf.h"
#include <math.h>
#include <stdint.h>

// ?????
#define BASE_HEIGHT      50.0    // ????(mm)
const double link_len = {100.0, 80.0, 60.0, 40.0, 20.0, 10.0}; // ????
#define MAX_ITERATIONS   100     // ??????
#define ERROR_THRESHOLD  0.1     // ????(mm)
#define M_PI             3.14159265358979323846

// ????
#define JOINT_OFFSET     45.0    // ???????
#define SERVO_MIN_DEG    0.0     // ??????
#define SERVO_MAX_DEG    270.0   // ??????

// ?????
typedef struct {
    double x;
    double y;
} Point2D;

// ??????
static int inverse_kinematics(const double target, double angles);
static double rad_to_deg(double rad);
static double joint_to_servo_angle(double joint_deg);
static double to_servo_angle(double rad, int servo_id);

// ?????:??????,??????
int compute_servo_angles(const double target, double servo_angles) 
{
    double joint_angles_rad;
    int result = inverse_kinematics(target, joint_angles_rad);
    
    if(result == 0) {
        for(int i = 0; i < 6; i++) {
            servo_angles[i] = to_servo_angle(joint_angles_rad[i], i);
        }
        return 0;  // ??
    }
    return -1;  // ??
}

// ????????
static int inverse_kinematics(const double target, double angles) 
{
    const double base_h = BASE_HEIGHT;
    double r = sqrt(target*target + target*target);
    angles = atan2(target, target);  // ????

    // ???????
    double z_eff = target - base_h;
    Point2D planar_target = {r, z_eff};

    // ???????
    Point2D positions = {{0,0}};
    for(int i = 1; i < 6; i++) {
        positions[i].x = positions[i-1].x + link_len[i-1];
    }

    // CCD????
    for(int iter = 0; iter < MAX_ITERATIONS; iter++) {
        double dx = planar_target.x - positions.x;
        double dy = planar_target.y - positions.y;
        double error = sqrt(dx*dx + dy*dy);
        if(error < ERROR_THRESHOLD) break;

        // ?????????
        for(int i = 4; i >= 0; i--) {
            Point2D cur_to_end = {positions.x - positions[i].x, positions.y - positions[i].y};
            Point2D cur_to_target = {planar_target.x - positions[i].x, planar_target.y - positions[i].y};
            
            double dot = cur_to_end.x*cur_to_target.x + cur_to_end.y*cur_to_target.y;
            double det = cur_to_end.x*cur_to_target.y - cur_to_end.y*cur_to_target.x;
            double delta_angle = atan2(det, dot);

            // ????
            for(int j = i+1; j < 6; j++) {
                double x_rel = positions[j].x - positions[i].x;
                double y_rel = positions[j].y - positions[i].y;
                positions[j].x = positions[i].x + x_rel*cos(delta_angle) - y_rel*sin(delta_angle);
                positions[j].y = positions[i].y + x_rel*sin(delta_angle) + y_rel*cos(delta_angle);
            }
        }
    }

    // ??????
    double prev_angle = 0.0;
    for(int i = 0; i < 5; i++) {
        double dx = positions[i+1].x - positions[i].x;
        double dy = positions[i+1].y - positions[i].y;
        double abs_angle = atan2(dy, dx);
        angles[i+1] = abs_angle - prev_angle;
        prev_angle = abs_angle;
    }

    // ?????
    double final_dx = planar_target.x - positions.x;
    double final_dy = planar_target.y - positions.y;
    double final_error = sqrt(final_dx*final_dx + final_dy*final_dy);
    return (final_error < 5.0) ? 0 : -1;
}

// ????
static double rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

static double joint_to_servo_angle(double joint_deg) {
    double servo_deg = joint_deg + JOINT_OFFSET;
    while(servo_deg < SERVO_MIN_DEG) servo_deg += 360.0;
    while(servo_deg >= SERVO_MAX_DEG) servo_deg -= 360.0;
    return servo_deg;
}

static double to_servo_angle(double rad, int servo_id) {
    return joint_to_servo_angle(rad_to_deg(rad));
}
