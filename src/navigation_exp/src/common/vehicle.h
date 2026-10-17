#pragma once

#include <cmath>
#include <chrono>
#include "common/mathematics.h"

struct Kinematics {
    float x, y, z;
    float roll_abt_y_axis, pitch_abt_x_axis, yaw_abt_z_axis; // radians
    float vel_x, vel_y, vel_z;
    std::chrono::time_point<std::chrono::steady_clock> posted_control_time;

    Kinematics() : x(0), y(0), z(0), roll_abt_y_axis(0), pitch_abt_x_axis(0), yaw_abt_z_axis(0), vel_x(0), vel_y(0), vel_z(0), posted_control_time(std::chrono::steady_clock::now()) {}

    Vertex gravity = Vertex(0, 0, 0);

    Vertex up() const {
        Vertex g = gravity;
        return (g.magnitude() == 0) ? Vertex(0, -1, 0) : ~g;
    }

    void update_position(std::chrono::time_point<std::chrono::steady_clock> control_time, Motion &motion) {
        std::chrono::duration<double> delta_t = control_time - posted_control_time;
        double seconds_delta = delta_t.count();

        const Vertex accel(motion.accel_x, motion.accel_y, motion.accel_z);
        gravity = (gravity.magnitude() == 0) ? accel : gravity * 0.98f + accel * 0.02f;

        const Vertex u = up();
        const Vertex forward = ~(Vertex(0, 0, 1) - u * u.z);
        const Vertex right = ~(Vertex(1, 0, 0) - u * u.x);
        const float a_forward = accel.dot(forward);
        const float a_right = accel.dot(right);
        const float c = std::cos(yaw_abt_z_axis);
        const float s = std::sin(yaw_abt_z_axis);

        vel_x += (a_right * c + a_forward * s) * seconds_delta;
        vel_y += (-a_right * s + a_forward * c) * seconds_delta;

        x += vel_x * seconds_delta;
        y += vel_y * seconds_delta;
    }

    void update_orientation(std::chrono::time_point<std::chrono::steady_clock> control_time, Motion &motion) {
        std::chrono::duration<double> delta_t = control_time - posted_control_time;
        double seconds_delta = delta_t.count();

        const Vertex u = up();
        yaw_abt_z_axis -= (motion.gyro_x * u.x + motion.gyro_y * u.y + motion.gyro_z * u.z) * seconds_delta;
        pitch_abt_x_axis += motion.gyro_x * seconds_delta;
        roll_abt_y_axis += motion.gyro_z * seconds_delta;
    }
};

struct PointVector
{
    float x;
    float y;
};

struct Vehicle
{
    float width;
    float height;
    float wheel_height;
    float wheel_diameter;
    PointVector destination;

    PointVector heading_vector;

    PointVector mxpy; // left top, corner closest to realsense
    PointVector pxpy;
    PointVector mxmy;
    PointVector pxmy;

    Kinematics orientation;

    void update_vehicle_intrinsics(){
        heading_vector.x = std::sin(orientation.yaw_abt_z_axis);
        heading_vector.y = std::cos(orientation.yaw_abt_z_axis);
        const float length = height;

        mxpy.x = orientation.x - (width/2)*heading_vector.y;
        mxpy.y = orientation.y + (width/2)*heading_vector.x;

        pxpy.x = orientation.x + (width/2)*heading_vector.y;
        pxpy.y = orientation.y - (width/2)*heading_vector.x;

        mxmy.x = orientation.x - length*heading_vector.x - (width/2)*heading_vector.y;
        mxmy.y = orientation.y - length*heading_vector.y + (width/2)*heading_vector.x;

        pxmy.x = orientation.x - length*heading_vector.x + (width/2)*heading_vector.y;
        pxmy.y = orientation.y - length*heading_vector.y - (width/2)*heading_vector.x;
    }
};