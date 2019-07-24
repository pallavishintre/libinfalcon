#include "test_falcon_impedance.h"

using namespace libnifalcon;

[[noreturn]]void force_test();

int main()
{
    initializeFalcon();
    force_test();
}

/**
 * @brief calculate_forces Essentially a PD controller
 * @param position  Current position vector
 * @param old_position  Previous position vector
 * @param delta_time    dt between last and current position
 * @return force vector
 */
std::array<double, 3> calculate_forces(std::array<double, 3> position, std::array<double, 3> old_position, double delta_time) {
    double scale = 750; //how many newtons per meter displacement to apply. Basically proportional gain
    double max_force = 10;  //Maximum force allowed (Newtons). Maximum recommended is 10
    double co_dt = 5000000;   //coefficient for derivative gain
    std::array<double, 3> ret;
    for (unsigned int i = 0; i < 3; i++) {
        double err_dt = (position[i] - old_position[i]) * delta_time;  //since setpoint is always 0, don't need to subtract that
        ret[i] = (-position[i] * scale) - (co_dt * err_dt);
        if (ret[i] > max_force) ret[i] = max_force; //Limit force
        if (ret[i] < -max_force) ret[i] = -max_force;
    }
//    printf("xyz: %5.3f %5.3f %5.3f\t", position[0], position[1], position[2]);
//    printf("xyzf: %5.3f %5.3f %5.3f\n", ret[0], ret[1], ret[2]);
    return ret;
}

void force_test() {
    std::array<double, 3> position, force_setpoint, old_position;
    double z_offset = -0.12, delta_time = 0;
    long old_time, new_time;
    timespec ts;
    old_position = dev.getPosition();
    clock_gettime(CLOCK_REALTIME, &ts);
    old_time = ts.tv_nsec;
    while (true) {
        if (dev.runIOLoop()) {
            position = dev.getPosition();
            position[2] += z_offset;

            clock_gettime(CLOCK_REALTIME, &ts);
            new_time = ts.tv_nsec;
            delta_time = new_time - old_time;
            if (delta_time < 0) delta_time += 1000000000;   //Offset by a billion nanoseconds to correct for a new second
            delta_time /= 1000000000.0; //Convert to seconds

            force_setpoint = calculate_forces(position, old_position, delta_time);
            dev.setForce(force_setpoint);

            printf("dt: %5.7f\t", delta_time);
            printf("012: %5d %5d %5d \t", firmware->getEncoderValues()[0], firmware->getEncoderValues()[1], firmware->getEncoderValues()[2]);
            printf("xyz: %5.4f %5.4f %5.4f\t", position[0], position[1], position[2]);
            printf("xyzf: %5.3f %5.3f %5.3f\n", force_setpoint[0], force_setpoint[1], force_setpoint[2]);

            old_position = position;
            old_time = new_time;
        }
    }
}
