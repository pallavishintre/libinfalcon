#include "falcon_practice.h"

using namespace libnifalcon;

[[noreturn]] void simulator() {
    std::cout << "Setting up connection to simulation" << std::endl;

    zmq::context_t context;
    zmq::socket_t pub, sub;
    pub = zmq::socket_t(context, ZMQ_PUB);
    sub = zmq::socket_t(context, ZMQ_SUB);

    std::string pubSocket = "tcp://*:5557";
    std::string subSocket = "tcp://localhost:5558";
    std::cout << "Publishing on " << pubSocket << std::endl;
    std::cout << "Subscribing on" << subSocket << std::endl;

    pub.bind(pubSocket);
    int hwm_limit = 1;
    pub.setsockopt(ZMQ_SNDHWM, &hwm_limit, sizeof(hwm_limit));
    pub.setsockopt(ZMQ_SNDBUF, &hwm_limit, sizeof(hwm_limit));
    std::cout << "Publisher socket connected" << std::endl;

    sub.connect(subSocket);
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    std::cout << "Sockets are ready" << std::endl;

    nlohmann::json j_pub, j_sub;
    std::array<double, 3> hip_pos = {0, 0, 0};
    std::array<double, 3> hip_vel = {0, 0, 0};
    std::array<double, 3> pos_command = {0, 0, 0};
    std::array<double, 3> force_setpoint = {0, 0, 0};
    std::array<double, 3> pos_previous = dev.getPosition();
    pos_previous[2] -= 0.12; //offset z
    //double kp_x = 250, kp_y = 250, kp_z = 250;    //Proportional gains
    //double kd_x = 10000000, kd_y = 10000000, kd_z = 10000000;  //Derivative gains
    double kp_x = 100, kp_y = 100, kp_z = 100;    //Proportional gains
    double kd_x = 1, kd_y = 1, kd_z = 1;  //Derivative gains
    double err_x, err_y, err_z; //Proportional error
    double err_dx_dt = 0, err_dy_dt = 0, err_dz_dt = 0;  //Derivative error
    timespec prev_ts, current_ts;
    double dt;
    double sec_since_last_pub = 0;
    clock_gettime(CLOCK_REALTIME, &prev_ts);
    while (true) {
        if (dev.runIOLoop()) {
            clock_gettime(CLOCK_REALTIME, &current_ts);
            dt = (current_ts.tv_nsec - prev_ts.tv_nsec)/1000000000.0;//Converts to seconds
            if (dt < 0) dt = (current_ts.tv_nsec + 1000000000 - prev_ts.tv_nsec)/1000000000.0;

            std::array<double, 3> pos_current = dev.getPosition();
            //offset z
            pos_current[2] -= 0.12;

            //            //Calculate error (difference in position between Falcon and HIP)
            //            err_x = hip_pos[0] - pos_current[0];
            //            err_y = hip_pos[1] - pos_current[1];
            //            err_z = hip_pos[2] - pos_current[2];

            //Calculate error
            err_x = pos_command[0] - pos_current[0];
            err_y = pos_command[1] - pos_current[1];
            err_z = pos_command[2] - pos_current[2];


            //            //Calculate rate of change of error (difference in velocity between Falcon and HIP)
            //            err_dx_dt = hip_vel[0] - ((pos_current[0] - hip_pos[0]) - (pos_previous[0] - hip_pos[0])) * dt;
            //            err_dy_dt = hip_vel[1] - ((pos_current[1] - hip_pos[1]) - (pos_previous[1] - hip_pos[1])) * dt;
            //            err_dz_dt = hip_vel[2] - ((pos_current[2] - hip_pos[2]) - (pos_previous[2] - hip_pos[2])) * dt;

            //Calculate rate of change of error (difference in velocity between Falcon and HIP)
            err_dx_dt = ((pos_current[0] - pos_command[0]) - (pos_previous[0] - pos_command[0])) * dt;
            err_dy_dt = ((pos_current[1] - pos_command[1]) - (pos_previous[1] - pos_command[1])) * dt;
            err_dz_dt = ((pos_current[2] - pos_command[2]) - (pos_previous[2] - pos_command[2])) * dt;

            //Calculate force_setpoint
            force_setpoint[0] = (err_x * kp_x) + (err_dx_dt * kd_x);
            force_setpoint[1] = (err_y * kp_y) + (err_dy_dt * kd_y);
            force_setpoint[2] = (err_z * kp_z) + (err_dz_dt * kd_z);

            double limit = 10.0;

            if (force_setpoint[0] > limit) {
                force_setpoint[0] = limit;
            }
            if (force_setpoint[0] < -limit) {
                force_setpoint[0] = -limit;
            }
            if (force_setpoint[1] > limit) {
                force_setpoint[1] = limit;
            }
            if (force_setpoint[1] < -limit) {
                force_setpoint[1] = -limit;
            }
            if (force_setpoint[2] > limit) {
                force_setpoint[2] = limit;
            }
            if (force_setpoint[2] < -limit) {
                force_setpoint[2] = -limit;
            }

            printf("x:%6.3f|y:%6.3f|xf:%5.2f|yf:%5.2f|"
                   "err_dx_dt:%6.3f|err_dy_dt:%6.3f|"
                   "dt:%0.6f|nsec:%ld|sec:%f\n",
                   pos_current[0], pos_current[1], force_setpoint[0], force_setpoint[1],
                   err_dx_dt, err_dy_dt,
                   dt, current_ts.tv_nsec, current_ts.tv_nsec/1000000000.0
                   );

            //send force command to Falcon
            dev.setForce({force_setpoint[0], force_setpoint[1], force_setpoint[2]});

            //update previous position and time
            pos_previous = pos_current;
            prev_ts = current_ts;

            sec_since_last_pub += dt;

            //publish position at approx 30Hz
            if (sec_since_last_pub > 1/30.0) {
                j_pub["force_x"] = -force_setpoint[0];
                j_pub["force_y"] = -force_setpoint[1];
                j_pub["force_z"] = -force_setpoint[2];
                zmq::message_t contents(j_pub.dump());
                pub.send(contents, zmq::send_flags::dontwait);
                sec_since_last_pub = 0;
            }

            //Read incoming messages
            zmq::message_t contents;
            if (sub.recv(contents, zmq::recv_flags::dontwait)) {
                std::string str(static_cast<char*>(contents.data()), contents.size());
                j_sub = nlohmann::json::parse(str);
                std::cout << j_sub.dump() << std::endl;
                printf("%5.3f\t%5.3f\t%5.3f", force_setpoint[0], force_setpoint[1], force_setpoint[2]);
                hip_pos[0] = j_sub["Position"][0].get<double>() / 100;
                hip_pos[1] = j_sub["Position"][1].get<double>() / 100;
                hip_pos[2] = j_sub["Position"][2].get<double>() / 100;
                hip_vel[0] = j_sub["Velocity"][0].get<double>() / 100;
                hip_vel[1] = j_sub["Velocity"][1].get<double>() / 100;
                hip_vel[2] = j_sub["Velocity"][2].get<double>() / 100;
            }
        }
    }
}

int main()
{
    initializeFalcon();
    simulator();
}
