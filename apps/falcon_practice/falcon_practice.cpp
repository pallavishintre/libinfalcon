#include "falcon_practice.h"

using namespace libnifalcon;

[[noreturn]] void simulator() {
    std::cout << "Setting up connection to simulation" << std::endl;

    zmq::context_t context;
    zmq::socket_t pub, sub;
    pub = zmq::socket_t(context, ZMQ_PUB);
    sub = zmq::socket_t(context, ZMQ_SUB);

    std::string pubSocket = "tcp://localhost:5557";
    std::string subSocket = "tcp://*:5558";
    std::cout << "Publishing on " << pubSocket << std::endl;
    std::cout << "Subscribing on" << subSocket << std::endl;

    pub.setsockopt(ZMQ_CONFLATE, 1);
    pub.connect (pubSocket);
    std::cout << "Publisher socket connected" << std::endl;

    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub.setsockopt(ZMQ_CONFLATE, 1);
    sub.bind(subSocket);
    std::cout << "Sockets are ready" << std::endl;

    std::array<double, 3> pos_command = {0, 0, 0};
    std::array<double, 3> force_setpoint = {0, 0, 0};
    std::array<double, 3> pos_previous = dev.getPosition();
    pos_previous[2] -= 0.12; //offset z
    double kp_x = 250, kp_y = 250, kp_z = 250;    //Proportional gains
    double kd_x = 10000000, kd_y = 10000000, kd_z = 10000000;  //Derivative gains
    double err_x, err_y, err_z; //Proportional error
    double err_dx_dt = 0, err_dy_dt = 0, err_dz_dt = 0;  //Derivative error
    double sim_force_x = 0, sim_force_y = 0, sim_force_z = 0;
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

            //Calculate errorr
            err_x = pos_current[0] - pos_command[0];
            err_y = pos_current[1] - pos_command[1];
            err_z = pos_current[2] - pos_command[2];


            //Calculate rate of change of error
            err_dx_dt = ((pos_current[0] - pos_command[0]) - (pos_previous[0] - pos_command[0])) * dt;
            err_dy_dt = ((pos_current[1] - pos_command[1]) - (pos_previous[1] - pos_command[1])) * dt;
            err_dz_dt = ((pos_current[2] - pos_command[2]) - (pos_previous[2] - pos_command[2])) * dt;

            //Calculate force_setpoint
            force_setpoint[0] = -(err_x * kp_x) - (err_dx_dt * kd_x) + sim_force_x;
            force_setpoint[1] = -(err_y * kp_y) - (err_dy_dt * kd_y) + sim_force_y;
            force_setpoint[2] = -(err_z * kp_z) - (err_dz_dt * kd_z) + sim_force_z;

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
                   "dt:%0.6f|nsec:%ld|sec:%f|"
                   "sin_xyz: %0.4f %0.4f %0.4f\n",
                   pos_current[0], pos_current[1], force_setpoint[0], force_setpoint[1],
                   err_dx_dt, err_dy_dt,
                   dt, current_ts.tv_nsec, current_ts.tv_nsec/1000000000.0,
                   sim_force_x, sim_force_y, sim_force_z);

            //send force command to Falcon
            dev.setForce(force_setpoint);

            //update previous position and time
            pos_previous = pos_current;
            prev_ts = current_ts;

            sec_since_last_pub += dt;

            //publish position at approx 30Hz
            if (sec_since_last_pub > 1/30.0) {
                std::string msg = std::to_string(pos_current[0]) + " " + std::to_string(pos_current[1]) + " " + std::to_string(-pos_current[2]);
                std::cout << "Sending data " << msg << " Current setpoint: " << force_setpoint[0] << " " << force_setpoint[1] << " " << force_setpoint[2] << std::endl;
                pub.send (zmq::buffer(msg), zmq::send_flags::dontwait);
                sec_since_last_pub = 0;
            }

//            zmq::message_t zmqMsg;
//            while (sub.recv(zmqMsg, zmq::recv_flags::dontwait)) {
//                std::cout << "Processing incoming message" << std::endl;
//                std::string rcvStr = std::string(static_cast<char*>(zmqMsg.data()), zmqMsg.size());
//                std::cout << rcvStr << std::endl;
//                char cstr[100];
//                memset(cstr, 0, sizeof(cstr));
//                std::strcpy(cstr, rcvStr.c_str());
//                char * pch;
//                pch = std::strtok(cstr, "(), ");
//                unsigned long forceidx = 0;
//                std::array<double, 3> force;
//                while (pch != nullptr) {
//                    force[forceidx++] = std::atof(pch);// * 1.5;
//                    std::cout << pch << std::endl;
//                    pch = std::strtok(nullptr, "(), ");
//                }
//                sim_force_x = force[0]/3000.0;
//                sim_force_y = force[1]/3000.0;
//                sim_force_z = -force[2]/3000.0;
//            }

        }
    }
}

int main()
{
    initializeFalcon();
    simulator();
}

