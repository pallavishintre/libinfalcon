/***
 * @file findfalcons.cpp
 * @brief Minimal open-and-run test for the C++ version of libnifalcon
 * @author Kyle Machulis (kyle@nonpolynomial.com)
 * @version $Id$
 * @copyright (c) 2007-2008 Nonpolynomial Labs/Kyle Machulis
 * @license BSD License
 *
 * $HeadURL$
 *
 * Project info at http://libnifalcon.nonpolynomial.com/
 *
 */

#include "displacement_based_force.h"

using namespace libnifalcon;

void runFalconTest();
[[noreturn]]void force_test();
[[noreturn]]void simulator();

int main()
{
    /*======================================================================================
    zmq::context_t context (1);
    //zmq::socket_t socket (context, ZMQ_PUSH);
    zmq::socket_t socket (context, ZMQ_PUB);

    std::cout << "Connecting to hello world server…" << std::endl;
    socket.connect ("tcp://localhost:5565");

    //  Do 10 requests, waiting each time for a response
    //for (int request_nbr = 0; request_nbr != 1000; request_nbr++) {
    unsigned int request_nbr = 0;
    while (true) {
        std::string msg = std::to_string(request_nbr++);
        std::cout << "Sending Hello " << msg << "…" << std::endl;
        socket.send (zmq::buffer(msg), zmq::send_flags::none);

        usleep(50000);

    }
    =============================================================================*/
    init_falcon();
    force_test();
    //init_zmq(5557, 5558);
    //simulator();
}

std::array<double, 3> calculate_forces(std::array<double, 3> position) {
    double scale = 500; //how many newtons per meter displacement to apply
    double max_force = 5;  //Maximum force allowed
    std::array<double, 3> ret;
    for (unsigned int i = 0; i < 3; i++) {
        ret[i] = -position[i] * scale;
        if (ret[i] > max_force) ret[i] = max_force;
        if (ret[i] < -max_force) ret[i] = -max_force;
    }
//    printf("xyz: %5.3f %5.3f %5.3f\t", position[0], position[1], position[2]);
//    printf("xyzf: %5.3f %5.3f %5.3f\n", ret[0], ret[1], ret[2]);
    return ret;
}

void force_test() {
    std::array<double, 3> position, force_setpoint;
    double z_offset = -0.135;
    while (true) {
        if (dev.runIOLoop()) {
            position = dev.getPosition();
            position[2] += z_offset;

            force_setpoint = calculate_forces(position);
            dev.setForce(force_setpoint);

            printf("012: %5d %5d %5d\t", firmware->getEncoderValues()[0], firmware->getEncoderValues()[1], firmware->getEncoderValues()[2]);
            printf("xyz: %5.3f %5.3f %5.3f\t", position[0], position[1], position[2]);
            printf("xyzf: %5.3f %5.3f %5.3f\n", force_setpoint[0], force_setpoint[1], force_setpoint[2]);
        }
    }
}

void simulator() {
    std::cout << "Entered simulator" << std::endl;

    zmq::context_t context (1);
    pub = zmq::socket_t(context, ZMQ_PUB);
    sub = zmq::socket_t(context, ZMQ_SUB);

    std::string pubSocket1 = "tcp://localhost:" + std::to_string(5557);
    std::string subSocket1 = "tcp://*:" + std::to_string(5558);
    std::cout << pubSocket1 << " " << subSocket1 << std::endl;

    //pub.setsockopt(ZMQ_CONFLATE, 1);
    pub.connect (pubSocket1);
    std::cout << "pub connected" << std::endl;

    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub.setsockopt(ZMQ_CONFLATE, 1);
    sub.bind(subSocket1);
    std::cout << "Sockets are ready" << std::endl;

    std::array<double, 3> pos_command = {0, 0, 0};
    std::array<double, 3> force_setpoint = {0, 0, 0};
    std::array<double, 3> pos_previous = dev.getPosition();
    pos_previous[2] -= 0.125; //offset z
    //double kp_x = 100, kp_y = 100, kp_z = 100;    //Proportional gains
    double kp_x = 0, kp_y = 0, kp_z = 0;    //Proportional gains
    double kd_x = 10000000, kd_y = 10000000, kd_z = 10000000;  //Derivative gains
    double err_x, err_y, err_z; //Proportional error
    double err_dx_dt = 0, err_dy_dt = 0, err_dz_dt = 0;  //Derivative of error
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
            pos_current[2] -= 0.15;
            //pos_current[1] += 0.05;

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

            double limit = 5.0;

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

            zmq::message_t zmqMsg;
            while (sub.recv(zmqMsg, zmq::recv_flags::dontwait)) {
                std::cout << "Processing incoming message" << std::endl;
                std::string rcvStr = std::string(static_cast<char*>(zmqMsg.data()), zmqMsg.size());
                std::cout << rcvStr << std::endl;
                char cstr[100];
                memset(cstr, 0, sizeof(cstr));
                std::strcpy(cstr, rcvStr.c_str());
                char * pch;
                pch = std::strtok(cstr, "(), ");
                unsigned long forceidx = 0;
                std::array<double, 3> force;
                while (pch != nullptr) {
                    force[forceidx++] = std::atof(pch) * 1.5;
                    std::cout << pch << std::endl;
                    pch = std::strtok(nullptr, "(), ");
                }
                sim_force_x = force[0]/3000.0;
                sim_force_y = force[1]/3000.0;
                sim_force_z = -force[2]/3000.0;
            }

        }
    }
}

void runFalconTest()
{
    zmq::context_t context (1);
    zmq::socket_t pub (context, ZMQ_PUB);
    zmq::socket_t sub (context, ZMQ_SUB);

    pub.connect ("tcp://localhost:5565");
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub.bind("tcp://*:5568");

    //  Do 10 requests, waiting each time for a response
    //for (int request_nbr = 0; request_nbr != 1000; request_nbr++) {
    //    unsigned int request_nbr = 0;
    //    while (true) {
    //        std::string msg = std::to_string(request_nbr++);
    //        std::cout << "Sending Hello " << msg << "…" << std::endl;
    //        socket.send (zmq::buffer(msg), zmq::send_flags::none);

    //        usleep(50000);

    //    }


    for(int j = 0; j < 3; ++j)
    {
        firmware->setLEDStatus(static_cast<unsigned int>(2 << (j % 3)));
        for(int i = 0; i < 10000; )
        {
            if(dev.runIOLoop()) ++i;
            else continue;
            std::array<double, 3> pos = dev.getPosition();
            // offset z
            pos[2] -= 0.11;
            pos[1] += 0.05;
            //printf("Loops: %8d | Enc1: %5d | Enc2: %5d | Enc3: %5d | X: %.5f | Y: %.5f | Z: %.5f\n",
            //       (j*1000)+i,  firmware->getEncoderValues()[0], firmware->getEncoderValues()[1], firmware->getEncoderValues()[2], pos[0], pos[1], pos[2]);
            std::string msg = std::to_string(pos[0]) + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]);
            //std::cout << "Sending data " << msg << std::endl;
            pub.send (zmq::buffer(msg), zmq::send_flags::none);
            usleep(33000);
            zmq::message_t zmqMsg;
            while (sub.recv(zmqMsg, zmq::recv_flags::dontwait)) {
                std::string rcvStr = std::string(static_cast<char*>(zmqMsg.data()), zmqMsg.size());
                std::cout << rcvStr << std::endl;
                char cstr[100];
                memset(cstr, 0, sizeof(cstr));
                std::strcpy(cstr, rcvStr.c_str());
                char * pch;
                pch = std::strtok(cstr, "(), ");
                unsigned long forceidx = 0;
                std::array<double, 3> force;
                while (pch != nullptr) {
                    //std::string tempStr = pch;
                    //std::cout << tempStr << std::endl;
                    force[forceidx++] = std::atof(pch) * 1.5;
                    std::cout << pch << std::endl;
                    pch = std::strtok(nullptr, "(), ");
                    //force[forceidx++] = std::atof(tempStr.c_str());
                }
                force[1] = 0;
                force[2] = 0;
                dev.setForce(force);
            }

            ++count;
        }
    }
    firmware->setLEDStatus(0);
    dev.runIOLoop();
    dev.close();
}
