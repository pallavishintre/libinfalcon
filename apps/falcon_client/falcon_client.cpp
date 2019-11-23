#include "falcon_client.h"

using namespace libnifalcon;

static nlohmann::json json;
static std::array<double, 3> force_setpoint = {0, 0, 0};
static std::array<double, 3> force_send = {0, 0, 0};

[[noreturn]] void req_comm() {
    sleep(1);   //Give time for falcon to stabalize

    zmq::context_t context;
    zmq::socket_t req = zmq::socket_t(context, zmq::socket_type::req);
    std::string reqSocketStr = "tcp://10.203.53.193:5560";
    req.connect(reqSocketStr);
    req.setsockopt(ZMQ_RCVTIMEO, 500);
    std::cout << "Requester socket connected to " << reqSocketStr << std::endl;

    while (true) {
        json["ID"] = 1;
        json["force_x"] = -force_send[0];
        json["force_y"] = -force_send[1];
        json["force_z"] = -force_send[2];
        zmq::message_t contents(json.dump());
        req.send(contents, zmq::send_flags::none);
        std::cout << "sent " + json.dump() << std::endl;
        req.recv(contents, zmq::recv_flags::none);
        std::string str(static_cast<char*>(contents.data()), contents.size());
        std::cout << "received " + str << std::endl;
    }
}

[[noreturn]] void simulator() {
    std::array<double, 3> pos_command = {0, 0, 0};
    std::array<double, 3> pos_previous = dev.getPosition();
    pos_previous[2] -= 0.12; //offset z
    double kp[] = {150, 150, 150};  //Proportional gains, xyz
    double kd[] = {1, 1, 1};    //Derivative gains, xyz
    double err[3];  //Proportional error, xyz
    double err_dt[3];   //Derivative error, xyz
    timespec prev_ts, current_ts;
    double dt;
    clock_gettime(CLOCK_REALTIME, &prev_ts);
    std::thread host_comm (req_comm);
    while (true) {
        if (dev.runIOLoop()) {
            clock_gettime(CLOCK_REALTIME, &current_ts);
            dt = (current_ts.tv_nsec - prev_ts.tv_nsec)/1000000000.0;//Converts to seconds
            if (dt < 0) dt = (current_ts.tv_nsec + 1000000000 - prev_ts.tv_nsec)/1000000000.0;

            std::array<double, 3> pos_current = dev.getPosition();
            //offset z
            pos_current[2] -= 0.125;

            double limit = 5.0;
            double limit1 = 4.5;
            double limit2 = 3.0;
            double limit3 = 1.5;
            double fval1 = 5.0;
            double fval2 = 3.5;
            double fval3 = 2.0;
            double fval4 = 0.0;

            //Calculate proportional error, derivative error, and force setpoint
            for (unsigned i = 0; i < 3; i++){
                err[i] = pos_command[i] - pos_current[i];   //Proportional error
                err_dt[i] = ((pos_command[i] - pos_current[i]) - (pos_command[i] - pos_previous[i])) / dt;  //Derivative error
                force_setpoint[i] = (err[i] * kp[i]) + (err_dt[i] * kd[i]); //Force setpoint

                //Make sure applied force isn't too much
                if (force_setpoint[i] > limit) force_setpoint[i] = limit;
                else if (force_setpoint[i] < -limit) force_setpoint[i] = -limit;



//Discretization for x in positive direction

            if ((force_setpoint[0] < limit3) && (force_setpoint[0] >= -limit3)) {
                force_send[0] = fval4;
            }
            else if ((force_setpoint[0] >= limit3) && (force_setpoint[0] < limit2)){
                force_send[0] = fval3;
            }
            else if ((force_setpoint[0] >= limit2) && (force_setpoint[0] < limit1)){
                force_send[0] = fval2;
            }
            else if (force_setpoint[0] >= limit1){
                force_send[0] = fval1;
            }

//Discretization for x in negative direction

            else if ((force_setpoint[0] < -limit3) && (force_setpoint[0] >= -limit2)){
                force_send[0] = -fval3;
            }
            else if ((force_setpoint[0] < -limit2) && (force_setpoint[0] >= -limit1)){
                force_send[0] = -fval2;
            }
            else if (force_setpoint[0] < -limit1){
                force_send[0] = -fval1;
            }
//Discretization for y in positive direction

            if ((force_setpoint[1] < limit3) && (force_setpoint[1] >= -limit3)) {
                force_send[1] = fval4;
            }
            else if ((force_setpoint[1] >= limit3) && (force_setpoint[1] < limit2)){
                force_send[1] = fval3;
            }
            else if ((force_setpoint[1] >= limit2) && (force_setpoint[1] < limit1)){
                force_send[1] = fval2;
            }
            else if (force_setpoint[1] >= limit1){
                force_send[1] = fval1;
            }
            }




            //If you want to monitor things
            if (!!false) {
                printf("x:%6.3f|y:%6.3f|xf:%5.2f|yf:%5.2f|"
                       "err_dx_dt:%6.3f|err_dy_dt:%6.3f|"
                       "dt:%0.6f|nsec:%ld|sec:%f\n",
                       pos_current[0], pos_current[1], force_setpoint[0], force_setpoint[1],
                       err_dt[0], err_dt[1],
                       dt, current_ts.tv_nsec, current_ts.tv_nsec/1000000000.0
                       );
            }

            //send force command to Falcon. Done element-wise in case sign has to be changed for right and left hand coordinates
            dev.setForce({force_setpoint[0], force_setpoint[1], force_setpoint[2]});

            //update previous position and time
            pos_previous = pos_current;
            prev_ts = current_ts;

            //sec_since_last_pub += dt;
        }
    }
}

int main()
{
    initializeFalcon();
    simulator();
}


