#ifndef LOADFIRMWARE_H
#define LOADFIRMWARE_H

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

#include <string>
#include <thread>

#include <zmq.hpp>

void init_falcon();
void init_zmq(int base_socket_port);

extern std::shared_ptr<libnifalcon::FalconFirmware> firmware;
extern unsigned int num_falcons;
extern unsigned int count;
extern std::vector<libnifalcon::FalconDevice> dev;

extern zmq::socket_t pub1, sub1, pub2, sub2;

#endif // LOADFIRMWARE_H
