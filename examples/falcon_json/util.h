#ifndef LOADFIRMWARE_H
#define LOADFIRMWARE_H

#include "falcon/core/FalconDevice.h"
#include "falcon/grip/FalconGripFourButton.h"

#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

#include <string>
#include <thread>

#include <zmq.hpp>

void init_falcon();
void init_zmq(int pub_socket_port, int sub_socket_port);

extern std::shared_ptr<libnifalcon::FalconFirmware> firmware;
extern unsigned int num_falcons;
extern unsigned int count;
extern libnifalcon::FalconDevice dev;

extern zmq::socket_t pub;
extern zmq::socket_t sub;

#endif // LOADFIRMWARE_H
