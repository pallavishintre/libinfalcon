#ifndef UTIL_H
#define UTIL_H

#include "falcon/core/FalconDevice.h"
#include "falcon/grip/FalconGripFourButton.h"

#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

#include <zmq.hpp>
#include <string>
#include <thread>

bool initializeFalcon();

extern std::shared_ptr<libnifalcon::FalconFirmware> firmware;
extern libnifalcon::FalconDevice dev;

#endif // UTIL_H
