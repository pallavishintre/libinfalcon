#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <unistd.h>
#include <thread>

#include "falcon/core/FalconDevice.h"
#include "falcon/grip/FalconGripFourButton.h"

#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

#include <zmq.hpp>

#include <nlohmann/json.hpp>

bool initializeFalcon();

void createSockets(std::string pub_address, std::string sub_address);
void socketPublishJson(nlohmann::json j);
nlohmann::json socketReadJson();

void populateJsonDeviceCurrentPosition(nlohmann::json &j, double scale = 1);

extern std::shared_ptr<libnifalcon::FalconFirmware> firmware;
extern libnifalcon::FalconDevice dev;

#endif // UTIL_H
