#include "util.h"

bool calibrateDevice();

using namespace libnifalcon;

std::shared_ptr<libnifalcon::FalconFirmware> firmware;
libnifalcon::FalconDevice dev;

static bool displayed_calibrate_message = false;
static unsigned int count;
static unsigned int num_falcons;

bool initializeFalcon() {

    dev.setFalconFirmware<FalconFirmwareNovintSDK>();   //idk
    dev.setFalconKinematic<FalconKinematicStamper>();   //For kinematics

    firmware = dev.getFalconFirmware();

    if(!dev.getDeviceCount(num_falcons))
    {
        std::cout << "Cannot get device count" << std::endl;
        return false;
    }

    count = 0;

    std::cout << "Falcons found: " << static_cast<int>(num_falcons) << std::endl;

    if(num_falcons == 0)
    {
        std::cout << "No falcons found, exiting." << std::endl;
        return false;
    }

    if (num_falcons > 1) {
        std::cout << "Too many falcons found, please only connect one to this machine" << std::endl;
        return false;
    }

        std::cout << "Opening falcon "  << std::endl;

        while (!dev.open(0))
        {
            std::cout << "Cannot open falcon - Error: " << std::endl; // << dev.getErrorCode() << std::endl;
            //return;
        }
        std::cout << "Opened falcon" << std::endl;

        if(!dev.isFirmwareLoaded())
        {
            std::cout << "Loading firmware" << std::endl;
            while (!dev.getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
            {
                std::cout << "Could not load firmware. Trying again." << std::endl;
            }
            std::cout <<"Firmware loaded" << std::endl;
        } else {
            std::cout << "Firmware already loaded" << std::endl;
        }

    while (!calibrateDevice());
    return true;
}

bool calibrateDevice()
{
    dev.getFalconFirmware()->setHomingMode(true);
    dev.runIOLoop();
    if(!dev.getFalconFirmware()->isHomed())
    {
        dev.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
        if (!displayed_calibrate_message){
            std::cout << "Falcon not currently calibrated. Move control all the way out then push straight all the way in." << std::endl;
            displayed_calibrate_message = true;
        }
        return false;
    }
    std::cout << "Falcon calibrated successfully." << std::endl;
    dev.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
    return true;
}

void populateJsonDeviceCurrentPosition(nlohmann::json &j, double scale) {
    dev.runIOLoop();
    std::array<double, 3> pos = dev.getPosition();
    j["current_position_x"] = pos[0] * scale;
    j["current_position_y"] = pos[1] * scale;
    j["current_position_z"] = pos[2] * scale;
}
