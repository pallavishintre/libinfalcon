#include "util.h"

using namespace libnifalcon;

std::shared_ptr<libnifalcon::FalconFirmware> firmware;
unsigned int num_falcons = 0;
unsigned int count;
libnifalcon::FalconDevice dev;

zmq::socket_t pub;
zmq::socket_t sub;

void init_falcon() {

    dev.setFalconFirmware<FalconFirmwareNovintSDK>();   //idk
    dev.setFalconKinematic<FalconKinematicStamper>();   //For kinematics

    firmware = dev.getFalconFirmware();


    if(!dev.getDeviceCount(num_falcons))
    {
        std::cout << "Cannot get device count" << std::endl;
        return;
    }

    count = 0;

    std::cout << "Falcons found: " << static_cast<int>(num_falcons) << std::endl;

    if(num_falcons == 0)
    {
        std::cout << "No falcons found, exiting..." << std::endl;
        return;
    }

    for(unsigned int z = 0; z < num_falcons; ++z)
    {
        std::cout << "Opening falcon " << z + 1  << std::endl;

        while (!dev.open(z))
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
                std::cout << "Could not load firmware" << std::endl;
                return;
            }
            std::cout <<"Firmware loaded" << std::endl;
        } else {
            std::cout << "Firmware already loaded" << std::endl;
        }
    }

    while (!dev.getFalconFirmware()->isHomed()) {
        dev.getFalconFirmware()->setHomingMode(true);
        dev.runIOLoop();
        if(!dev.getFalconFirmware()->isHomed())
        {
            dev.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
            std::cout << "Falcon not currently calibrated. Move control all the way out then push straight all the way in." << std::endl;
        }
        std::cout << "Falcon calibrated successfully." << std::endl;
        dev.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
    }
    return;
}

void init_zmq(int pub_socket_port, int sub_socket_port) {
    zmq::context_t context (1);
    pub = zmq::socket_t(context, ZMQ_PUB);
    sub = zmq::socket_t(context, ZMQ_SUB);

    std::string pubSocket = "tcp://localhost:" + std::to_string(pub_socket_port);
    std::string subSocket = "tcp://*:" + std::to_string(sub_socket_port);
    std::cout << pubSocket << " " << subSocket << std::endl;

    //pub.setsockopt(ZMQ_CONFLATE, 1);
    pub.connect (pubSocket);
    std::cout << "pub connected" << std::endl;

    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub.setsockopt(ZMQ_CONFLATE, 1);
    sub.bind(subSocket);
    std::cout << "Sockets are ready" << std::endl;
}
