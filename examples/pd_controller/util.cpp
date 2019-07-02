#include "util.h"

using namespace libnifalcon;


std::shared_ptr<libnifalcon::FalconFirmware> firmware;
unsigned int num_falcons;
unsigned int count;
std::vector<libnifalcon::FalconDevice> dev;

zmq::socket_t pub1, sub1, pub2, sub2;

void init_falcon() {

    dev.push_back(FalconDevice());

    dev[0].setFalconFirmware<FalconFirmwareNovintSDK>();   //idk
    dev[0].setFalconKinematic<FalconKinematicStamper>();   //For kinematics

    if(!dev[0].getDeviceCount(num_falcons))
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

    dev.clear();

    for (unsigned int i = 0; i < num_falcons; i++) {
        dev.push_back(FalconDevice());
        dev[i].setFalconFirmware<FalconFirmwareNovintSDK>();   //idk
        dev[i].setFalconKinematic<FalconKinematicStamper>();   //For kinematics
        if (!dev[i].open(i)) {
            std::cout << "Cannot open falcon - Error: " << std::endl;
            return;
        }
        std::cout << "Opened falcon " << i << std::endl;
        if (!dev[i].isFirmwareLoaded()) {
            for (unsigned int j = 0; j < 10; j++) {
                if (!dev[i].getFalconFirmware()->loadFirmware(false, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE))) {
                    std::cout << i << " Could not load firmware" << std::endl;
                    continue;
                } else {
                    std::cout << i << " Firmware loaded" << std::endl;
                    break;
                }
            }
            if (!dev[i].isFirmwareLoaded()) {
                std::cout << i << " Firmware didn't load correctly. Please try again" << std::endl;
                return;
            }
        }
        std::cout << i << " Falcon firmware " << i << " is loaded" << std::endl;
    }

//        while (true){
//            while (!dev[0].isOpen()) dev[0].open(0);
//            std::cout << dev[0].getPosition()[0] << std::endl;
//            dev[0].runIOLoop();

//            while (!dev[1].isOpen()) dev[1].open(1);
//            std::cout << dev[1].getPosition()[0] << std::endl;
//            dev[1].runIOLoop();
//        }
}

void init_zmq(int base_socket_port) {
    zmq::context_t context (1);
    pub1 = zmq::socket_t(context, ZMQ_PUB);
    sub1 = zmq::socket_t(context, ZMQ_SUB);
    pub2 = zmq::socket_t(context, ZMQ_PUB);
    sub2 = zmq::socket_t(context, ZMQ_SUB);

    std::string pubSocket1 = "tcp://localhost:" + std::to_string(base_socket_port + 1);
    std::string subSocket1 = "tcp://*:" + std::to_string(base_socket_port);
    std::cout << pubSocket1 << " " << subSocket1 << std::endl;

    std::string pubSocket2 = "tcp://localhost:" + std::to_string(base_socket_port + 3);
    std::string subSocket2 = "tcp://*:" + std::to_string(base_socket_port + 2);
    std::cout << pubSocket2 << " " << subSocket2 << std::endl;

    //pub.setsockopt(ZMQ_CONFLATE, 1);
    pub1.connect (pubSocket1);
    pub2.connect (pubSocket2);
    std::cout << "pubs connected" << std::endl;

    sub1.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub1.setsockopt(ZMQ_CONFLATE, 1);
    sub1.bind(subSocket1);
    sub2.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub2.setsockopt(ZMQ_CONFLATE, 1);
    sub2.bind(subSocket2);
    std::cout << "Sockets are ready" << std::endl;
}
