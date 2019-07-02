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

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <zmq.hpp>
#include <unistd.h>

using namespace libnifalcon;

void runFalconTest();

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

    runFalconTest();
    return 0;
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

    std::shared_ptr<FalconFirmware> firmware;
	unsigned int num_falcons = 0;
	unsigned int count;
	FalconDevice dev;

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

		if(!dev.open(z))
		{
			std::cout << "Cannot open falcon - Error: " << std::endl; // << dev.getErrorCode() << std::endl;
			return;
		}
		std::cout << "Opened falcon" << std::endl;

		if(!dev.isFirmwareLoaded())
		{
			std::cout << "Loading firmware" << std::endl;
            if(!dev.getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
            {
                std::cout << "Could not load firmware" << std::endl;
                return;
            }
            else
            {
                std::cout <<"Firmware loaded" << std::endl;
                break;
            }
		}

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
}
