#include "falcon_json.h"

using namespace libnifalcon;

[[noreturn]] void pubThread (void)
{
    nlohmann::json j;
    zmq::context_t context;
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5563");

    initializeFalcon();

    while (1) {
        populateJsonDeviceCurrentPosition(j);
        zmq::message_t contents(j.dump());
        publisher.send(contents, zmq::send_flags::none);
        usleep(10000);
    }
}

[[noreturn]] void subThread(void)
{
    zmq::context_t context;
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.connect("tcp://localhost:5563");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    while (1) {
        zmq::message_t contents;
        subscriber.recv(contents, zmq::recv_flags::none);
        std::string str(static_cast<char*>(contents.data()), contents.size());
        nlohmann::json j = nlohmann::json::parse(str);
        std::cout << j.dump() << std::endl;
    }
}

int main()
{
    std::thread pub_thread(pubThread);
    std::thread sub_thread(subThread);
    pub_thread.join();  //Should never complete
    sub_thread.join();  //Should never complete
}



