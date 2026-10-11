#include <iostream>
#include <unistd.h>

#include "drone_state.hpp"
#include "mavlink_receiver.hpp"
#include "mavlink_sender.hpp"
#include "pso_interface.hpp"

int main()
{
    
    DroneState state;
    
    MAVLinkSender sender;

    MAVLinkReceiver receiver;

    if(!receiver.init())
    {
        std::cout
            << "Error UART\n";

        return -1;
    }

    if(!sender.init())
    {
        std::cout
            << "Error initializing MAVLink sender\n";

        return -1;
    }

    std::cout
        << "Esperando PX4..."
        << std::endl;

    PSOInterface pso;
        while(true)
    {
        receiver.receive(state);

        auto solution = pso.runPSO(state);
        sender.sendPSOCommand(solution);
        
        sender.sendHeartbeat();

        std::array<double, 9> pso_values = solution;

        std::cout
            << "PSO: "
            << pso_values[0] << " "
            << pso_values[1] << " "
            << pso_values[2] << " "
            << pso_values[3] << " "
            << pso_values[4] << " "
            << pso_values[5] << " "
            << pso_values[6] << " "
            << pso_values[7] << " "
            << pso_values[8]
            << std::endl;
            /*<< "Roll: "
            << state.roll
            << " Pitch: "
            << state.pitch
            << " Yaw: "
            << state.yaw
            << " LAT:" << state.lat
            << " LON:" << state.lon
            << " ALT:" << state.alt
            << " BARO:" << state.pressure
            << " AIR:" << state.airspeed
            << " RC1:" << state.rc_channels[0]
            << "HEARTBEAT ENVIADO" << state.flight_mode
            << std::endl; */

        usleep(100000);
    }

    return 0;
}