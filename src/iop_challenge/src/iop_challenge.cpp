/**
 */

#include <iostream>

#include "jaus/core/Component.h"

#define SUBSYSTEM_ID 155            // The subsystem ID of our robot
#define JTC_SUBSYSTEM_ID 42         // The subsystem ID of the JTC
#define JTC_ADDRESS "192.168.1.42"  // The IP address of the JTC

using namespace std;

int main() {
    // Create and initialize the component
    JAUS::Component component;
    component.AccessControlService()->SetTimeoutPeriod(0);
    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "Segway RMP");
    if (!component.Initialize(JAUS::Address(SUBSYSTEM_ID, 1, 1))) {
        cout << "Failed to initialize component" << endl;
        return 0;
    }
    component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);

    while (component.ManagementService()->GetStatus() != JAUS::Management::Status::Shutdown) {
    }

    // Cleanup
    component.Shutdown();
    return 0;
}
