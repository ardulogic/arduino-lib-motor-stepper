// Hardware Motors
#include <MotorStepper.h>

MotorStepper MOTOR_MAIN = MotorStepper(10, 11);

void setup() {   
    // Init Motors
    MOTOR_MAIN.Init();

    // void MotorWaypoints::Set(uint8_t idx, long position, int frequency, unsigned int delay_ms, bool enabled)
    MotorStepper::Waypoints()->Set(0, 5000, 5000, 100, true);
    MotorStepper::SetMode(MODE_WAYPOINT);
    MotorStepper::Run();
}

void loop(void) {
    
}
