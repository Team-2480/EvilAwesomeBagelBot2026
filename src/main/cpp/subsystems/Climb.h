
#pragma once

#include "frc2/command/SubsystemBase.h"

class ClimbSubsystem : public frc::SubsystemBase {
    public:
    ClimbSubsystem() {}
    void Periodic() override;

    void SetClimb (bool climb_set);

    private:
        //motors here 
bool climb_up = false;

};