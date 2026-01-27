
#pragma once

#include "frc2/command/SubsystemBase.h"

class ClimbSubsystem : public frc::SubsystemBase {
    public:
    ClimbSubsystem() {}
    void Periodic() override;

    void SetClimb (bool climb_set);

    bool climb_up = false;
    bool climb_set = false;

    private:
        //motors here 

};
