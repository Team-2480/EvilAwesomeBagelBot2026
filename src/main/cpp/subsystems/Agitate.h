
#pragma once

class AgitateSubsystem : public frc::SubsystemBase{
    public:
    AgitateSubsystem(){
        rev::spark::SparkBaseConfig agitate_config{};

        agitate_config.encoder.PositionConversionFactor(1).VelocityConversionFactor(1);
        
        agitate_config.closedLoop
            .SetFeedbackSensor(rev::spark::FeedbackSendor::kPrimaryEncoder)
            .Pid(0.1, 0, 0)
            .OutputRange(-1, 1);

         agitate_driver.Configure(agitate_config, rev::ResetMode::kResetSafeParameter,
                                    rev::PersistMode::kPersistParameters);




    private:
    bool agitate_on = false;
        rev::spark::SparkMax agitate_driver = 
            rev::spark::SparkMax(42, rev::spark::SparkMax::MotorType::kBrushed);
         rev::spark::SparkClosedLoopController agitate_driver_controller =
            agitate_driver.GetClosedLoopController();
    }


};