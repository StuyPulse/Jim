package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;

public class ArmIntake extends ArmFollowTrajectory {
    
    public ArmIntake() {
        super(() -> Manager.getInstance().getIntakeTrajectory());
    }
    
}
