package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;

public class ArmReady extends ArmFollowTrajectory {

    public ArmReady() {
        super(() -> Manager.getInstance().getReadyTrajectory());
    }
    
}
