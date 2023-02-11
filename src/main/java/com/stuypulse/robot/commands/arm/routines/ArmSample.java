package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmSample extends ArmFollowTrajectory {

    public ArmSample(Arm arm) {
        super(() -> Manager.getInstance().getSampleTrajectory(arm));
    }
    
}
