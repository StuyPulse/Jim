package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;

public class ArmNeutral extends ArmFollowTrajectory {

    public ArmNeutral() {
        super(() -> Manager.getInstance().getNeutralTrajectory());
    }

}
