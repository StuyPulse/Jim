package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;

public class ArmScore extends ArmFollowTrajectory {

    public ArmScore() {
        super(() -> Manager.getInstance().getScoreTrajectory());
    }

}
