package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmTrajectory;

public interface ArmTrajectories {
    ArmTrajectory NEUTRAL = new ArmTrajectory().addState(-90, 90);

    public interface High {
        ArmTrajectory CONE_READY = new ArmTrajectory().addState(0, 0);
        ArmTrajectory CUBE_READY = new ArmTrajectory().addState(0, 0);
    }
}
