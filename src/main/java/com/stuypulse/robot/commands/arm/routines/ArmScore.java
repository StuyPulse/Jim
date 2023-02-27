package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

public class ArmScore extends ArmRoutine {
    
    public ArmScore() {
        super(Manager.getInstance()::getScoreTrajectory);
    }

    @Override
    public ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        // assumes that ready was called before score
        return new ArmTrajectory().addState(dest);
    }

}
