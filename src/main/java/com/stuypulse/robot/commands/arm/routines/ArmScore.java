package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.constants.ArmTrajectories;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmScore extends ArmRoutine {
    
    public ArmScore() {
        super(Manager.getInstance()::getScoreTrajectory);
    }

    public void initialize() {
        super.initialize();

        var state = Arm.getInstance().getState();

        trajectory =
            ArmTrajectories.generateTrajectory(
                state,
                Manager.getInstance().getScoreTrajectory());
    }
}
