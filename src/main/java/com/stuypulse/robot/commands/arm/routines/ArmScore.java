package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
public class ArmScore extends ArmRoutine {
    
    public ArmScore() {
        super(Routine.SCORE, Manager.getInstance()::getScoreTrajectory);
    }
}
