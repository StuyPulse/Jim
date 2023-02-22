package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
public class ArmOuttake extends ArmRoutine {
    
    public ArmOuttake() {
        super(Routine.OUTTAKE, Manager.getInstance()::getOuttakeTrajectory);
    }
}
