package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
public class ArmManual extends ArmRoutine {
    
    public ArmManual() {
        super(Routine.SCORE, Manager.getInstance()::getManualControl);
    }
}
