package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.IArm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TipInHighOppositeReady extends InstantCommand {
    public TipInHighOppositeReady(IArm arm) {
        super(() -> {
            arm.setTargetShoulderAngle(-15);
            arm.setTargetWristAngle(+20, false);
        }, arm);
    }
}
