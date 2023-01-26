package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.IArm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TipInHighOppositeScore extends InstantCommand {
    public TipInHighOppositeScore(IArm arm) {
        arm.setTargetShoulderAngle(+15);
        arm.setTargetWristAngle(-20);
    }
}
