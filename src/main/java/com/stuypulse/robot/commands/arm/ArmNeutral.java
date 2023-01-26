package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.IArm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmNeutral extends InstantCommand {
    public ArmNeutral(IArm arm) {
        super(() -> {
            arm.setTargetShoulderAngle(-90);
            arm.setTargetWristAngle(+90, false);
        }, arm);
    }
}
