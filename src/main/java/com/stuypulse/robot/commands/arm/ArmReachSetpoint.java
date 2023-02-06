package com.stuypulse.robot.commands.arm;

import static com.stuypulse.robot.constants.Settings.Arm.*;
import com.stuypulse.robot.subsystems.arm.IArm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmReachSetpoint extends CommandBase {

    private final IArm arm;

    private final Rotation2d targetShoulderAngle;
    private final Rotation2d targetWristAngle;

    public ArmReachSetpoint(Rotation2d targetShoulderAngle, Rotation2d targetWristAngle) {
        arm = IArm.getInstance();

        this.targetShoulderAngle = targetShoulderAngle;
        this.targetWristAngle = targetWristAngle;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTargetShoulderAngle(targetShoulderAngle);
        arm.setTargetWristAngle(targetWristAngle);
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(Rotation2d.fromDegrees(Shoulder.TOLERANCE))
                && arm.isWristAtAngle(Rotation2d.fromDegrees((Wrist.TOLERANCE)));
    }
}
