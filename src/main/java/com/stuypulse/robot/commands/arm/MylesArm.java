package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.IArm;
import com.stuypulse.robot.util.ArmState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MylesArm extends CommandBase {
    
    private final IArm arm;

    private final ArmState[] setpoints;
    private int currentIdx;

    public MylesArm(ArmState... setpoints) {
        this.setpoints = setpoints;
        arm = IArm.getInstance();
        currentIdx = 0;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        currentIdx = 0;
        // arm.setTargetShoulderAngle(setpoints[0].getShoulderRotation());
        // arm.setTargetWristAngle(setpoints[0].getWristRotation());
    }

    private ArmState getCurrentSetpoint() {
        return setpoints[currentIdx];
    }

    @Override
    public void execute() {
        ArmState setpoint = getCurrentSetpoint();
        // arm.setTargetShoulderAngle(setpoint.getShoulderRotation());
        // arm.setTargetWristAngle(setpoint.getWristRotation());

        if (arm.isShoulderAtAngle(Rotation2d.fromDegrees(4)) && arm.isWristAtAngle(Rotation2d.fromDegrees(4))) {
            currentIdx++;
        }
    }

    @Override
    public boolean isFinished() {
        return currentIdx >= setpoints.length;
    }

    @Override
    public void end(boolean i) {
        System.out.println("THIS COMMAND HAS ENDED");
    }
}
