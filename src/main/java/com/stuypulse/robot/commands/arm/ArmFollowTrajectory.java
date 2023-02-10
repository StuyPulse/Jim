package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.AsstarImp;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmFollowTrajectory extends CommandBase {
    
    private final Arm arm;

    private ArmTrajectory trajectory;
    private int currentIdx;

    public ArmFollowTrajectory(ArmTrajectory trajectory) {
        arm = Arm.getInstance();
        
        this.trajectory = trajectory;
        currentIdx = 0;

        addRequirements(arm);
    }

    public ArmFollowTrajectory(Rotation2d finalShoulderAngle, Rotation2d finalWristAngle) {
        arm = Arm.getInstance();

        this.trajectory = AsstarImp.generateTrajectory(30, 120, arm.getShoulderAngle().getDegrees(), arm.getWristAngle().getDegrees(), finalShoulderAngle.getDegrees(), finalWristAngle.getDegrees())
        ;
        currentIdx = 0;

        addRequirements(arm);
    }

    public ArmFollowTrajectory() {
        this(new ArmTrajectory());
    }

    protected void setTrajectory(ArmTrajectory trajectory) {
        this.trajectory = trajectory;
    }
    
    protected ArmTrajectory getTrajectory() {
        return trajectory;
    }

    private ArmState getCurrentSetpoint() {
        return trajectory.getStates().get(currentIdx);
    }

    private boolean atSetpoint() {
        return arm.isArmAtState(
            Rotation2d.fromDegrees(Shoulder.TOLERANCE),
            Rotation2d.fromDegrees(Wrist.TOLERANCE));
    }

    @Override
    public void initialize() {
        currentIdx = 0;
    }

    @Override
    public void execute() {
        arm.setTargetState(getCurrentSetpoint());

        if (atSetpoint()) {
            currentIdx++;
        }
    }

    @Override
    public boolean isFinished() {
        return currentIdx >= trajectory.getStates().size();
    }

    @Override
    public void end(boolean i) {
    }
}
