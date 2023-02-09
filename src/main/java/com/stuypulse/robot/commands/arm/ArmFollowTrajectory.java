package com.stuypulse.robot.commands.arm;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmFollowTrajectory extends CommandBase {
    
    private final Arm arm;
    private Supplier<ArmTrajectory> trajectorySupplier;

    private ArmTrajectory trajectory;
    private int currentIdx;

    public ArmFollowTrajectory(Supplier<ArmTrajectory> trajectorySupplier) {
        arm = Arm.getInstance();
        
        this.trajectorySupplier = trajectorySupplier;
        currentIdx = 0;

        addRequirements(arm);
    }

    public ArmFollowTrajectory(ArmTrajectory trajectory) {
        this(() -> trajectory);
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

        trajectory = trajectorySupplier.get();
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
