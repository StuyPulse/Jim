package com.stuypulse.robot.commands.arm;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmBFSField;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmFollowTrajectory extends CommandBase {
    
    private final Arm arm;
    private Supplier<ArmBFSField> trajectorySupplier;

    private BStream finished;
    public ArmFollowTrajectory(Supplier<ArmBFSField> trajectorySupplier) {
        arm = Arm.getInstance();
        
        this.trajectorySupplier = trajectorySupplier;

        finished = BStream.create(this::atSetpoint)
            .filtered(new BDebounceRC.Rising(0.2));

        addRequirements(arm);
    }

    public ArmFollowTrajectory(ArmBFSField trajectory) {
        this(() -> trajectory);
    }

    private ArmBFSField.Node getCurrentNode() {
        return trajectorySupplier.get().getNode(arm.getState());
    }

    private ArmState getCurrentSetpoint() {
        return getCurrentNode().travel(30).getArmState();
    }

    private boolean atSetpoint() {
        return getCurrentNode().travel(10).isSetpoint() && arm.isArmAtState(
            Rotation2d.fromDegrees(Shoulder.TOLERANCE),
            Rotation2d.fromDegrees(Wrist.TOLERANCE));
    }

    @Override
    public void execute() {
        arm.setTargetState(getCurrentSetpoint());
    }

    @Override
    public boolean isFinished() {
        return false; /// finished.get();
    }

}
