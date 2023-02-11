package com.stuypulse.robot.commands.arm;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.AstarImp;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmFollowTrajectory extends CommandBase {
    
    private final Arm arm;
    private Supplier<ArmTrajectory> trajectorySupplier;

    private StopWatch timer;
    private ArmTrajectory trajectory;

    public ArmFollowTrajectory(Supplier<ArmTrajectory> trajectorySupplier) {
        arm = Arm.getInstance();
        
        timer = new StopWatch();
        this.trajectorySupplier = trajectorySupplier;

        addRequirements(arm);
    }

    public ArmFollowTrajectory(Rotation2d finalShoulderAngle, Rotation2d finalWristAngle) {
        arm = Arm.getInstance();

        timer =  new StopWatch();
        this.trajectory = AstarImp.generateTrajectory(arm.getShoulderAngle().getDegrees(), arm.getWristAngle().getDegrees(), finalShoulderAngle.getDegrees(), finalWristAngle.getDegrees());

        addRequirements(arm);
    }

    public ArmFollowTrajectory(ArmTrajectory trajectory) {
        this(() -> trajectory);
    }
    
    protected ArmTrajectory getTrajectory() {
        return trajectory;
    }

    @Override
    public void initialize() {
        timer.reset();
        trajectory = trajectorySupplier.get();
    }

    @Override
    public void execute() {
        Optional<ArmState> state = trajectory.calculate(timer.getTime());

        if (state.isPresent()) {
            arm.setTargetState(state.get());
        } else {
            System.out.println("DONE");
        }
    }

    @Override
    public boolean isFinished() {
        return trajectory.calculate(timer.getTime()).isEmpty();
    }

}
