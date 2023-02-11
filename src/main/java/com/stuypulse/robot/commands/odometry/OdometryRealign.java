package com.stuypulse.robot.commands.odometry;

import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OdometryRealign extends InstantCommand {
    private final Odometry odometry;
    private final Supplier<Rotation2d> references;

    public OdometryRealign(Supplier<Rotation2d> references) {
        odometry = Odometry.getInstance();
        this.references = references;

        // addRequirements(null);
    }

    public OdometryRealign(Rotation2d reference) {
        this(() -> reference);
    }

    @Override
    public void initialize() {
        Pose2d pose = odometry.getPose();
        odometry.reset(new Pose2d(pose.getTranslation(), references.get()));
    }

}
