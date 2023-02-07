package com.stuypulse.robot.commands.odometry;

import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OdometryReset extends InstantCommand {
    private final Odometry odometry;
    private final Supplier<Pose2d> references;

    public OdometryReset(Supplier<Pose2d> references) {
        odometry = Odometry.getInstance();
        this.references = references;

        // addRequirements(null);
    }

    public OdometryReset(Pose2d reference) {
        this(() -> reference);
    }

    @Override
    public void initialize() {
        odometry.reset(references.get());
    }

}
