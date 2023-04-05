package com.stuypulse.robot.commands.odometry;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OdometryToggleDataAuton extends InstantCommand {

    private final boolean useAprilTags;

    public OdometryToggleDataAuton(boolean useAprilTags) {
        this.useAprilTags = useAprilTags;
    }

    public void initialize() {
        Odometry.USE_APRIL_TAGS_AUTON.set(useAprilTags);
    }
}
