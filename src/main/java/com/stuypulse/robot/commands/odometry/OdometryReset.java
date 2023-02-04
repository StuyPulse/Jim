package com.stuypulse.robot.commands.odometry;

import com.stuypulse.robot.subsystems.odometry.IOdometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OdometryReset extends InstantCommand {
    
    public OdometryReset() {
        super(() -> IOdometry.getInstance().reset(new Pose2d(3.12,0.76, new Rotation2d(Units.degreesToRadians(0))))
        );
    }

}
