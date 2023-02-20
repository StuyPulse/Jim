package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDriveBalanceAlign extends SwerveDriveToPose {
    public SwerveDriveBalanceAlign() {
        super(() -> new Pose2d(Odometry.getInstance().getTranslation(), new Rotation2d(getAngle())));
    }
    
    private static double getAngle() {
        double angle = 0;

        double currentAngle = Odometry.getInstance().getRotation().getDegrees();
        // angle
        if (currentAngle > -45 && currentAngle <= 45) {
            angle = 0;
        } else if (currentAngle > 45 && currentAngle <= 135) {
            angle = 90;
        } else if (currentAngle < -45 && currentAngle >= -135) {
            angle = -90;
        } else if (currentAngle < -135 && currentAngle >= 135 ) {
            angle = 180;
        }

        return angle;
    }
}
