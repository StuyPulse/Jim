package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDriveBalanceAlign extends SwerveDriveToPose {
    public SwerveDriveBalanceAlign() {
        super(() -> {
            var odometry = Odometry.getInstance();
            return new Pose2d(odometry.getTranslation(), getSnappedAngle(odometry.getRotation()));
        });
    }
    
    private static Rotation2d getSnappedAngle(Rotation2d angle) {
        double currentAngle = angle.getDegrees();
        // angle
        if (currentAngle > -45 && currentAngle <= 45) {
            currentAngle = 0;
        } else if (currentAngle > 45 && currentAngle <= 135) {
            currentAngle = 90;
        } else if (currentAngle < -45 && currentAngle >= -135) {
            currentAngle = -90;
        } else if (currentAngle < -135 && currentAngle >= 135 ) {
            currentAngle = 180;
        }

        return Rotation2d.fromDegrees(currentAngle);
    }
}
