// /************************ PROJECT JIM *************************/
// /* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
// /* This work is licensed under the terms of the MIT license.  */
// /**************************************************************/

// package com.stuypulse.robot.commands.swerve.balance;

// import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
// import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;

// public class SwerveDriveBalanceAlign extends SwerveDriveToPose {
//     public SwerveDriveBalanceAlign() {
//         super(() -> {
//             var odometry = AbstractOdometry.getInstance();
//             return new Pose2d(odometry.getTranslation(), getSnappedAngle(odometry.getRotation()));
//         });
//     }

//     private static Rotation2d getSnappedAngle(Rotation2d angle) {
//         double currentAngle = angle.getDegrees();
//         // angle
//         if (currentAngle > -45 && currentAngle <= 45) {
//             currentAngle = 0;
//         } else if (currentAngle > 45 && currentAngle <= 135) {
//             currentAngle = 90;
//         } else if (currentAngle < -45 && currentAngle >= -135) {
//             currentAngle = -90;
//         } else if (currentAngle < -135 && currentAngle >= 135 ) {
//             currentAngle = 180;
//         }

//         return Rotation2d.fromDegrees(currentAngle);
//     }
// }
