package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {
    public static Translation2d getMirroredTranslation(Translation2d translation) {
        return new Translation2d(translation.getX(), Field.HEIGHT - translation.getY());
    }

    public static Pose2d getMirroredPose(Pose2d pose) {
        return new Pose2d(getMirroredTranslation(pose.getTranslation()), pose.getRotation());
    }

    public static Translation3d getMirroredTranslation(Translation3d translation) {
        var mirrored = getMirroredTranslation(translation.toTranslation2d());
        return new Translation3d(mirrored.getX(), mirrored.getY(), translation.getZ());
    }

    public static Pose3d getMirroredPose(Pose3d pose) {
        return new Pose3d(getMirroredTranslation(pose.getTranslation()), pose.getRotation());
    }

    // 2 origin system, convert from red to blue
    public static Pose2d getAlliancePose(Pose2d bluePose, Alliance alliance) {
        if (alliance == Alliance.Blue)
            return bluePose;
        
        return getMirroredPose(bluePose);
    }
}
