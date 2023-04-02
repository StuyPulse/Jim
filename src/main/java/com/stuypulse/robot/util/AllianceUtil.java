package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {
    public static Number getMirroredYPose(Number y) {
        return IStream.create(() -> Field.HEIGHT - y.doubleValue()).number();
    }

    public static Translation2d getMirroredTranslation(Translation2d translation) {
        return new Translation2d(translation.getX(), getMirroredYPose(translation.getY()).doubleValue());
    }

    public static Pose2d getMirroredPose(Pose2d pose) {
        return new Pose2d(getMirroredTranslation(pose.getTranslation()), pose.getRotation());
    }

    public static Translation2d getReflectedTranslation(Translation2d translation) {
        return new Translation2d(Field.WIDTH - translation.getX(), translation.getY());
    }

    public static Pose2d getReflectedPose(Pose2d pose) {
        return new Pose2d(getReflectedTranslation(pose.getTranslation()), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // 2 origin system, convert from red to blue
    public static Pose2d getAlliancePose(Pose2d bluePose, Alliance alliance) {
        if (alliance == Alliance.Blue)
            return bluePose;
        
        return getMirroredPose(bluePose);
    }
}
