package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {
    public static Pose2d mirroredPose(Pose2d pose) {
        return new Pose2d(pose.getX(), Field.HEIGHT - pose.getY(), pose.getRotation());
    }

    // 2 origin system, convert from red to blue
    public static Pose2d getAlliancePose(Pose2d bluePose, Alliance alliance) {
        if (alliance == Alliance.Blue)
            return bluePose;
        
        return mirroredPose(bluePose);
    }
}
