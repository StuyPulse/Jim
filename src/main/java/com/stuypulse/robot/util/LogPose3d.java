package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LogPose3d {
    public static void logPose3d(String name, Pose3d pose) {
        SmartDashboard.putNumberArray(name, new double[] {
           pose.getX(),
           pose.getY(),
           pose.getZ(),
           pose.getRotation().getQuaternion().getW(), 
           pose.getRotation().getQuaternion().getX(),
           pose.getRotation().getQuaternion().getY(),
           pose.getRotation().getQuaternion().getZ(),
        });
    }
}
