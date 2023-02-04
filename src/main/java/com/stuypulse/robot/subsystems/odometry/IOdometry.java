package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IOdometry extends SubsystemBase {
    
    private static IOdometry instance;

    public static IOdometry getInstance(){
        if(instance == null) {
            instance = new Odometry();
        }
        return instance;
    }

    public abstract Field2d getField();

    public abstract void reset( Pose2d pose2d);
    
    public abstract Pose2d getPose();

    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }
}
