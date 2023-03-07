package com.stuypulse.robot.util;

import java.util.Optional;

import com.stuypulse.robot.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    
    private static double kCaptureDelayMs = 11.0;

    private String tableName;

    private final DoubleEntry latencyEntry;
    private final IntegerEntry idEntry;
    private final DoubleArrayEntry blueBotposeEntry;
    private final DoubleArrayEntry redBotposeEntry;
    
    private Optional<AprilTagData> data;

    public Limelight(String tableName) {
        this.tableName = tableName;

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);
        latencyEntry = limelight.getDoubleTopic("tl").getEntry(0);
        idEntry = limelight.getIntegerTopic("tid").getEntry(0);

        blueBotposeEntry = limelight.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]);
        redBotposeEntry = limelight.getDoubleArrayTopic("botpose_wpired").getEntry(new double[0]);

        data = Optional.empty();
    }

    public String getTableName() {
        return tableName;
    }

    public Optional<AprilTagData> getAprilTagData() {
        return data;
    }

    public void updateAprilTagData() {
        double[] botposeData;
        if (RobotContainer.getCachedAlliance() == Alliance.Blue) {
            botposeData = blueBotposeEntry.get();
        } else {
            botposeData = redBotposeEntry.get();
        }

        if (botposeData.length != 7) {
            data = Optional.empty();
            return;
        }

        Pose2d botpose = new Pose2d(botposeData[0], botposeData[1], Rotation2d.fromDegrees(botposeData[5]));
        double latency = botposeData[6];
        int id = (int) idEntry.get();

        SmartDashboard.putNumber("Limelight/" + getTableName() + "/Pose X", botpose.getX());
        SmartDashboard.putNumber("Limelight/" + getTableName() + "/Pose Y", botpose.getY());
        SmartDashboard.putNumber("Limelight/" + getTableName() + "/Pose Rotation (Deg)", botpose.getRotation().getDegrees());
        SmartDashboard.putNumber("Limelight/" + getTableName() + "/Tag ID", id);
        SmartDashboard.putNumber("Limelight/" + getTableName() + "/Latency", latency);

        
        data = Optional.of(new AprilTagData(botpose, latency, id));
    }

}