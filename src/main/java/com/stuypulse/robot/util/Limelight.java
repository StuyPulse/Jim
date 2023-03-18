package com.stuypulse.robot.util;

import java.util.Optional;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight {

    private String tableName;

    private final IntegerEntry idEntry;
    private final DoubleArrayEntry blueBotposeEntry;
    private final DoubleArrayEntry redBotposeEntry;
    
    private Optional<AprilTagData> data;

    private final Pose3d robotRelativePose;

    public Limelight(String tableName, Pose3d robotRelativePose) {
        this.tableName = tableName;
        this.robotRelativePose = robotRelativePose;

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);


        blueBotposeEntry = limelight.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]);
        redBotposeEntry = limelight.getDoubleArrayTopic("botpose_wpired").getEntry(new double[0]);
        idEntry = limelight.getIntegerTopic("tid").getEntry(0);

        data = Optional.empty();
    }

    public String getTableName() {
        return tableName;
    }

    public Optional<AprilTagData> getAprilTagData() {
        return data;
    }

    public boolean hasAprilTagData() {
        return getAprilTagData().isPresent();
    }

    private double[] getPoseData() {
        return RobotContainer.getCachedAlliance() == Alliance.Blue ?
            blueBotposeEntry.get() :
            redBotposeEntry.get();
    }

    public void updateAprilTagData() {
        double[] botposeData = getPoseData();

        if (botposeData.length != 7) {
            data = Optional.empty();
            return;
        }

        Pose2d botpose = new Pose2d(botposeData[0], botposeData[1], Rotation2d.fromDegrees(botposeData[5]));
        double latency = Units.millisecondsToSeconds(botposeData[6]);
        int id = (int) idEntry.get();

        if (!Field.isValidAprilTagId(id)) {
            data = Optional.empty();
            return;
        }

        data = Optional.of(new AprilTagData(botpose, latency, id, this));
    }

    public Rotation2d getRobotRelativeRotation() {
        return robotRelativePose.getRotation().toRotation2d();
    }

    public Pose2d getRobotRelativePose() {
        return robotRelativePose.toPose2d();
    }
}