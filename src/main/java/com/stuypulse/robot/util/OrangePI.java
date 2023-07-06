package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

public class OrangePI implements Camera {

    private String deviceID;

    private final IntegerEntry idEntry;
    private final DoubleArrayEntry robotPose;

    private final Pose3d robotRelativePose;

    private Optional<AprilTagData> data;

    public OrangePI(String deviceID, Pose3d robotRelativePose) {
        this.deviceID = deviceID;
        this.robotRelativePose = robotRelativePose;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(deviceID);

        robotPose = table.getDoubleArrayTopic("robot_pose").getEntry(new double[0]);
        idEntry = table.getIntegerTopic("tid").getEntry(0);
    }

    public String getTableName() {
        return deviceID;
    }

    public Optional<AprilTagData> getAprilTagData() {
        return data;
    }

    public boolean hasAprilTagData() {
        return getAprilTagData().isPresent();
    }

    private double[] getPoseData() {
        return robotPose.get();
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

        data = Optional.of(new AprilTagData(botpose, latency, id, robotRelativePose));
    }

}
