/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import static com.stuypulse.robot.constants.Settings.Vision.Limelight.*;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

public class Limelight {

    public enum DataType {
        APRIL_TAG(0),
        TAPE(1),
        CUBE_DETECTION(2);

        public int type;
        private DataType(int type) {
            this.type = type;
        }

        public boolean isAprilTag() {
            return type == 0;
        }

        public boolean isReflectiveTape() {
            return type == 1;
        }

        public boolean isCubeDetection() {
            return type == 2;
        }
    }


    private final String tableName;
    private final IntegerEntry idEntry;
    private final DoubleArrayEntry blueBotposeEntry;
    private final DoubleArrayEntry redBotposeEntry;
    
    private final DoubleEntry txEntry;
    private final DoubleEntry tyEntry;
    private final IntegerEntry tvEntry;
    private final IntegerEntry pipelineEntry;

    private final int limelightId;

    private Optional<AprilTagData> data;

    private final Pose3d robotRelativePose;

    public Limelight(String tableName, Pose3d robotRelativePose) {
        this.tableName = tableName;
        this.robotRelativePose = robotRelativePose;

        limelightId = tableName == FIRST_LIMELIGHT ? 0 : 1;

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        blueBotposeEntry = limelight.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]);
        redBotposeEntry = limelight.getDoubleArrayTopic("botpose_wpired").getEntry(new double[0]);
        idEntry = limelight.getIntegerTopic("tid").getEntry(0);

        txEntry = limelight.getDoubleTopic("tx").getEntry(0.0);
        tyEntry = limelight.getDoubleTopic("ty").getEntry(0.0);
        tvEntry = limelight.getIntegerTopic("tv").getEntry(0);

        pipelineEntry = limelight.getIntegerTopic("pipeline").getEntry(0);

        data = Optional.empty();
    }

    public String getTableName() {
        return tableName;
    }

    public DataType getPipeline() {
        switch((int)pipelineEntry.get()) {
            case 0: return DataType.APRIL_TAG;
            case 1: return DataType.TAPE;
            case 2: return DataType.CUBE_DETECTION;
            default: return DataType.APRIL_TAG;
        }
    }

    public void setPipeline(DataType type) {
        NetworkTableInstance.getDefault().getTable(tableName).getEntry("pipeline").setNumber(type.type);
    }

    public Optional<AprilTagData> getAprilTagData() {
        return data;
    }

    public boolean hasAprilTagData() {
        return getAprilTagData().isPresent();
    }

    public boolean hasReflectiveTapeData() {
        return tvEntry.get() == 1 && getPipeline().isReflectiveTape();
    }

    public boolean hasCubeDetectionData() {
        return tvEntry.get() == 1 && getPipeline().isCubeDetection();
    }

    private double[] getPoseData() {
        return RobotContainer.getCachedAlliance() == Alliance.Blue ?
            blueBotposeEntry.get() :
            redBotposeEntry.get();
    }

    public double getXAngle() {
        if(hasAprilTagData()) {
            return Double.NaN;
        }
        return txEntry.get() + Units.radiansToDegrees(POSITIONS[limelightId].getRotation().getZ());
    }

    public double getYAngle() {
        if(hasAprilTagData()) {
            return Double.NaN;
        }
        return tyEntry.get() + Units.radiansToDegrees(POSITIONS[limelightId].getRotation().getY());
    }

    public double getDistanceToPeg() {
        if(hasAprilTagData()) {
            return Double.NaN;
        }

        double heightDiff = Math.abs(POSITIONS[limelightId].getZ() - 
                        (Manager.getInstance().getNodeLevel() == NodeLevel.HIGH ? Field.HIGH_PEG_HEIGHT : Field.MID_PEG_HEIGHT));

        return heightDiff / Math.tan(getYAngle()) + POSITIONS[limelightId].getX();
    }

    public double getDistanceToCube() {
        // assuming cube Z = 0
        if(hasCubeDetectionData()) {
            return POSITIONS[limelightId].getZ() / Math.abs(Rotation2d.fromDegrees(getYAngle()).getTan()) + POSITIONS[limelightId].getX();
        }
        return Double.NaN;
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