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

public class Limelight {
    
    private static double kCaptureDelayMs = 11.0;

    private String tableName;

    private final DoubleEntry latencyEntry;
    private final IntegerEntry idEntry;

    private DoubleArrayEntry botposeEntry;
    
    private Alliance alliance;

    public Limelight(String tableName) {
        this.tableName = tableName;

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        latencyEntry = limelight.getDoubleTopic("tl").getEntry(0);
        idEntry = limelight.getIntegerTopic("tid").getEntry(0);

        alliance = DriverStation.getAlliance();
    }

    private void updateAlliance() {
        alliance = RobotContainer.getCachedAlliance();

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        if (alliance == Alliance.Blue) {
            botposeEntry = limelight.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[] {});
        } else {
            botposeEntry = limelight.getDoubleArrayTopic("botpose_wpired").getEntry(new double[] {});
        }
    }

    public String getTableName() {
        return tableName;
    }

    public Optional<AprilTagData> getPoseData() {
        if (RobotContainer.getCachedAlliance() != alliance)
            updateAlliance();

        double[] botposeData = botposeEntry.get();
        
        if (botposeData.length != 6) {
            return Optional.empty();
        }

        Pose2d botpose = new Pose2d(botposeData[0], botposeData[1], Rotation2d.fromDegrees(botposeData[5]));
        double latency = Units.millisecondsToSeconds(latencyEntry.get() + kCaptureDelayMs);
        int id = (int) idEntry.get();
        
        return Optional.of(new AprilTagData(botpose, latency, id));
    }

}
