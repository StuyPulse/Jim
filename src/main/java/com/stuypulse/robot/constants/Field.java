package com.stuypulse.robot.constants;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.util.AllianceUtil;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface Field {
    SmartNumber CHARGING_STATION_CENTER = new SmartNumber("Field/Charging Station Center", 172.631 - 14);
    double PEG_TO_CHARGING_STATION_EDGE = Units.inchesToMeters(60.69);

    double GRID_DEPTH = Units.inchesToMeters(54.25);

    double WIDTH = 16.54;
    double HEIGHT = 8.02;

    Pose2d BLUE_APRIL_TAGS[] = {
        // 1-4
        new Pose2d(WIDTH - Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(180)),
        new Pose2d(WIDTH - Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(180)),
        new Pose2d(WIDTH - Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(180)),
        new Pose2d(WIDTH - Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(180)),

        // 5-8
        new Pose2d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), new Rotation2d(0)),
        new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19),  new Rotation2d(0)),
    };

    public static boolean isValidAprilTagId(int id) {
        return id >= 1 && id <= 8;
    }

    public static Pose2d getAprilTagFromId(int id) {
        if (RobotContainer.getCachedAlliance() == Alliance.Blue) {
            return BLUE_APRIL_TAGS[id - 1];
        } else {
            return AllianceUtil.getMirroredPose(BLUE_APRIL_TAGS[8 - id]);
        }
    }

    public interface ScoreXPoses {
        public interface High {
            double CUBE = 1.98;
            double CONE_TIP_IN = 2.0;
            double CONE_TIP_OUT = 1.783;
        }

        public interface Mid {
            double CUBE = 2.042;
            double CONE_TIP_IN = 2.052;
            double CONE_TIP_OUT = 2.184;
        }
    }

    // red left to right
    public interface ScoreYPoses {
        double ONE =   7.4367;
        double TWO =   6.905;
        double THREE = 6.3238;
        double FOUR =  5.7976;
        double FIVE =  5.2947;
        double SIX =   4.666;
        double SEVEN = 4.035;
        double EIGHT = 3.557;
        double NINE =  2.89;

        double RED_Y_POSES[] = {
            ScoreYPoses.ONE,
            ScoreYPoses.TWO,
            ScoreYPoses.THREE,
            ScoreYPoses.FOUR,
            ScoreYPoses.FIVE,
            ScoreYPoses.SIX,
            ScoreYPoses.SEVEN,
            ScoreYPoses.EIGHT,
            ScoreYPoses.NINE
        };

        double BLUE_Y_POSES[] = {
            AllianceUtil.getMirroredYPose(ScoreYPoses.NINE),
            AllianceUtil.getMirroredYPose(ScoreYPoses.EIGHT),
            AllianceUtil.getMirroredYPose(ScoreYPoses.SEVEN),
            AllianceUtil.getMirroredYPose(ScoreYPoses.SIX),
            AllianceUtil.getMirroredYPose(ScoreYPoses.FIVE),
            AllianceUtil.getMirroredYPose(ScoreYPoses.FOUR),
            AllianceUtil.getMirroredYPose(ScoreYPoses.THREE),
            AllianceUtil.getMirroredYPose(ScoreYPoses.TWO),
            AllianceUtil.getMirroredYPose(ScoreYPoses.ONE)
        };
    }

}

