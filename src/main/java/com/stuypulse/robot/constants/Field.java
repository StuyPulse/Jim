package com.stuypulse.robot.constants;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
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
            double CUBE_BACK = 1.98;
            double CUBE_FRONT = 1.830060;
            double CONE_TIP_IN = 1.894;
            double CONE_TIP_OUT = 1.783;
        }

        public interface Mid {
            double CUBE_BACK = 1.868;
            double CUBE_FRONT = 2.083577;
            double CONE_TIP_IN = 2.275;
            double CONE_TIP_OUT = 2.1433;
        }

        // Low Cube: 1.768088
    }

    // red left to right
    public interface ScoreYPoses {
        public static double[] getYPoseArray(Alliance alliance, ScoreSide side) {
            if (side == ScoreSide.FRONT)
                return alliance == Alliance.Red ? Front.RED_Y_POSES : Front.BLUE_Y_POSES;
            else
                return alliance == Alliance.Red ? Back.RED_Y_POSES : Back.BLUE_Y_POSES;
        }

        public interface Back {
            double ONE =   7.4376;
            double TWO =   6.905;
            double THREE = 6.3238;
            double FOUR =  5.822;
            double FIVE =  5.2947;
            double SIX =   4.6;
            double SEVEN = 4.1028;
            double EIGHT = 3.557;
            double NINE =  2.89;
    
            double RED_Y_POSES[] = {
                Back.ONE,
                Back.TWO,
                Back.THREE,
                Back.FOUR,
                Back.FIVE,
                Back.SIX,
                Back.SEVEN,
                Back.EIGHT,
                Back.NINE
            };
    
            double BLUE_Y_POSES[] = {
                AllianceUtil.getMirroredYPose(Back.NINE),
                AllianceUtil.getMirroredYPose(Back.EIGHT),
                AllianceUtil.getMirroredYPose(Back.SEVEN),
                AllianceUtil.getMirroredYPose(Back.SIX),
                AllianceUtil.getMirroredYPose(Back.FIVE),
                AllianceUtil.getMirroredYPose(Back.FOUR),
                AllianceUtil.getMirroredYPose(Back.THREE),
                AllianceUtil.getMirroredYPose(Back.TWO),
                AllianceUtil.getMirroredYPose(Back.ONE)
            };
        }

        public interface Front {
            private static double backToFront(double backYPose) {
                return backYPose - Units.inchesToMeters(3.0);
            }
    
            double RED_Y_POSES[] = {
                backToFront(Back.ONE),
                backToFront(Back.TWO),
                backToFront(Back.THREE),
                backToFront(Back.FOUR),
                backToFront(Back.FIVE),
                backToFront(Back.SIX),
                backToFront(Back.SEVEN),
                backToFront(Back.EIGHT),
                backToFront(Back.NINE)
            };
    
            double BLUE_Y_POSES[] = {
                AllianceUtil.getMirroredYPose(backToFront(Back.NINE)),
                AllianceUtil.getMirroredYPose(backToFront(Back.EIGHT)),
                AllianceUtil.getMirroredYPose(backToFront(Back.SEVEN)),
                AllianceUtil.getMirroredYPose(backToFront(Back.SIX)),
                AllianceUtil.getMirroredYPose(backToFront(Back.FIVE)),
                AllianceUtil.getMirroredYPose(backToFront(Back.FOUR)),
                AllianceUtil.getMirroredYPose(backToFront(Back.THREE)),
                AllianceUtil.getMirroredYPose(backToFront(Back.TWO)),
                AllianceUtil.getMirroredYPose(backToFront(Back.ONE))
            };
        }
    }

}

