/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
import com.stuypulse.robot.util.AllianceUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
            SmartNumber CUBE_BACK = new SmartNumber("Alignment/X Poses/High/Cube Back", 1.98);
            SmartNumber CUBE_FRONT = new SmartNumber("Alignment/X Poses/High/Cube Front", 1.830060);
            SmartNumber CONE_TIP_IN = new SmartNumber("Alignment/X Poses/High/Cone Tip In", 1.894 - 0.075);
            SmartNumber CONE_TIP_OUT = new SmartNumber("Alignment/X Poses/High/Cone Tip Out", 1.82 - 0.05);
        }

        public interface Mid {
            SmartNumber CUBE_BACK = new SmartNumber("Alignment/X Poses/Mid/Cube Back", 1.868);
            SmartNumber CUBE_FRONT = new SmartNumber("Alignment/X Poses/Mid/Cube Front", 2.083577);
            SmartNumber CONE_TIP_IN = new SmartNumber("Alignment/X Poses/Mid/Cone Tip In", 2.275 - 0.075);
            SmartNumber CONE_TIP_OUT = new SmartNumber("Alignment/X Poses/Mid/Cone Tip Out", 2.1433 - 0.1);
        }

        public interface Low {
            SmartNumber BACK = new SmartNumber("Alignment/X Poses/Low/Back", 1.85);
            SmartNumber FRONT = new SmartNumber("Alignment/X Poses/Low/Front", 1.85);
        }

        // Low Cube: 1.768088
    }

    // red left to right
    public interface ScoreYPoses {
        public static Number[] getYPoseArray(Alliance alliance, ScoreSide side) {
            if (side == ScoreSide.FRONT)
                return alliance == Alliance.Red ? Front.RED_Y_POSES : Front.BLUE_Y_POSES;
            else
                return alliance == Alliance.Red ? Back.RED_Y_POSES : Back.BLUE_Y_POSES;
        }

        public static Number redToBlueNYC(Number yPose) {
            return IStream.create(() -> yPose.doubleValue() - Units.inchesToMeters(3.0)).number();
        }

        public interface Back {
            SmartNumber ONE =   new SmartNumber("Alignment/Y Poses/Red 1", 7.4376);
            SmartNumber TWO =   new SmartNumber("Alignment/Y Poses/Red 2", 6.905);
            SmartNumber THREE = new SmartNumber("Alignment/Y Poses/Red 3", 6.3238);
            SmartNumber FOUR =  new SmartNumber("Alignment/Y Poses/Red 4", 5.822);
            SmartNumber FIVE =  new SmartNumber("Alignment/Y Poses/Red 5", 5.2947);
            SmartNumber SIX =   new SmartNumber("Alignment/Y Poses/Red 6", 4.6);
            SmartNumber SEVEN = new SmartNumber("Alignment/Y Poses/Red 7", 4.1028);
            SmartNumber EIGHT = new SmartNumber("Alignment/Y Poses/Red 8", 3.557);
            SmartNumber NINE =  new SmartNumber("Alignment/Y Poses/Red 9", 2.89);

            Number RED_Y_POSES[] = {
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

            Number BLUE_Y_POSES[] = {
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.NINE)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.EIGHT)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.SEVEN)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.SIX)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.FIVE)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.FOUR)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.THREE)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.TWO)),
                redToBlueNYC(AllianceUtil.getMirroredYPose(Back.ONE))
            };
        }

        public interface Front {
            private static Number backToFront(Number backYPose) {
                return IStream.create(() -> backYPose.doubleValue() - Units.inchesToMeters(3.0)).number();
            }

            Number RED_Y_POSES[] = {
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

            Number BLUE_Y_POSES[] = {
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.NINE))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.EIGHT))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.SEVEN))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.SIX))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.FIVE))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.FOUR))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.THREE))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.TWO))),
                redToBlueNYC(AllianceUtil.getMirroredYPose(backToFront(Back.ONE)))
            };
        }
    }

}
