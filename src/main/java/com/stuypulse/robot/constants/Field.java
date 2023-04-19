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

    // intake offset from center to the right
    double INTAKE_OFFSET_RIGHT = Units.inchesToMeters(1.625);

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
            SmartNumber CUBE_BACK = new SmartNumber("Alignment/X Poses/High/Cube Back", 1.846);
            SmartNumber CUBE_FRONT = new SmartNumber("Alignment/X Poses/High/Cube Front", 1.825);
            SmartNumber CONE_TIP_IN = new SmartNumber("Alignment/X Poses/High/Cone Tip In", 1.881);
            SmartNumber CONE_TIP_OUT = new SmartNumber("Alignment/X Poses/High/Cone Tip Out", 1.822);
        }

        public interface Mid {
            SmartNumber CUBE_BACK = new SmartNumber("Alignment/X Poses/Mid/Cube Back", 1.881);
            SmartNumber CUBE_FRONT = new SmartNumber("Alignment/X Poses/Mid/Cube Front", 2.106);
            SmartNumber CONE_TIP_IN = new SmartNumber("Alignment/X Poses/Mid/Cone Tip In", 2.281);
            SmartNumber CONE_TIP_OUT = new SmartNumber("Alignment/X Poses/Mid/Cone Tip Out", 2.102);
        }

        public interface Low {
            SmartNumber BACK = new SmartNumber("Alignment/X Poses/Low/Back", 1.825);
            SmartNumber FRONT = new SmartNumber("Alignment/X Poses/Low/Front", 1.825);
        }
    }

    // red left to right
    public interface ScoreYPoses {
        public static Number[] getYPoseArray(Alliance alliance, ScoreSide side) {
            if (side == ScoreSide.FRONT)
                return alliance == Alliance.Red ? Front.RED_Y_POSES : Front.BLUE_Y_POSES;
            else
                return alliance == Alliance.Red ? Back.RED_Y_POSES : Back.BLUE_Y_POSES;
        }

        public static double middleToFront(double midYPose) {
            return midYPose - Field.INTAKE_OFFSET_RIGHT; 
        }

        public static double middleToBack(double midYPose) {
            return midYPose + Field.INTAKE_OFFSET_RIGHT;
        }

        public static Number redToBlue(Number redXPose) {
            return IStream.create(() -> redXPose.doubleValue() + 2 * INTAKE_OFFSET_RIGHT).number();
        }

        public interface Back {
            SmartNumber ONE =   new SmartNumber("Alignment/Y Poses/Red 1", middleToBack(7.494778));
            SmartNumber TWO =   new SmartNumber("Alignment/Y Poses/Red 2", middleToBack(6.935978));
            SmartNumber THREE = new SmartNumber("Alignment/Y Poses/Red 3", middleToBack(6.377178));
            SmartNumber FOUR =  new SmartNumber("Alignment/Y Poses/Red 4", middleToBack(5.843778));
            SmartNumber FIVE =  new SmartNumber("Alignment/Y Poses/Red 5", middleToBack(5.259578));
            SmartNumber SIX =   new SmartNumber("Alignment/Y Poses/Red 6", middleToBack(4.700778));
            SmartNumber SEVEN = new SmartNumber("Alignment/Y Poses/Red 7", middleToBack(4.14197));
            SmartNumber EIGHT = new SmartNumber("Alignment/Y Poses/Red 8", middleToBack(3.583178));
            SmartNumber NINE =  new SmartNumber("Alignment/Y Poses/Red 9", middleToBack(3.024378));

            /*
             * Theoretical values:
             * 295.07in | 7.494778m
             * 273.07in | 6.935978m
             * 251.07in | 6.377178m
             * 230.07in | 5.843778m
             * 207.07in | 5.259578m
             * 185.07in | 4.700778m
             * 163.07in | 4.141978m
             * 141.07in | 3.583178m
             * 119.07in | 3.024378m
             */
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

            /*
             * Theoretical values:
             * 196in | 4.9784m
             * 174in | 4.4196m
             * 152in | 3.8608m
             * 130in | 3.3020m
             * 108in | 2.7432m
             * 86in  | 2.1844m
             * 64in  | 1.6256m
             * 42in  | 1.0668m
             * 20in  | 0.5080m
             */
            Number BLUE_Y_POSES[] = {
                redToBlue(Back.NINE),
                redToBlue(Back.EIGHT),
                redToBlue(Back.SEVEN),
                redToBlue(Back.SIX),
                redToBlue(Back.FIVE),
                redToBlue(Back.FOUR),
                redToBlue(Back.THREE),
                redToBlue(Back.TWO),
                redToBlue(Back.ONE)
            };
        }

        public interface Front {
            private static Number backToFront(Number backYPose) {
                return IStream.create(() -> backYPose.doubleValue() - 2 * INTAKE_OFFSET_RIGHT).number();
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
                redToBlue(backToFront(Back.NINE)),
                redToBlue(backToFront(Back.EIGHT)),
                redToBlue(backToFront(Back.SEVEN)),
                redToBlue(backToFront(Back.SIX)),
                redToBlue(backToFront(Back.FIVE)),
                redToBlue(backToFront(Back.FOUR)),
                redToBlue(backToFront(Back.THREE)),
                redToBlue(backToFront(Back.TWO)),
                redToBlue(backToFront(Back.ONE))
            };
        }
    }

}
