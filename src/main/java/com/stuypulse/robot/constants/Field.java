/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

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

    public interface Pegs {
        double RED_Y[] = {
            7.494778,
            6.935978,
            6.377178,
            5.843778,
            5.259578,
            4.700778,
            4.141978,
            3.583178,
            3.024378
        };

        double BLUE_Y[] = {
            4.9784,
            4.4196,
            3.8608,
            3.3020,
            2.7432,
            2.1844,
            1.6256,
            1.0668,
            0.5080
        };
    }

    static class RedBlueNumber extends Number { private double red, blue; public RedBlueNumber(double red, double blue) { this.red = red; this.blue = blue; } public double doubleValue() { if (RobotContainer.getCachedAlliance() == Alliance.Red) return red; return blue; } public int intValue() { return (int) doubleValue(); } public float floatValue() { return (float) doubleValue(); } public long longValue() { return (long) doubleValue(); } }

    public interface ScoreXPoses {
        public interface High {
            Number CUBE_BACK = 1.846;
            Number CUBE_FRONT = 1.825;
            Number CONE_TIP_IN = new RedBlueNumber(1.881, 1.8556);
            Number CONE_TIP_OUT = 1.772;
        }

        public interface Mid {
            Number CUBE_BACK = 1.881;
            Number CUBE_FRONT = 2.106;
            Number CONE_TIP_IN = 2.231;
            Number CONE_TIP_OUT = 2.18;
        }

        public interface Low {
            Number BACK = 1.825;
            Number FRONT = 1.825;
        } 
    }

    // red left to right
    public interface ScoreYPoses {
        public static double[] getYPoseArray(Alliance alliance, ScoreSide side) {
            if (side == ScoreSide.FRONT)
                return alliance == Alliance.Red ? Front.RED_Y_POSES : Front.BLUE_Y_POSES;
            else
                return alliance == Alliance.Red ? Back.RED_Y_POSES : Back.BLUE_Y_POSES;
        }

        public static double middleToBack(double midYPose) {
            return midYPose + Field.INTAKE_OFFSET_RIGHT;
        }

        public interface Back {
            double RED_Y_POSES[] = {
                middleToBack(Pegs.RED_Y[0]),
                middleToBack(Pegs.RED_Y[1]),
                middleToBack(Pegs.RED_Y[2]),
                middleToBack(Pegs.RED_Y[3]),
                middleToBack(Pegs.RED_Y[4]),
                middleToBack(Pegs.RED_Y[5]),
                middleToBack(Pegs.RED_Y[6]),
                middleToBack(Pegs.RED_Y[7]),
                middleToBack(Pegs.RED_Y[8])
            };

            double BLUE_Y_POSES[] = {
                middleToBack(Pegs.BLUE_Y[0]),
                middleToBack(Pegs.BLUE_Y[1]),
                middleToBack(Pegs.BLUE_Y[2]),
                middleToBack(Pegs.BLUE_Y[3]),
                middleToBack(Pegs.BLUE_Y[4]),
                middleToBack(Pegs.BLUE_Y[5]),
                middleToBack(Pegs.BLUE_Y[6]),
                middleToBack(Pegs.BLUE_Y[7]),
                middleToBack(Pegs.BLUE_Y[8])
            };
        }

        public interface Front {
            public static double middleToFront(double midYPose) {
                return midYPose - Field.INTAKE_OFFSET_RIGHT; 
            }

            double RED_Y_POSES[] = {
                middleToFront(Pegs.RED_Y[0]),
                middleToFront(Pegs.RED_Y[1]),
                middleToFront(Pegs.RED_Y[2]),
                middleToFront(Pegs.RED_Y[3]),
                middleToFront(Pegs.RED_Y[4]),
                middleToFront(Pegs.RED_Y[5]),
                middleToFront(Pegs.RED_Y[6]),
                middleToFront(Pegs.RED_Y[7]),
                middleToFront(Pegs.RED_Y[8])
            };

            double BLUE_Y_POSES[] = {
                middleToFront(Pegs.BLUE_Y[0]),
                middleToFront(Pegs.BLUE_Y[1]),
                middleToFront(Pegs.BLUE_Y[2]),
                middleToFront(Pegs.BLUE_Y[3]),
                middleToFront(Pegs.BLUE_Y[4]),
                middleToFront(Pegs.BLUE_Y[5]),
                middleToFront(Pegs.BLUE_Y[6]),
                middleToFront(Pegs.BLUE_Y[7]),
                middleToFront(Pegs.BLUE_Y[8])
            };
        }
    }

}
