package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.AllianceUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Field {
    Translation2d CHARGING_STATION_CENTER = new Translation2d(172.631, 0);
    double PEG_TO_CHARGING_STATION_EDGE = Units.inchesToMeters(60.69);

    double WIDTH = 16.54;
    double HEIGHT = 8.02;

    Pose3d APRIL_TAGS[] = {
        new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0, 0 , 0)),
        new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0 , 0)),
        new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0 , 0)),
        new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0 , 0)),
    };

    public static boolean isAprilTagBlueFromId(int id) {
        return id >= 5 || id <= 8;
    }

    public static Pose3d getAprilTagFromId(int id) {
        if (isAprilTagBlueFromId(id)) {
            return APRIL_TAGS[8 - id];
        } else {
            return AllianceUtil.getMirroredPose(APRIL_TAGS[id - 1]);
        }
    }

    // blue left to right
    public interface ScorePoses {
        Pose2d ONE = new Pose2d();
        Pose2d TWO = new Pose2d();
        Pose2d THREE = new Pose2d();
        Pose2d FOUR = new Pose2d();
        Pose2d FIVE = new Pose2d();
        Pose2d SIX = new Pose2d();
        Pose2d SEVEN = new Pose2d();
        Pose2d EIGHT = new Pose2d();
        Pose2d NINE = new Pose2d();
    }

    Pose2d BLUE_ALIGN_POSES[] = {
        ScorePoses.ONE,
        ScorePoses.TWO,
        ScorePoses.THREE,
        ScorePoses.FOUR,
        ScorePoses.FIVE,
        ScorePoses.SIX,
        ScorePoses.SEVEN,
        ScorePoses.EIGHT,
        ScorePoses.NINE
    };

    Pose2d RED_ALIGN_POSES[] = {
        AllianceUtil.getMirroredPose(ScorePoses.NINE),
        AllianceUtil.getMirroredPose(ScorePoses.EIGHT),
        AllianceUtil.getMirroredPose(ScorePoses.SEVEN),
        AllianceUtil.getMirroredPose(ScorePoses.SIX),
        AllianceUtil.getMirroredPose(ScorePoses.FIVE),
        AllianceUtil.getMirroredPose(ScorePoses.FOUR),
        AllianceUtil.getMirroredPose(ScorePoses.THREE),
        AllianceUtil.getMirroredPose(ScorePoses.TWO),
        AllianceUtil.getMirroredPose(ScorePoses.ONE)
    };

}

