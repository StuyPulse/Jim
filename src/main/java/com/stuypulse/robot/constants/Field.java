package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.AllianceUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Field {
    Translation2d CHARGING_STATION_CENTER = new Translation2d(172.631, 0);
    double PEG_TO_CHARGING_STATION_EDGE = Units.inchesToMeters(60.69);

    double GRID_DEPTH = Units.inchesToMeters(54.25);

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
        double DEFAULT_X_DIST = GRID_DEPTH + Units.inchesToMeters(37) / 2.0;

        Translation2d ONE =   new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(196));
        Translation2d TWO =   new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(174.125));
        Translation2d THREE = new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(152));
        Translation2d FOUR =  new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(130));
        Translation2d FIVE =  new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(108.125));
        Translation2d SIX =   new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(86));
        Translation2d SEVEN = new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(64));
        Translation2d EIGHT = new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(42.125));
        Translation2d NINE =  new Translation2d(DEFAULT_X_DIST, Units.inchesToMeters(20));
    }

    Translation2d BLUE_ALIGN_POSES[] = {
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

    Translation2d RED_ALIGN_POSES[] = {
        AllianceUtil.getMirroredTranslation(ScorePoses.NINE),
        AllianceUtil.getMirroredTranslation(ScorePoses.EIGHT),
        AllianceUtil.getMirroredTranslation(ScorePoses.SEVEN),
        AllianceUtil.getMirroredTranslation(ScorePoses.SIX),
        AllianceUtil.getMirroredTranslation(ScorePoses.FIVE),
        AllianceUtil.getMirroredTranslation(ScorePoses.FOUR),
        AllianceUtil.getMirroredTranslation(ScorePoses.THREE),
        AllianceUtil.getMirroredTranslation(ScorePoses.TWO),
        AllianceUtil.getMirroredTranslation(ScorePoses.ONE)
    };

}

