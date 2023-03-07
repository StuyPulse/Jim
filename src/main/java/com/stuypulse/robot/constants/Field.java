package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.AllianceUtil;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Field {
    SmartNumber CHARGING_STATION_CENTER = new SmartNumber("Field/Charging Station Center", 172.631 - 14);
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

    public static boolean isValidAprilTagId(int id) {
        return id >= 1 && id <= 8;
    }

    public static boolean isAprilTagBlueFromId(int id) {
        return id >= 5 && id <= 8;
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
        double CUBE_LOW_X = 694;
        double CUBE_MID_X = 694;
        double CUBE_HIGH_X = 2.1;

        double CONE_LOW_X = 694;
        double CONE_MID_X = 694;
        double CONE_HIGH_X = 2.052;



        Translation2d ONE =   new Translation2d(CUBE_HIGH_X, (3.03));
        Translation2d TWO =   new Translation2d(CUBE_HIGH_X, (4.8087));
        Translation2d THREE = new Translation2d(CUBE_HIGH_X, (3.724));
        Translation2d FOUR =  new Translation2d(CUBE_HIGH_X, (4.258));
        Translation2d FIVE =  new Translation2d(CUBE_HIGH_X, (5.044971));
        Translation2d SIX =   new Translation2d(CUBE_HIGH_X, (5.308));
        Translation2d SEVEN = new Translation2d(CUBE_HIGH_X, (5.8913));
        Translation2d EIGHT = new Translation2d(CUBE_HIGH_X, (6.674766));
        Translation2d NINE =  new Translation2d(CUBE_HIGH_X, (7)); // definitely wrong
    }

    Translation2d RED_ALIGN_POSES[] = {
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

    Translation2d BLUE_ALIGN_POSES[] = {
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

