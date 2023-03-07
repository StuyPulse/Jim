package com.stuypulse.robot.subsystems.vision;

import java.util.List;

import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Vision extends SubsystemBase {

    public SmartBoolean APRIL_TAG_RESET = new SmartBoolean("Odometry/April Tag Reset", false);

    /** SINGLETON **/
    private static final Vision instance;

    static { 
        instance = new VisionImpl();
    }

    public static Vision getInstance() {
        return instance; 
    }

    protected Vision() {
    }

    /** VISION TYPES **/

    // TODO: remove noise type
    public enum Noise {
        LOW,
        MID,
        HIGH;
    }

    public static class Result {
        public final AprilTagData data;
        public Noise noise;

        public Result(AprilTagData data, Noise error) {
            this.data = data;
            this.noise = error;
        }

        public Pose2d getPose() {
            return data.pose;
        }

        public double getLatency() {
            return data.latency;
        }

        public Noise getNoise() {
            return noise;
        }

        public AprilTagData getData(){
            return data;
        }
    }

    /** ABSTRACT METHODS **/
    public abstract List<Result> getResults();
}
