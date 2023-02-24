package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface ArmTrajectories {

    public static enum ShoulderState {
        FRONT,
        INSIDE,
        BACK;

        public static ShoulderState getState(Rotation2d shoulderAngle) {
            double normalizedDeg = shoulderAngle.minus(Rotation2d.fromDegrees(-90)).getDegrees();
            if (Math.abs(normalizedDeg) < Settings.Arm.Shoulder.INTAKE_OVER_BUMPER_ANGLE.get())
                return INSIDE;

            return (normalizedDeg > 0) ? FRONT : BACK;
        }

        public static ShoulderState getFrontBackState(Rotation2d shoulderAngle) {
            double normalizedDeg = shoulderAngle.minus(Rotation2d.fromDegrees(-90)).getDegrees();

            return (normalizedDeg > 0) ? FRONT : BACK;
        }

        public boolean isOppositeFrom(ShoulderState other) {
            if (this == FRONT && other == BACK) return true;
            if (this == BACK && other == FRONT) return true;
            return false;
        }
    }

    public static ArmTrajectory generateTrajectory(ArmState src, ArmState dest) {
        ArmTrajectory out = new ArmTrajectory();

        ShoulderState srcState = ShoulderState.getState(src.getShoulderState());
        ShoulderState destState = ShoulderState.getState(dest.getShoulderState());

        SmartDashboard.putString("Arm/Shoulder/Source State", srcState.name());
        SmartDashboard.putString("Arm/Shoulder/Destination State", destState.name());

        // no safe points necessary
        if (srcState == destState) {
            return out.addState(dest);
        }

        // if trying to cross, add both safety poitns
        if (srcState.isOppositeFrom(destState)) {
            if (srcState == ShoulderState.FRONT) {
                out.addState(kSafePointFront);
                out.addState(kSafePointBack);
            } else {
                out.addState(kSafePointBack);
                out.addState(kSafePointFront);
            }
        }

        // if starting within bumper or ending within bumper, add one safety point
        else {
            if (srcState == ShoulderState.FRONT || destState == ShoulderState.FRONT) {
                out.addState(kSafePointFront);
            } else {
                out.addState(kSafePointBack);
            }
        }

        return out.addState(dest);
    }

    ArmState kSafePointFront = new ArmState(
        new SmartNumber("Arm Trajectories/Safe Point Front Shoulder", -70),
        new SmartNumber("Arm Trajectories/Safe Point Front Wrist", +90));
    
    ArmState kSafePointBack = new ArmState(
        new SmartNumber("Arm Trajectories/Safe Point Back Shoulder", -110),
        new SmartNumber("Arm Trajectories/Safe Point Back Wrist", +90));

    /* Intaking */

    public interface Acquire {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Acquire Front Shoulder", -85),
			new SmartNumber("Arm Trajectories/Acquire Front Wrist", 0));
    }

    public interface Deacquire {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Deacquire Front Shoulder", -80),
			new SmartNumber("Arm Trajectories/Deacquire Front Wrist", 10));
    }

    public interface Neutral {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Stowed Front Shoulder", -85),
			new SmartNumber("Arm Trajectories/Stowed Front Wrist", 180 - 15));
    }


    /* Ready */

    public interface Ready {
        public interface Mid {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Shoulder", -180 - 0),
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Wrist", -180 - -60));
            
            ArmState kConeTipOutFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out Front Shoulder", -25),
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out Front Wrist", 60));
            
            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Cube Front Shoulder", -30),
                new SmartNumber("Arm Trajectories/Ready Mid Cube Front Wrist", 45));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Cube Back Shoulder", -180 - -30),
                new SmartNumber("Arm Trajectories/Ready Mid Cube Back Wrist", -180 - 45));
        }

        public interface High {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Shoulder", -180 - 0),
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Wrist", -180 - -15));
            
            ArmState kConeTipUpBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Shoulder", -180 - 0),
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Wrist", -180 - 55));

            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Cube Front Shoulder", -180 - (-175)),
                new SmartNumber("Arm Trajectories/Ready High Cube Front Wrist", 180 - 130));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Cube Back Shoulder", -180 - -180 - (-175)),
                new SmartNumber("Arm Trajectories/Ready High Cube Back Wrist", -180 - 180 - 130));
        }
    }

    /* Score */

    public interface Score {
        public interface Mid {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Score Mid Tip In Back Shoulder", -180 - -5),
                new SmartNumber("Arm Trajectories/Score Mid Tip In Back Wrist", -180 - -90));
            
            ArmState kConeTipOutFront = new ArmState(
                new SmartNumber("Arm Trajectories/Score Mid Tip Out Front Shoulder", -35),
                new SmartNumber("Arm Trajectories/Score Mid Tip Out Front Wrist", 40));
            
            ArmState kCubeFront = Ready.Mid.kCubeFront;
            ArmState kCubeBack = Ready.Mid.kCubeBack;
        }

        public interface High {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Score High Tip In Back Shoulder", -180 - -7),
                new SmartNumber("Arm Trajectories/Score High Tip In Back Wrist", -180 - 20));
            
            ArmState kConeTipUpBack = Ready.High.kConeTipUpBack;

            ArmState kCubeFront = Ready.High.kCubeFront;
            ArmState kCubeBack = Ready.High.kCubeBack;
        }
    }

}
