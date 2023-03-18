package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmState;
import com.stuypulse.stuylib.network.SmartNumber;

public interface ArmTrajectories {

    /* Intaking */

    public interface Acquire {
        ArmState kCone = new ArmState(
			      new SmartNumber("Arm Trajectories/Acquire Cone Tip In Front Shoulder", -70.82),
			      new SmartNumber("Arm Trajectories/Acquire Cone Tip In Front Wrist", 7.3));
        ArmState kCube = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Cube Front Shoulder", -70.82),
            new SmartNumber("Arm Trajectories/Acquire Cube Front Wrist", 8.37));
        ArmState kIntermediate = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Shoulder", -45),
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Wrist", 0));

        ArmState kCubeAuton = new ArmState(
            new SmartNumber("Arm Trajectories/Auton Acquire Cube Front Shoulder", -70.82),
            new SmartNumber("Arm Trajectories/Auton Acquire Cube Front Wrist", 8.37));
        ArmState kIntermediateAuton = new ArmState(
            new SmartNumber("Arm Trajectories/Auton Acquire Intermediate Front Shoulder", -45),
            new SmartNumber("Arm Trajectories/Auton Acquire Intermediate Front Wrist", 0));
    }

    public interface Deacquire {
        ArmState kFrontTrajectory = new ArmState(
			    new SmartNumber("Arm Trajectories/Deacquire Front Shoulder", -77),
			    new SmartNumber("Arm Trajectories/Deacquire Front Wrist", 90));

        ArmState kBackTrajectory = new ArmState(
            new SmartNumber("Arm Trajectories/Deacquire Back Shoulder", -134),
            new SmartNumber("Arm Trajectories/Deacquire Back Wrist", -22.9));
    }

    public interface Stow {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Stowed Front Shoulder", -77),
			new SmartNumber("Arm Trajectories/Stowed Front Wrist", 165));
    }


    /* Ready */

    public interface Ready {
        public interface Mid {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Shoulder", -160),
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Wrist", -175));
            
            ArmState kConeTipOutFront = new ArmState(
                new SmartNumber("Arm Trajectories/Mid Tip Out/Shoulder", -25),
                new SmartNumber("Arm Trajectories/Mid Tip Out/Wrist", 60));
            
            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/Mid Cube Front/Shoulder", -30),
                new SmartNumber("Arm Trajectories/Mid Cube Front/Wrist", 45));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/Mid Cube Back/Shoulder", -175),
                new SmartNumber("Arm Trajectories/Mid Cube Back/Wrist", -65));
        }

        public interface High {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Shoulder", 179),
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Wrist", -178));
            
            ArmState kConeTipUpBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Shoulder", -171.66),
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Wrist", 121.12));

            ArmState kConeTipOutFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip Out Front Shoulder", -4),
                new SmartNumber("Arm Trajectories/Ready High Tip Out Front Wrist", 43));

            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/High Cube Front/Shoulder", -180 - (-175)),
                new SmartNumber("Arm Trajectories/High Cube Front/Wrist", 180 - 130));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/High Cube Back/Shoulder", -175),
                new SmartNumber("Arm Trajectories/High Cube Back/Wrist", -153));
        }

        public interface Score {
            public interface High {
                ArmState kConeTipOutFront = new ArmState(
                    new SmartNumber("Arm Trajectories/Score High Tip Out Front Shoulder", -11),
                    new SmartNumber("Arm Trajectories/Score High Tip Out Front Wrist", 43));    
            }
        }
    }

}
