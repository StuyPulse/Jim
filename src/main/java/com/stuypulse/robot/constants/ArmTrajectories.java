package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmState;
import com.stuypulse.stuylib.network.SmartNumber;

public interface ArmTrajectories {

    /* Intaking */

    public interface Acquire {
        ArmState kCone = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Cone Shoulder", -75),
            new SmartNumber("Arm Trajectories/Acquire Cone Wrist", 12));
        ArmState kCube = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Cube Shoulder", -75),
            new SmartNumber("Arm Trajectories/Acquire Cube Wrist", 12));

        ArmState kHPCone = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire HP Cone Shoulder", 9),
            new SmartNumber("Arm Trajectories/Acquire HP Cone Wrist", 10));

        ArmState kIntermediate = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Shoulder", -55),
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Wrist", 0));

        ArmState kCubeAuton = new ArmState(
            new SmartNumber("Arm Trajectories/Auton Acquire Cube Front Shoulder", -70.82),
            new SmartNumber("Arm Trajectories/Auton Acquire Cube Front Wrist", 8.37));

        ArmState kBOOMCubeAuton = new ArmState(
            new SmartNumber("Arm Trajectories/Auton BOOM Acquire Cube Front Shoulder", -70.82),
            new SmartNumber("Arm Trajectories/Auton BOOM Acquire Cube Front Wrist", 8.37));
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
			new SmartNumber("Arm Trajectories/Stowed Front Shoulder", -85),
			new SmartNumber("Arm Trajectories/Stowed Front Wrist", 165));
    }


    /* Ready */

    public interface Ready {
        public interface Mid {
            ArmState kConeTipUpBack = new ArmState(
                new SmartNumber("Arm Trajectories/Alignment High Tip Up Back Shoulder", -135.5),
                new SmartNumber("Arm Trajectories/Alignment High Tip Up Back Wrist",175.5));

            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Shoulder", -175),
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Wrist", -158));
            
            ArmState kConeTipOutFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out/Shoulder", -14),
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out/Wrist", 42));

            ArmState kAutonCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Auton Mid Cube Back/Shoulder", -133.1),
                new SmartNumber("Arm Trajectories/Ready Auton Mid Cube Back/Wrist", -133.5));
            
            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/Mid Cube Front/Shoulder", -26),
                new SmartNumber("Arm Trajectories/Mid Cube Front/Wrist", 49));

            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/Mid Cube Back/Shoulder", -175),
                new SmartNumber("Arm Trajectories/Mid Cube Back/Wrist", -62));
        }

        public interface High {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Shoulder", -190),
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Wrist", -170));
            
            ArmState kConeTipUpBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Shoulder", -170),
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Wrist", 135.5));

            ArmState kConeTipOutFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip Out Front Shoulder", 3),
                new SmartNumber("Arm Trajectories/Ready High Tip Out Front Wrist", 50));

            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/High Cube Front/Shoulder", -10),
                new SmartNumber("Arm Trajectories/High Cube Front/Wrist", 46));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/High Cube Back/Shoulder", -186),
                new SmartNumber("Arm Trajectories/High Cube Back/Wrist", -138));
        }

        public interface Score {
            public interface High {
                ArmState kConeTipOutFront = new ArmState(
                    new SmartNumber("Arm Trajectories/Score High Tip Out Front Shoulder", -11),
                    new SmartNumber("Arm Trajectories/Score High Tip Out Front Wrist", 44));    
            }

            public interface Mid {
                ArmState kConeTipOutFront = new ArmState(
                    new SmartNumber("Arm Trajectories/Score High Tip Out Front Shoulder", -28),
                    new SmartNumber("Arm Trajectories/Score High Tip Out Front Wrist", 44));    
            }
        }
    }

}