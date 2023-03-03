package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmState;
import com.stuypulse.stuylib.network.SmartNumber;

public interface ArmTrajectories {

    /* Intaking */

    public interface Acquire {
        ArmState kCone = new ArmState(
			new SmartNumber("Arm Trajectories/Acquire Cone/Shoulder", -85),
			new SmartNumber("Arm Trajectories/Acquire Cone/Wrist", -15));
        ArmState kCube = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Cube/Shoulder", -71),
            new SmartNumber("Arm Trajectories/Acquire Cube/Wrist", -5.0));
        ArmState kIntermediate = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Intermediate/Shoulder", -45),
            new SmartNumber("Arm Trajectories/Acquire Intermediate/Wrist", 0));
    }

    public interface Deacquire {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Deacquire/Shoulder", -65),
			new SmartNumber("Arm Trajectories/Deacquire/Wrist", 45));
    }

    public interface Stow {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Stowed/Shoulder", -75),
			new SmartNumber("Arm Trajectories/Stowed/Wrist", 165));
    }


    /* Ready */

    public interface Ready {
        public interface Mid {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Mid Tip In/Shoulder", -165),
                new SmartNumber("Arm Trajectories/Mid Tip In/Wrist", 179));
            
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
                new SmartNumber("Arm Trajectories/High Tip In/Shoulder", 177),
                new SmartNumber("Arm Trajectories/High Tip In/Wrist", 175));
            
            ArmState kConeTipUpBack = new ArmState(
                new SmartNumber("Arm Trajectories/High Tip Up/Shoulder", 180),
                new SmartNumber("Arm Trajectories/High Tip Up/Wrist", 120));

            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/High Cube Front/Shoulder", -180 - (-175)),
                new SmartNumber("Arm Trajectories/High Cube Front/Wrist", 180 - 130));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/High Cube Back/Shoulder", 180),
                new SmartNumber("Arm Trajectories/High Cube Back/Wrist", -160));
        }
    }

}
