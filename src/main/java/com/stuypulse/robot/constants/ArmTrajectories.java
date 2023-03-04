package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmState;
import com.stuypulse.stuylib.network.SmartNumber;

public interface ArmTrajectories {

    /* Intaking */

    public interface Acquire {
        ArmState kCone = new ArmState(
			new SmartNumber("Arm Trajectories/Acquire Cone Front Shoulder", -85),
			new SmartNumber("Arm Trajectories/Acquire Cone Front Wrist", -15));
        ArmState kCube = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Cube Front Shoulder", -71),
            new SmartNumber("Arm Trajectories/Acquire Cube Front Wrist", -5.0));
        ArmState kIntermediate = new ArmState(
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Shoulder", -45),
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Wrist", 0));
    }

    public interface Deacquire {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Deacquire Front Shoulder", -65),
			new SmartNumber("Arm Trajectories/Deacquire Front Wrist", 45));
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
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Shoulder", -165),
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Wrist", 179));
            
            ArmState kConeTipOutFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out Front Shoulder", -25),
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out Front Wrist", 60));
            
            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Cube Front Shoulder", -30),
                new SmartNumber("Arm Trajectories/Ready Mid Cube Front Wrist", 45));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready Mid Cube Back Shoulder", -175),
                new SmartNumber("Arm Trajectories/Ready Mid Cube Back Wrist", -65));
        }

        public interface High {
            ArmState kConeTipInBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Shoulder", 177),
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Wrist", 175));
            
            ArmState kConeTipUpBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Shoulder", 180),
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Wrist", 120));

            ArmState kCubeFront = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Cube Front Shoulder", -180 - (-175)),
                new SmartNumber("Arm Trajectories/Ready High Cube Front Wrist", 180 - 130));
            ArmState kCubeBack = new ArmState(
                new SmartNumber("Arm Trajectories/Ready High Cube Back Shoulder", 180),
                new SmartNumber("Arm Trajectories/Ready High Cube Back Wrist", -160));
        }
    }

}
