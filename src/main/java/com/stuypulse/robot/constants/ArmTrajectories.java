/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import com.stuypulse.robot.util.ArmState;

public interface ArmTrajectories {

    /* Intaking */

    public interface Acquire {
        ArmState kCone = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Acquire Cone Shoulder", -75),
            new SmartNumber("Arm Trajectories/Acquire Cone Wrist", 7));
        ArmState kCube = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Acquire Cube Shoulder", -75),
            new SmartNumber("Arm Trajectories/Acquire Cube Wrist", 12));

        ArmState kHPCone = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Acquire HP Cone Shoulder", 0),    
            new SmartNumber("Arm Trajectories/Acquire HP Cone Wrist", 0));

        ArmState kIntermediate = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Shoulder", -55),
            new SmartNumber("Arm Trajectories/Acquire Intermediate Front Wrist", 0));

        ArmState kCubeAuton = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Auton Acquire Cube Front Shoulder", -70.82),
            new SmartNumber("Arm Trajectories/Auton Acquire Cube Front Wrist", 8.37));

        ArmState kBOOMCubeAuton = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Auton BOOM Acquire Cube Front Shoulder", -70.82),
            new SmartNumber("Arm Trajectories/Auton BOOM Acquire Cube Front Wrist", 8.37));
        ArmState kIntermediateAuton = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Auton Acquire Intermediate Front Shoulder", -45),
            new SmartNumber("Arm Trajectories/Auton Acquire Intermediate Front Wrist", 0));
    }

    public interface Deacquire {
        ArmState kFrontTrajectory = ArmState.fromWristHorizontal(
			    new SmartNumber("Arm Trajectories/Deacquire Front Shoulder", -77),
			    new SmartNumber("Arm Trajectories/Deacquire Front Wrist", 90));

        ArmState kBackTrajectory = ArmState.fromWristHorizontal(
            new SmartNumber("Arm Trajectories/Deacquire Back Shoulder", -134),
            new SmartNumber("Arm Trajectories/Deacquire Back Wrist", -22.9));
    }

    public interface Stow {
        ArmState kTrajectory = new ArmState(
			new SmartNumber("Arm Trajectories/Stowed Front Shoulder", -85),
			new SmartNumber("Arm Trajectories/Stowed Front Wrist", 180 /*165*/));
    }


    /* Ready */

    public interface Ready {
        public interface Mid {
            ArmState kConeTipUpBack = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Ready Mid Tip Up Back Shoulder", -172),
                new SmartNumber("Arm Trajectories/Ready Mid Tip Up Back Wrist", 136));

            ArmState kConeTipInBack = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Shoulder", -171),
                new SmartNumber("Arm Trajectories/Ready Mid Tip In Back Wrist", -175));

                // -4, 0
            ArmState kConeTipOutFront = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out/Shoulder", -14),
                new SmartNumber("Arm Trajectories/Ready Mid Tip Out/Wrist", 42));

            ArmState kAutonCubeBack = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Ready Auton Mid Cube Back/Shoulder", -165),
                new SmartNumber("Arm Trajectories/Ready Auton Mid Cube Back/Wrist", -133));

            ArmState kCubeFront = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Mid Cube Front/Shoulder", -21),
                new SmartNumber("Arm Trajectories/Mid Cube Front/Wrist", 49));

            ArmState kCubeBack = ArmState.fromWristHorizontal( 
                new SmartNumber("Arm Trajectories/Mid Cube Back/Shoulder", -175),
                new SmartNumber("Arm Trajectories/Mid Cube Back/Wrist", -62));
        }

        public interface High {
            ArmState kConeTipInBack = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Shoulder", -185),
                new SmartNumber("Arm Trajectories/Ready High Tip In Back Wrist", -180));

            // -175, 128
            ArmState kConeTipUpBack = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Shoulder", -179),
                new SmartNumber("Arm Trajectories/Ready High Tip Up Back Wrist", 136));

            ArmState kConeTipOutFront = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Ready High Tip Out Front Shoulder", 3),
                new SmartNumber("Arm Trajectories/Ready High Tip Out Front Wrist", 37));

            ArmState kCubeFront = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/High Cube Front/Shoulder", -5),
                new SmartNumber("Arm Trajectories/High Cube Front/Wrist", 46));

            ArmState kCubeAutonBack = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/High Auton Cube Back/Shoulder", -186 - 5),
                new SmartNumber("Arm Trajectories/High Auton Cube Back/Wrist", -138 + 5));

            ArmState kCubeBack = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/High Cube Back/Shoulder", -186),
                new SmartNumber("Arm Trajectories/High Cube Back/Wrist", -138));
        }
    }

    public interface Score {
        public interface High {
            ArmState kConeTipOutFront = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Score High Tip Out Front Shoulder", -5),
                new SmartNumber("Arm Trajectories/Score High Tip Out Front Wrist", 37));
        }

        public interface Mid {
            ArmState kConeTipOutFront = ArmState.fromWristHorizontal(
                new SmartNumber("Arm Trajectories/Score Mid Tip Out Front Shoulder", -28 - 5),
                new SmartNumber("Arm Trajectories/Score Mid Tip Out Front Wrist", 44));
        }
    }

}
