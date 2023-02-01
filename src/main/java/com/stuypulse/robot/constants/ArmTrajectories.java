package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmTrajectory;

public interface ArmTrajectories {
    ArmTrajectory NEUTRAL = new ArmTrajectory().addState(0, 0);

    public interface SameSide {
        public interface High {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(-15, 55);
                ArmTrajectory SCORE = new ArmTrajectory().addState(-15, 25);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(READY).addState(NEUTRAL);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(READY).addState(NEUTRAL);
            }
        }
        public interface Mid {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 45);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(READY).addState(NEUTRAL);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(READY).addState(NEUTRAL);
            }
        }
        public interface Low {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(READY).addState(NEUTRAL);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(READY).addState(NEUTRAL);
            }
        }
    }

    // public interface OppositeSide {
    //     public interface High {
    //         public interface Cone {
    //             ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //         }
    //         public interface Cube {
    //             ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //         }
    //     }
    //     public interface Mid {
    //         public interface Cone {
    //             ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //         }
    //         public interface Cube {
    //             ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //         }
    //     }
    //     public interface Low {
    //         public interface Cone {
    //             ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //         }
    //         public interface Cube {
    //             ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //             ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
    //         }
    //     }
    // }
}
