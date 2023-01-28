package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmTrajectory;

public interface ArmTrajectories {
    ArmTrajectory NEUTRAL = new ArmTrajectory().addState(-90, 90);

    public interface SameSide {
        public interface High {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
        }
        public interface Mid {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
        }
        public interface Low {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
        }
    }

    public interface OppositeSide {
        public interface High {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
        }
        public interface Mid {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
        }
        public interface Low {
            public interface Cone {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
            public interface Cube {
                ArmTrajectory READY = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE = new ArmTrajectory().addState(0, 0);
                ArmTrajectory INTAKE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
                ArmTrajectory SCORE_TO_NEUTRAL = new ArmTrajectory().addState(0, 0);
            }
        }
    }
}
