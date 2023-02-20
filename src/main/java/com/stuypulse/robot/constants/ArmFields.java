package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmBFSField;
import com.stuypulse.stuylib.util.StopWatch;

public final class ArmFields {

    static {
        StopWatch timer = new StopWatch();

        Neutral.kTrajectory.getSetpoint();
        Intake.kTrajectory.getSetpoint();

        Ready.Low.kConeTipInOpposite.getSetpoint();

        Ready.Mid.kConeTipInSame.getSetpoint();
        Ready.Mid.kConeTipInOpposite.getSetpoint();
        Ready.Mid.kConeTipOutSame.getSetpoint();
        Ready.Mid.kCube.getSetpoint();

        Ready.High.kConeTipInSame.getSetpoint();
        Ready.High.kConeTipInOpposite.getSetpoint();
        Ready.High.kCube.getSetpoint();

        Score.Mid.kConeTipInSame.getSetpoint();
        Score.Mid.kConeTipInOpposite.getSetpoint();
        Score.Mid.kConeTipOutSame.getSetpoint();

        Score.High.kConeTipInSame.getSetpoint();
        Score.High.kConeTipInOpposite.getSetpoint();

        System.out.println("ArmBFSFields Generated in " + timer.reset() + " seconds.");
    }

    public static void load() {
        System.out.println("Hello World!");
    }

    public interface Intake {
        public static final ArmBFSField kTrajectory = new ArmBFSField(-55, 0, Constraints.CONSTRAINT);
    }

    public interface Neutral {
        public static final ArmBFSField kTrajectory = new ArmBFSField(-90, +90, Constraints.CONSTRAINT);
    }

    /* Intaking */

    /* Ready */

    public interface Ready {
        public interface Low {
            ArmBFSField kConeTipInSame = Intake.kTrajectory;
            ArmBFSField kConeTipInOpposite = new ArmBFSField(-40, -150, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = Intake.kTrajectory;
            ArmBFSField kConeTipOutOpposite = kConeTipInOpposite;

            ArmBFSField kCube = Intake.kTrajectory;
        }

        public interface Mid {
            ArmBFSField kConeTipInSame = new ArmBFSField(0, -75, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(0, -60, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = new ArmBFSField(-20, 85, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = new ArmBFSField(-45, 60, Constraints.CONSTRAINT);
        }

        public interface High {
            ArmBFSField kConeTipInSame = new ArmBFSField(0, -30, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(0, -30, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = new ArmBFSField(-20, 70, Constraints.CONSTRAINT);;
        }
    }

    /* Score */

    public interface Score {
        public interface Low {
            ArmBFSField kConeTipInSame = Ready.Low.kConeTipInSame;
            ArmBFSField kConeTipInOpposite = Ready.Low.kConeTipInOpposite;
            
            ArmBFSField kConeTipOutSame = Ready.Low.kConeTipOutSame;
            ArmBFSField kConeTipOutOpposite = Ready.Low.kConeTipOutOpposite;

            ArmBFSField kCube = Ready.Low.kCube;
        }

        public interface Mid {
            ArmBFSField kConeTipInSame = new ArmBFSField(-5, -85, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(-5, -90, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = new ArmBFSField(-35, 90, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = Ready.Mid.kCube;
        }

        public interface High {
            ArmBFSField kConeTipInSame = new ArmBFSField(0, -70, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(0, -70, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = Ready.High.kCube;
        }
    }

}
