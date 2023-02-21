package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmBFSField;
import com.stuypulse.stuylib.util.StopWatch;

public final class ArmFields {

    static {
        StopWatch timer = new StopWatch();

        Neutral.kTrajectory.getSize();
        Intake.kTrajectory.getSize();

        Ready.Low.kConeTipInOpposite.getSize();

        Ready.Mid.kConeTipInSame.getSize();
        Ready.Mid.kConeTipInOpposite.getSize();
        Ready.Mid.kConeTipOutSame.getSize();
        Ready.Mid.kCube.getSize();

        Ready.High.kConeTipInSame.getSize();
        Ready.High.kConeTipInOpposite.getSize();
        Ready.High.kCube.getSize();

        Score.Mid.kConeTipInSame.getSize();
        Score.Mid.kConeTipInOpposite.getSize();
        Score.Mid.kConeTipOutSame.getSize();

        Score.High.kConeTipInSame.getSize();
        Score.High.kConeTipInOpposite.getSize();

        System.out.println("ArmBFSFields Generated in " + timer.reset() + " seconds.");
    }

    public static void load() {
        System.out.println("Hello World!");
    }

    public interface Intake {
        public static final ArmBFSField kTrajectory = new ArmBFSField(-80, 0 /*-8*/, Constraints.CONSTRAINT);
        // public static final ArmBFSField kBackTrajectory = new ArmBFSField(-103, 170, Constraints.CONSTRAINT);
    }

    public interface Outtake {
        public static final ArmBFSField kTrajectory = new ArmBFSField(-80, 10, Constraints.CONSTRAINT);
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
            ArmBFSField kConeTipInSame = new ArmBFSField(-5, -20, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(0, -60, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = new ArmBFSField(-25, 60, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = new ArmBFSField(-30, 45, Constraints.CONSTRAINT);
        }

        public interface High {
            ArmBFSField kConeTipInSame = new ArmBFSField(11, -22, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(0, -30, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = new ArmBFSField(-30, 10, Constraints.CONSTRAINT);;
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
            ArmBFSField kConeTipInSame = new ArmBFSField(-8, -28, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(-5, -90, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = new ArmBFSField(-35, 40, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = Ready.Mid.kCube;
        }

        public interface High {
            ArmBFSField kConeTipInSame = new ArmBFSField(11, -32, Constraints.CONSTRAINT);
            ArmBFSField kConeTipInOpposite = new ArmBFSField(0, -70, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = Ready.High.kCube;
        }
    }

}
