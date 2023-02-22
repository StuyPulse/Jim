package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ArmBFSField;
import com.stuypulse.stuylib.util.StopWatch;

public final class ArmFields {

    static {
        StopWatch timer = new StopWatch();

        Neutral.kTrajectory.getSize();
        Acquire.kTrajectory.getSize();
        Deacquire.kTrajectory.getSize();

        Ready.Low.kConeTipInOpposite.getSize();

        Ready.Mid.kConeTipInOpposite.getSize();
        Ready.Mid.kConeTipOutSame.getSize();
        Ready.Mid.kCube.getSize();

        Ready.High.kConeTipInOpposite.getSize();
        Ready.High.kCube.getSize();

        Score.Mid.kConeTipInOpposite.getSize();
        Score.Mid.kConeTipOutSame.getSize();

        Score.High.kConeTipInOpposite.getSize();

        System.out.println("ArmBFSFields Generated in " + timer.reset() + " seconds.");
    }

    public static void load() {
        System.out.println("Hello World!");
    }

    public interface Acquire {
        ArmBFSField kTrajectory = new ArmBFSField(-80, 0 /*-8*/, Constraints.CONSTRAINT);
    }

    public interface Deacquire {
        ArmBFSField kTrajectory = new ArmBFSField(-80, 10, Constraints.CONSTRAINT);
    }

    public interface Neutral {
        ArmBFSField kTrajectory = new ArmBFSField(-85, 180 - 15, Constraints.CONSTRAINT);
    }

    /* Intaking */

    /* Ready */

    public interface Ready {
        public interface Low {
            ArmBFSField kConeTipInSame = Acquire.kTrajectory;
            ArmBFSField kConeTipInOpposite = new ArmBFSField(-40, -150, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = Acquire.kTrajectory;
            ArmBFSField kConeTipOutOpposite = kConeTipInOpposite;

            ArmBFSField kCube = Acquire.kTrajectory;
        }

        public interface Mid {
            ArmBFSField kConeTipInOpposite = new ArmBFSField(0, -60, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = new ArmBFSField(-25, 60, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = new ArmBFSField(-30, 45, Constraints.CONSTRAINT);
        }

        public interface High {
            ArmBFSField kConeTipInOpposite = new ArmBFSField(-15, 15, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = new ArmBFSField(-180 - (-175), 180 - 130, Constraints.CONSTRAINT);
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
            ArmBFSField kConeTipInOpposite = new ArmBFSField(-5, -90, Constraints.CONSTRAINT);
            
            ArmBFSField kConeTipOutSame = new ArmBFSField(-35, 40, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = Ready.Mid.kCube;
        }

        public interface High {
            ArmBFSField kConeTipInOpposite = new ArmBFSField(-7, 20, Constraints.CONSTRAINT);
            
            ArmBFSField kCube = Ready.High.kCube;
        }
    }

}
