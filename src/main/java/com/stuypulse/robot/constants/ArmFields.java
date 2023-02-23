package com.stuypulse.robot.constants;

import java.io.File;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.fields.*;
import com.stuypulse.robot.util.ArmBFSField;
import com.stuypulse.robot.util.FieldFileUtil;
import com.stuypulse.stuylib.util.StopWatch;

public final class ArmFields {

    public static void load() {
        if (Robot.isSimulation()) {
            File[] files = FieldFileUtil.getFieldDirectory().listFiles();

            if (files != null) {
                for (File f : files) {
                    f.delete();
                }
            }
    
            System.out.println("Deleted old bfs fields");
        }

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
        System.out.println("Hello World!");
    }

    public interface Acquire {
        ArmBFSField kTrajectory = ArmBFSField.create(-80, 0 /*-8*/, Constraints.CONSTRAINT, AcquireBFS.array, "Acquire");
    }

    public interface Deacquire {
        ArmBFSField kTrajectory = ArmBFSField.create(-80, 10, Constraints.CONSTRAINT, DeacquireBFS.array, "Deacquire");
    }

    public interface Neutral {
        ArmBFSField kTrajectory = ArmBFSField.create(-85, 180 - 15, Constraints.CONSTRAINT, NeutralBFS.array, "Neutral");
    }

    /* Intaking */

    /* Ready */

    public interface Ready {
        public interface Low {
            ArmBFSField kConeTipInSame = Acquire.kTrajectory;
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(-40, -150, Constraints.CONSTRAINT, ReadyLowTipInOppositeBFS.array, "ReadyLowTipInOpposite");
            
            ArmBFSField kConeTipOutSame = Acquire.kTrajectory;
            ArmBFSField kConeTipOutOpposite = kConeTipInOpposite;

            ArmBFSField kCube = Acquire.kTrajectory;
        }

        public interface Mid {
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(0, -60, Constraints.CONSTRAINT, ReadyMidTipInOppositeBFS.array, "ReadyMidTipInOpposite");
            
            ArmBFSField kConeTipOutSame = ArmBFSField.create(-25, 60, Constraints.CONSTRAINT, ReadyMidCubeBFS.array, "ReadyMidCube");
            
            ArmBFSField kCube = ArmBFSField.create(-30, 45, Constraints.CONSTRAINT, ReadyMidCubeBFS.array, "ReadyMidCube");
        }

        public interface High {
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(15, -15, Constraints.CONSTRAINT, ReadyHighTipInOppositeBFS.array, "ReadyHighTipInOpposite");
            
            ArmBFSField kCube = ArmBFSField.create(-180 - (-175), 180 - 130, Constraints.CONSTRAINT, ReadyHighCubeBFS.array, "ReadyHighCube");
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
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(-5, -90, Constraints.CONSTRAINT, ScoreMidTipInSameBFS.array, "ScoreMidTipInSame");
            
            ArmBFSField kConeTipOutSame = ArmBFSField.create(-35, 40, Constraints.CONSTRAINT, ScoreMidTipOutSameBFS.array, "ScoreMidTipOutSame");
            
            ArmBFSField kCube = Ready.Mid.kCube;
        }

        public interface High {
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(-7, 20, Constraints.CONSTRAINT, ScoreHighTipInOppositeBFS.array, "ScoreHighTipInOpposite");
            
            ArmBFSField kCube = Ready.High.kCube;
        }
    }

}
