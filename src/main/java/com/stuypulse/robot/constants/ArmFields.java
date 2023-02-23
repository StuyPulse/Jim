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
        System.out.println("Hello World!");
    }

    public interface Intake {
        ArmBFSField kTrajectory = ArmBFSField.create(-70, 5, Constraints.CONSTRAINT, IntakeBFS.array, "Intake");
        // ArmBFSField kBackTrajectory = ArmBFSField.create(-103, 170, Constraints.CONSTRAINT);
    }

    public interface Neutral {
        ArmBFSField kTrajectory = ArmBFSField.create(-90, +90, Constraints.CONSTRAINT, NeutralBFS.array, "Neutral");
    }

    /* Intaking */

    /* Ready */

    public interface Ready {
        public interface Low {
            ArmBFSField kConeTipInSame = Intake.kTrajectory;
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(-40, -150, Constraints.CONSTRAINT, ReadyLowTipInOppositeBFS.array, "ReadyLowTipInOpposite");
            
            ArmBFSField kConeTipOutSame = Intake.kTrajectory;
            ArmBFSField kConeTipOutOpposite = kConeTipInOpposite;

            ArmBFSField kCube = Intake.kTrajectory;
        }

        public interface Mid {
            ArmBFSField kConeTipInSame = ArmBFSField.create(-5, -20, Constraints.CONSTRAINT, ReadyMidTipInSameBFS.array, "ReadyMidTipInSame");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(0, -60, Constraints.CONSTRAINT, ReadyMidTipInOppositeBFS.array, "ReadyMidTipInOpposite");
            
            ArmBFSField kConeTipOutSame = ArmBFSField.create(-20, 85, Constraints.CONSTRAINT, ReadyMidTopOutSameBFS.array, "ReadyMidTopOutSame");
            
            ArmBFSField kCube = ArmBFSField.create(-45, 60, Constraints.CONSTRAINT, ReadyMidCubeBFS.array, "ReadyMidCube");
        }

        public interface High {
            ArmBFSField kConeTipInSame = ArmBFSField.create(11, -22, Constraints.CONSTRAINT, ReadyHighTipInSameBFS.array, "ReadyHighTipInSame");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(0, -30, Constraints.CONSTRAINT, ReadyHighTipInOppositeBFS.array, "ReadyHighTipInOpposite");
            
            ArmBFSField kCube = ArmBFSField.create(-20, 70, Constraints.CONSTRAINT, ReadyHighCubeBFS.array, "ReadyHighCube");
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
            ArmBFSField kConeTipInSame = ArmBFSField.create(-8, -28, Constraints.CONSTRAINT, ScoreMidTipInSameBFS.array, "ScoreMidTipInSame");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(-5, -90, Constraints.CONSTRAINT, ScoreMidTipInOppositeBFS.array, "ScoreMidTipInOpposite");
            
            ArmBFSField kConeTipOutSame = ArmBFSField.create(-35, 90, Constraints.CONSTRAINT, ScoreMidTipOutSameBFS.array, "ScoreMidTipOutSame");
            
            ArmBFSField kCube = Ready.Mid.kCube;
        }

        public interface High {
            ArmBFSField kConeTipInSame = ArmBFSField.create(11, -32, Constraints.CONSTRAINT, ScoreHighTipInSameBFS.array, "ScoreHighTipInSame");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(0, -70, Constraints.CONSTRAINT, ScoreHighTipInOppositeBFS.array, "ScoreHighTipInOpposite");
            
            ArmBFSField kCube = Ready.High.kCube;
        }
    }

}
