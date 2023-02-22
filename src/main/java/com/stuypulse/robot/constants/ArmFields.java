package com.stuypulse.robot.constants;

import java.io.File;

import com.stuypulse.robot.util.ArmBFSField;
import com.stuypulse.robot.util.FieldFileUtil;
import com.stuypulse.stuylib.util.StopWatch;

public final class ArmFields {

    public static void load() {
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
        ArmBFSField kTrajectory = ArmBFSField.create(-70, 5, Constraints.CONSTRAINT, "Intake");
        // ArmBFSField kBackTrajectory = ArmBFSField.create(-103, 170, Constraints.CONSTRAINT);
    }

    public interface Neutral {
        ArmBFSField kTrajectory = ArmBFSField.create(-90, +90, Constraints.CONSTRAINT, "Neutral");
    }

    /* Intaking */

    /* Ready */

    public interface Ready {
        public interface Low {
            ArmBFSField kConeTipInSame = Intake.kTrajectory;
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(-40, -150, Constraints.CONSTRAINT, "Ready Low Tip In Opposite");
            
            ArmBFSField kConeTipOutSame = Intake.kTrajectory;
            ArmBFSField kConeTipOutOpposite = kConeTipInOpposite;

            ArmBFSField kCube = Intake.kTrajectory;
        }

        public interface Mid {
            ArmBFSField kConeTipInSame = ArmBFSField.create(-5, -20, Constraints.CONSTRAINT, "Ready Mid Tip In Same");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(0, -60, Constraints.CONSTRAINT, "Ready Mid Tip In Opposite");
            
            ArmBFSField kConeTipOutSame = ArmBFSField.create(-20, 85, Constraints.CONSTRAINT, "Ready Mid Top Out Same");
            
            ArmBFSField kCube = ArmBFSField.create(-45, 60, Constraints.CONSTRAINT, "Ready Mid Cube");
        }

        public interface High {
            ArmBFSField kConeTipInSame = ArmBFSField.create(11, -22, Constraints.CONSTRAINT, "Ready High Tip In Same");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(0, -30, Constraints.CONSTRAINT, "Ready High Tip In Opposite");
            
            ArmBFSField kCube = ArmBFSField.create(-20, 70, Constraints.CONSTRAINT, "Ready High Cube");
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
            ArmBFSField kConeTipInSame = ArmBFSField.create(-8, -28, Constraints.CONSTRAINT, "Score Mid Tip In Same");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(-5, -90, Constraints.CONSTRAINT, "Score Mid Tip In Opposite");
            
            ArmBFSField kConeTipOutSame = ArmBFSField.create(-35, 90, Constraints.CONSTRAINT, "Score Mid Tip Out Same");
            
            ArmBFSField kCube = Ready.Mid.kCube;
        }

        public interface High {
            ArmBFSField kConeTipInSame = ArmBFSField.create(11, -32, Constraints.CONSTRAINT, "Score High Tip In Same");
            ArmBFSField kConeTipInOpposite = ArmBFSField.create(0, -70, Constraints.CONSTRAINT, "Score High Tip In Opposite");
            
            ArmBFSField kCube = Ready.High.kCube;
        }
    }

    public static void main(String... args) {
        File[] files = FieldFileUtil.getFieldDirectory().listFiles();

        if (files != null) {
            for (File f : files) {
                f.delete();
            }
        }

        System.out.println("Deleted old bfs fields");
    }

}
