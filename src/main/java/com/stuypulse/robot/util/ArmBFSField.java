package com.stuypulse.robot.util;

import java.util.ArrayDeque;
import java.util.Queue;

public class ArmBFSField {

    public interface Constraint {
        public boolean isInvalid(double armDeg, double wristDeg);

        public default Constraint add(Constraint next) {
            return (a, w) -> this.isInvalid(a, w) || next.isInvalid(a, w);
        }
    }

    public static final int kDegreeRange = 360;

    // This value does not matter as much as long as its small.
    public static final double kConstraintDistCost = (1 / 16.0);
    public static final double kEpsilon = kConstraintDistCost / 2.0;

    public static final int kArmNodeSpeed = 1;
    public static final int kWristNodeSpeed = 1;

    // Amount to downscale the field by
    // kBinning = 1) 360 x 360
    // kBinning = 2) 180 x 180
    // kBinning = 3) 120 x 120
    public static final int kBinning = 4;

    private static int normalize(int degrees) {
        return degrees - kDegreeRange * Math.floorDiv(degrees, kDegreeRange);
    }

    private static double normalizeZero(double degrees) {
        return degrees - kDegreeRange * Math.round(degrees / kDegreeRange);
    }

    private static double distance(double dx, double dy) {
        // return Math.hypot(dx, dy); // Diagonal
        return dx + dy;
    }

    public class Node {

        private final boolean mValid;

        private final int mArmDeg;
        private final int mWristDeg;

        private double mConstraintCost;
        private double mSetpointCost;

        private Node mNextNode;

        public Node(int armDeg, int wristDeg) {
            mArmDeg = normalize(armDeg);
            mWristDeg = normalize(wristDeg);

            mValid = !mConstraints.isInvalid(getArmDeg(), getWristDeg());

            mConstraintCost = mValid ? Double.MAX_VALUE : 0.0;
            mSetpointCost = Double.MAX_VALUE;

            mNextNode = this;
        }

        private double getDistanceCost(Node previous) {
            final double dx = normalizeZero(this.mArmDeg - previous.mArmDeg);
            final double dy = normalizeZero(this.mWristDeg - previous.mWristDeg);
            return distance(dx, dy);
        }

        private Node makeSetpoint() {
            if (!isValid())
                throw new RuntimeException("Setpoint (" + getArmDeg() + ", " + getWristDeg()  + ") is within constraint!");

            mNextNode = this;
            mSetpointCost = 0;
            return this;
        }

        private boolean expandConstraint(Node previous) {
            if (this == previous) {
                return false;
            }

            final double cost = this.getDistanceCost(previous) * kConstraintDistCost;
            final double newConstraintCost = previous.mConstraintCost + cost;

            if (newConstraintCost + kEpsilon < mConstraintCost) {
                mConstraintCost = newConstraintCost;
                return true;
            }

            return false;
        }

        private boolean expandSearch(Node previous) {
            // do not expand searches into invalid areas
            if (this == previous || this == previous.mNextNode || !this.isValid() || !previous.isValid()) {
                return false;
            }

            // the difference in constraint cost between adjacent nodes is small
            // so this will basically find the shortest path, but if there is an
            // equidistant path that is further from the constraints, take that one instead,
            final double cost = this.getDistanceCost(previous);
            final double newSetpointCost = previous.mSetpointCost + cost;
            final double newCost = newSetpointCost - previous.mConstraintCost;
            final double oldCost = mSetpointCost - mNextNode.mConstraintCost;

            if (newCost + kEpsilon < oldCost) {
                mSetpointCost = newSetpointCost;
                mNextNode = previous;

                return true;
            }

            return false;
        }

        private boolean expandEscape(Node previous) {
            // a valid node should not escape as it is valid
            if (this == previous || this == previous.mNextNode || this.isValid()) {
                return false;
            }

            // when escaping you only want to move one mech at a time
            if (mArmDeg != previous.mArmDeg && mWristDeg != previous.mWristDeg) {
                return false;
            }

            // if there is a new shorter escape path, take it
            final double cost = this.getDistanceCost(previous);
            final double newSetpointCost = previous.isValid() ? 0.0 : previous.mSetpointCost + cost;

            if (newSetpointCost + kEpsilon < mSetpointCost) {
                mSetpointCost = newSetpointCost;
                mNextNode = previous;

                return true;
            }

            return false;
        }

        public boolean isSetpoint() {
            return this == next();
        }

        public boolean isValid() {
            return mValid;
        }

        public double getArmDeg() {
            return normalizeZero(mArmDeg + mArmDegOffset);
        }

        public double getWristDeg() {
            return normalizeZero(mWristDeg + mWristDegOffset);
        }

        public ArmState getArmState() {
            return ArmState.fromDegrees(getArmDeg(), getWristDeg());
        }

        private Node getNeighbor(int dx, int dy) {
            return getRawNode(mArmDeg + kBinning * dx, mWristDeg + kBinning * dy);
        }

        public Node next() {
            return mNextNode;
        }

        public Node next(int depth) {
            Node result = this;

            while (depth-- > 0) {
                result = result.next();
            }

            return result;
        }

        public Node travel(double dist) {
            Node result = this;

            while (dist > 0 && !result.isSetpoint()) {
                dist -= result.getDistanceCost(result.next());
                result = result.next();
            }

            return result;
        }
    }

    private ArmBFSField mFlipped;
    private final Constraint mConstraints;
    private final double mTargetArmDeg;
    private final double mTargetWristDeg;
    private final double mArmDegOffset;
    private final double mWristDegOffset;
    private final Node[] mNodeMap;
    
    private static int instances = 0;

    public ArmBFSField(double targetArmDeg, double targetWristDeg, Constraint constraints) {
        this(targetArmDeg, targetWristDeg, constraints, new ArmBFSField(-180 - targetArmDeg, -180 - targetWristDeg, constraints, null));
        mFlipped.mFlipped = this;
    }

    private ArmBFSField(double targetArmDeg, double targetWristDeg, Constraint constraints, ArmBFSField flipped) {
        mFlipped = flipped;

        mConstraints = constraints;

        mTargetArmDeg = normalizeZero(targetArmDeg);
        mTargetWristDeg = normalizeZero(targetWristDeg);

        mArmDegOffset = targetArmDeg - kBinning * Math.round(targetArmDeg / kBinning);
        mWristDegOffset = targetWristDeg - kBinning * Math.round(targetWristDeg / kBinning);

        mNodeMap = new Node[getIndex(kDegreeRange - 1, kDegreeRange - 1) + 1];

        Queue<Node> openSet = new ArrayDeque<>(kDegreeRange);

        // initialize all nodes and test to see if they fit the constraints
        for (int arm = 0; arm < kDegreeRange; arm += kBinning) {
            for (int wrist = 0; wrist < kDegreeRange; wrist += kBinning) {
                final Node node = new Node(arm, wrist);

                // if the node is a member of the constraints,
                // add it to the constraint openset for later
                if (!node.isValid()) {
                    openSet.add(node);
                }

                mNodeMap[getIndex(arm, wrist)] = node;
            }
        }

        // we want to be able to find the shortest path that also remains the
        // furthest from all the constraints. doing a search will let us determine
        // the distance each node is from the closest constraint.

        Node next;
        while ((next = openSet.poll()) != null) {
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    final Node node = next.getNeighbor(dx, dy);
                    if (node.expandConstraint(next)) {
                        openSet.add(node);
                    }
                }
            }
        }

        // do a standard BFS on all of the valid nodes.
        openSet.add(getNode(targetArmDeg, targetWristDeg).makeSetpoint());

        while ((next = openSet.poll()) != null) {
            for (int dx = -kArmNodeSpeed; dx <= kArmNodeSpeed; ++dx) {
                for (int dy = -kWristNodeSpeed; dy <= kWristNodeSpeed; ++dy) {
                    final Node node = next.getNeighbor(dx, dy);

                    if (node.expandSearch(next)) {
                        openSet.add(node);
                    }

                    // if a node is invalid here, it means that it is touching
                    // a valid node, so it will be the beginning of our escape path.
                    else if (node.expandEscape(next)) {
                        openSet.add(node);
                    }
                }
            }
        }

        System.out.println("Initialized " + ++instances + "/30 ArmBFSFields");
    }

    public ArmBFSField(ArmState setpointState, Constraint constraint) {
        this(setpointState.getShoulderState().getDegrees(), setpointState.getWristState().getDegrees(), constraint);
    }

    private int getIndex(int armDeg, int wristDeg) {
        armDeg = normalize(armDeg) / kBinning;
        wristDeg = normalize(wristDeg) / kBinning;
        return armDeg * (kDegreeRange / kBinning) + wristDeg;
    }

    private Node getRawNode(int armDeg, int wristDeg) {
        return mNodeMap[getIndex(armDeg, wristDeg)];
    }

    public Node getNode(double armDeg, double wristDeg) {
        return getRawNode(
                (int) Math.round(armDeg - mArmDegOffset + kBinning / 2.0),
                (int) Math.round(wristDeg - mWristDegOffset + kBinning / 2.0));
    }

    public Node getNode(ArmState measuredState) {
        return getNode(measuredState.getShoulderState().getDegrees(), measuredState.getWristState().getDegrees());
    }

    public ArmState getSetpoint() {
        return ArmState.fromDegrees(mTargetArmDeg, mTargetWristDeg);
    }

    public ArmBFSField flipped() {
        return mFlipped;
    }

    public int getSize() {
        return mNodeMap.length;
    }

    public static void main(String... args) {
        Constraint constraints = (a, w) -> a >= 0.0;
        constraints = constraints.add((a, w) -> (Math.abs(a + 90) < 30) && (w > 150 || (30 > w)));

        ArmBFSField field = new ArmBFSField(210.23, 289.43, constraints);
        Node node = field.getNode(330, 250).next();

        System.out.println("arm,wrist");
        while (!node.isSetpoint()) {
            System.out.println(node.getArmDeg() + "," + node.getWristDeg());
            node = node.travel(20);
        }

        /**
         * arm,wrist
         * > -31.769999999999982,-112.57
         * > -31.769999999999982,-132.57
         * > -33.76999999999998,-154.57
         * > -43.76999999999998,-174.57
         * > -53.76999999999998,165.43
         * > -65.76999999999998,149.43
         * > -85.76999999999998,149.43
         * > -105.77000000000001,149.43
         * > -123.77000000000001,157.43
         * > -133.77,177.43
         * > -143.77,-162.57
         * > -149.77,-142.57
         * > -149.77,-122.57
         * > -149.77,-102.57
         * > -149.77,-82.57
         */
    }
}