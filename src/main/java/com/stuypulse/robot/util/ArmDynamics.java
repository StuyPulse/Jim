/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;

// https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
// https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/arm/ArmDynamics.java
public class ArmDynamics {

    private static final double g = 9.80655;

    private ArmJoint shoulder;
    private ArmJoint wrist;

    public ArmDynamics(ArmJoint shoulder, ArmJoint wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
        var t = M(position).times(acceleration)
            .plus(C(position, velocity).times(velocity))
            .plus(Tg(position));

        return VecBuilder.fill(
            shoulder.motor.getVoltage(t.get(0, 0), velocity.get(0, 0)),
            wrist.motor.getVoltage(t.get(1, 0), velocity.get(1, 0)));
    }

    public Vector<N4> simulate(Vector<N4> state, Vector<N2> voltage, double dt) {
        return new Vector<>(
            NumericalIntegration.rkdp(
            // NumericalIntegration.rk4(
                (Matrix<N4, N1> x, Matrix<N2, N1> u) -> {
                  // x = current state, u = voltages, return = state derivatives

                  // Get vectors from state
                  var position = VecBuilder.fill(x.get(0, 0), x.get(1, 0));
                  var velocity = VecBuilder.fill(x.get(2, 0), x.get(3, 0));

                  // Calculate torque
                  var shoulderTorque =
                      shoulder
                          .motor
                          .getTorque(shoulder.motor.getCurrent(velocity.get(0, 0), u.get(0, 0)));
                  var wristTorque =
                      wrist
                          .motor
                          .getTorque(wrist.motor.getCurrent(velocity.get(1, 0), u.get(1, 0)));
                  var torque = VecBuilder.fill(shoulderTorque, wristTorque);

                  // Calculate acceleration
                  var acceleration =
                      M(position)
                          .inv()
                          .times(
                              torque.minus(C(position, velocity).times(velocity)).minus(Tg(position)));

                  // Return state vector
                  return new MatBuilder<>(Nat.N4(), Nat.N1())
                      .fill(
                          velocity.get(0, 0),
                          velocity.get(1, 0),
                          acceleration.get(0, 0),
                          acceleration.get(1, 0));
                },
                state,
                voltage,
                dt));
      }

    private Matrix<N2, N2> M(Vector<N2> position) {
        var M = new Matrix<>(N2.instance, N2.instance);

        M.set(0, 0,
            shoulder.mass * Math.pow(shoulder.radius, 2) +
            wrist.mass * (Math.pow(shoulder.length, 2) + Math.pow(wrist.radius, 2)) +
            shoulder.moi +
            wrist.moi +
            2 * wrist.mass * shoulder.length * wrist.radius * Math.cos(position.get(1, 0)));

        M.set(1, 0,
            wrist.mass * Math.pow(wrist.radius, 2) +
            wrist.moi +
            wrist.mass * shoulder.length * wrist.radius * Math.cos(position.get(1, 0)));

        M.set(0, 1,
            wrist.mass * Math.pow(wrist.radius, 2) +
            wrist.moi +
            wrist.mass * shoulder.length * wrist.radius * Math.cos(position.get(1, 0)));

        M.set(1, 1,
            wrist.mass * Math.pow(wrist.radius, 2) + wrist.moi);

        return M;
    }

    private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
        var C = new Matrix<>(N2.instance, N2.instance);

        C.set(0, 0,
            -wrist.mass * shoulder.length * wrist.radius * Math.sin(position.get(1, 0)) * velocity.get(1, 0));

        C.set(0, 1,
            -wrist.mass * shoulder.length * wrist.radius * Math.sin(position.get(1, 0) * (velocity.get(0, 0) + velocity.get(1, 0))));

        C.set(1, 0,
            wrist.mass * shoulder.length * wrist.radius * Math.sin(position.get(1, 0)) * velocity.get(0, 0));

        C.set(1, 1, 0);

        return C;
    }

    private Matrix<N2, N1> Tg(Vector<N2> position) {
        var Tg = new Matrix<>(N2.instance, N1.instance);

        // Tg.set(0, 0,
        //     (shoulder.mass * shoulder.radius + wrist.mass * shoulder.length) * g * Math.cos(position.get(0, 0)) +
        //     wrist.mass * wrist.radius * g * Math.cos(position.get(0, 0) + position.get(1, 0)));

        // Tg.set(1, 0,
        //     wrist.mass * wrist.radius * g * Math.cos(position.get(0, 0) + position.get(1, 0)));

        return Tg;
    }
}
