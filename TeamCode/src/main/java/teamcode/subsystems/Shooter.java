/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import ftclib.subsystem.FtcShooter;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcPidController;
import trclib.subsystem.TrcShooter;

/**
 * This class implements an Shooter Subsystem.
 */
public class Shooter
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Shooter";

        public static final String MOTOR1_NAME                  = SUBSYSTEM_NAME + ".motor1";
        public static final FtcMotorActuator.MotorType MOTOR1_TYPE= FtcMotorActuator.MotorType.DcMotor;
        public static final boolean MOTOR1_INVERTED             = false;

        public static final boolean HAS_TWO_SHOOTER_MOTORS      = true;
        public static final String MOTOR2_NAME                  = SUBSYSTEM_NAME + ".motor2";
        public static final FtcMotorActuator.MotorType MOTOR2_TYPE= FtcMotorActuator.MotorType.DcMotor;
        public static final boolean MOTOR2_INVERTED             = true;

        public static final double GOBILDA1620_RPC              = 1.0 / ((1.0 + (46.0/17.0)) * 28.0);
        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients shooter1PidCoeffs =
            new TrcPidController.PidCoefficients(0.025, 0.0, 0.0, 0.039, 0.0);
        public static final TrcPidController.PidCoefficients shooter2PidCoeffs =
            new TrcPidController.PidCoefficients(0.025, 0.0, 0.0, 0.041, 0.0);
        public static final double SHOOTER_PID_TOLERANCE        = 10.0;

        public static final double SHOOTER_MIN_VEL              = 10.0;     // in RPM
        public static final double SHOOTER_MAX_VEL              = 1620.0;   // in RPM
        public static final double SHOOTER_MIN_VEL_INC          = 1.0;      // in RPM
        public static final double SHOOTER_MAX_VEL_INC          = 100.0;    // in RPM
        public static final double SHOOTER_DEF_VEL              = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL_INC          = 10.0;     // in RPM
    }   //class Params

    private final TrcShooter shooter;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter()
    {
        FtcShooter.Params shooterParams = new FtcShooter.Params()
            .setShooterMotor1(Params.MOTOR1_NAME, Params.MOTOR1_TYPE, Params.MOTOR1_INVERTED);
        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(Params.MOTOR2_NAME, Params.MOTOR2_TYPE, Params.MOTOR2_INVERTED);
        }
        shooter = new FtcShooter(Params.SUBSYSTEM_NAME, shooterParams).getShooter();
        configShooterMotor(shooter.getShooterMotor1(), Params.shooter1PidCoeffs);
        TrcMotor shooterMotor2 = shooter.getShooterMotor2();
        if (shooterMotor2 != null)
        {
            configShooterMotor(shooterMotor2, Params.shooter2PidCoeffs);
        }
    }

    public TrcShooter getShooter()
    {
        return shooter;
    }

    private void configShooterMotor(TrcMotor motor, TrcPidController.PidCoefficients pidCoeffs)
    {
        motor.setPositionSensorScaleAndOffset(Params.GOBILDA1620_RPC, 0.0);
        motor.setSoftwarePidEnabled(Params.SOFTWARE_PID_ENABLED);
        motor.setVelocityPidParameters(pidCoeffs, Params.SHOOTER_PID_TOLERANCE);
    }   //configShooterMotor

}   //class Shooter
