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
    private final TrcShooter shooter;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter()
    {
        FtcShooter.Params shooterParams = new FtcShooter.Params()
            .setShooterMotor1(
                RobotParams.Shooter.MOTOR1_NAME, RobotParams.Shooter.MOTOR1_TYPE, RobotParams.Shooter.MOTOR1_INVERTED);
        if (RobotParams.Shooter.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                RobotParams.Shooter.MOTOR2_NAME, RobotParams.Shooter.MOTOR2_TYPE, RobotParams.Shooter.MOTOR2_INVERTED);
        }
        shooter = new FtcShooter(RobotParams.Shooter.SUBSYSTEM_NAME, shooterParams).getShooter();
        configShooterMotor(shooter.getShooterMotor1(), RobotParams.Shooter.shooter1PidCoeffs);
        TrcMotor shooterMotor2 = shooter.getShooterMotor2();
        if (shooterMotor2 != null)
        {
            configShooterMotor(shooterMotor2, RobotParams.Shooter.shooter2PidCoeffs);
        }
    }

    public TrcShooter getShooter()
    {
        return shooter;
    }

    private void configShooterMotor(TrcMotor motor, TrcPidController.PidCoefficients pidCoeffs)
    {
        motor.setPositionSensorScaleAndOffset(RobotParams.Shooter.GOBILDA1620_RPC, 0.0);
        motor.setSoftwarePidEnabled(RobotParams.Shooter.SOFTWARE_PID_ENABLED);
        motor.setVelocityPidParameters(pidCoeffs, RobotParams.Shooter.SHOOTER_PID_TOLERANCE);
    }   //configShooterMotor

}   //class Shooter
