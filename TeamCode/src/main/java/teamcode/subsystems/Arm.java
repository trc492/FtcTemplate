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
import teamcode.RobotParams;
import trclib.motor.TrcMotor;

/**
 * This class implements an Arm Subsystem.
 */
public class Arm
{
    private final TrcMotor armMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Arm()
    {
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(RobotParams.Arm.MOTOR_NAME, RobotParams.Arm.MOTOR_TYPE, RobotParams.Arm.MOTOR_INVERTED)
            .setPositionScaleAndOffset(RobotParams.Arm.DEG_PER_COUNT, RobotParams.Arm.POS_OFFSET)
            .setPositionPresets(RobotParams.Arm.POS_PRESET_TOLERANCE, RobotParams.Arm.posPresets);
        armMotor = new FtcMotorActuator(motorParams).getMotor();
        armMotor.setSoftwarePidEnabled(RobotParams.Arm.SOFTWARE_PID_ENABLED);
        armMotor.setPositionPidParameters(RobotParams.Arm.posPidCoeffs, RobotParams.Arm.POS_PID_TOLERANCE);
        armMotor.setPositionPidPowerComp(this::getGravityComp);
        armMotor.setStallProtection(
            RobotParams.Arm.STALL_MIN_POWER, RobotParams.Arm.STALL_TOLERANCE,
            RobotParams.Arm.STALL_TIMEOUT, RobotParams.Arm.STALL_RESET_TIMEOUT);
    }

    public TrcMotor getArmMotor()
    {
        return armMotor;
    }

    private double getGravityComp(double currPower)
    {
        return RobotParams.Arm.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(armMotor.getPosition()));
    }

}   //class Arm
