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

import ftclib.subsystem.FtcIntake;
import teamcode.RobotParams;
import trclib.subsystem.TrcIntake;

/**
 * This class implements an Intake Subsystem.
 */
public class Intake
{
    private final TrcIntake intake;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Intake()
    {
        FtcIntake.Params intakeParams = new FtcIntake.Params()
            .setPrimaryMotor(RobotParams.Intake.PRIMARY_MOTOR_NAME, RobotParams.Intake.PRIMARY_MOTOR_TYPE,
                             RobotParams.Intake.PRIMARY_MOTOR_INVERTED)
            .setEntryDigitalInput(RobotParams.Intake.SENSOR_NAME, RobotParams.Intake.SENSOR_INVERTED, null);
        if (RobotParams.Intake.TWO_MOTOR_INTAKE)
        {
            intakeParams.setFollowerMotor(
                RobotParams.Intake.FOLLOWER_MOTOR_NAME, RobotParams.Intake.FOLLOWER_MOTOR_TYPE,
                RobotParams.Intake.FOLLOWER_MOTOR_INVERTED);
        }
        intake = new FtcIntake(RobotParams.Intake.SUBSYSTEM_NAME, intakeParams).getIntake();
    }

    public TrcIntake getIntake()
    {
        return intake;
    }

}   //class Intake
