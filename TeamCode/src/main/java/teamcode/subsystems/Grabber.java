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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.RobotParams;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements a Grabber Subsystem.
 */
public class Grabber
{
    private final Rev2mDistanceSensor rev2mSensor;
    private final TrcServoGrabber grabber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Grabber()
    {
        if (RobotParams.Grabber.USE_REV_2M_SENSOR)
        {
            rev2mSensor = FtcOpMode.getInstance().hardwareMap.get(
                Rev2mDistanceSensor.class, RobotParams.Grabber.REV_2M_SENSOR_NAME);
        }
        else
        {
            rev2mSensor = null;
        }

        FtcServoGrabber.Params grabberParams = new FtcServoGrabber.Params()
            .setPrimaryServo(RobotParams.Grabber.PRIMARY_SERVO_NAME, RobotParams.Grabber.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(RobotParams.Grabber.FOLLOWER_SERVO_NAME, RobotParams.Grabber.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(RobotParams.Grabber.OPEN_POS, RobotParams.Grabber.OPEN_TIME,
                                RobotParams.Grabber.CLOSE_POS, RobotParams.Grabber.CLOSE_TIME);

        if (rev2mSensor != null)
        {
            grabberParams.setAnalogSensorTrigger(
                this::getSensorData, RobotParams.Grabber.ANALOG_TRIGGER_INVERTED,
                RobotParams.Grabber.SENSOR_TRIGGER_THRESHOLD, RobotParams.Grabber.HAS_OBJECT_THRESHOLD,
                null);
        }
        else if (RobotParams.Grabber.USE_DIGITAL_SENSOR)
        {
            grabberParams.setDigitalInputTrigger(
                RobotParams.Grabber.DIGITAL_SENSOR_NAME, RobotParams.Grabber.DIGITAL_TRIGGER_INVERTED, null);
        }

        grabber = new FtcServoGrabber(RobotParams.Grabber.SUBSYSTEM_NAME, grabberParams).getGrabber();
        grabber.open();
    }

    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }

    private double getSensorData()
    {
        if (rev2mSensor != null)
        {
            return rev2mSensor.getDistance(DistanceUnit.INCH);
        }
        else
        {
            return 0.0;
        }
    }

}   //class Grabber
