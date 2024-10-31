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
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements a Grabber Subsystem.
 */
public class Grabber
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_SERVO_INVERTED     = !PRIMARY_SERVO_INVERTED;

        public static final double OPEN_POS                     = 0.2;
        public static final double OPEN_TIME                    = 0.5;
        public static final double CLOSE_POS                    = 0.55;
        public static final double CLOSE_TIME                   = 0.5;

        public static final boolean USE_ANALOG_SENSOR           = true;
        public static final String ANALOG_SENSOR_NAME           = SUBSYSTEM_NAME + ".sensor";
        public static final double SENSOR_TRIGGER_THRESHOLD     = 2.0;
        public static final boolean ANALOG_TRIGGER_INVERTED     = true;

        public static final boolean USE_DIGITAL_SENSOR          = false;
        public static final String DIGITAL_SENSOR_NAME          = SUBSYSTEM_NAME + ".sensor";
        public static final boolean DIGITAL_TRIGGER_INVERTED    = false;
    }   //class Params

    private final Rev2mDistanceSensor analogSensor;
    private final TrcServoGrabber grabber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Grabber()
    {
        if (Params.USE_ANALOG_SENSOR)
        {
            analogSensor = FtcOpMode.getInstance().hardwareMap.get(
                Rev2mDistanceSensor.class, Params.ANALOG_SENSOR_NAME);
        }
        else
        {
            analogSensor = null;
        }

        FtcServoGrabber.Params grabberParams = new FtcServoGrabber.Params()
            .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(Params.OPEN_POS, Params.OPEN_TIME, Params.CLOSE_POS, Params.CLOSE_TIME);

        if (analogSensor != null)
        {
            grabberParams.setAnalogSensorTrigger(
                this::getSensorData, Params.ANALOG_TRIGGER_INVERTED, Params.SENSOR_TRIGGER_THRESHOLD);
        }
        else if (Params.USE_DIGITAL_SENSOR)
        {
            grabberParams.setDigitalInputTrigger(Params.DIGITAL_SENSOR_NAME, Params.DIGITAL_TRIGGER_INVERTED);
        }

        grabber = new FtcServoGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
        grabber.open();
    }

    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }

    private double getSensorData()
    {
        if (analogSensor != null)
        {
            return analogSensor.getDistance(DistanceUnit.INCH);
        }
        else
        {
            return 0.0;
        }
    }

}   //class Grabber
