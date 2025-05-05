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

import ftclib.driverio.FtcDashboard;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoClaw;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcServoClaw;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Claw Subsystem. This implementation consists of two servos and a sensor to detect the
 * presence of an object and can auto grab it. The sensor can be either a digital sensor such as touch sensor or beam
 * break sensor) or an analog sensor such as a distance sensor.
 * There are many possible implementations by setting different parameters.
 * Please refer to the TrcLib documentation (<a href="https://trc492.github.io">...</a>) for details.
 */
public class Claw extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Claw";
        public static final boolean NEED_ZERO_CAL               = false;

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

    private final FtcDashboard dashboard;
    private final Rev2mDistanceSensor analogSensor;
    private final TrcServoClaw claw;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Claw()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        if (Params.USE_ANALOG_SENSOR)
        {
            analogSensor = FtcOpMode.getInstance().hardwareMap.get(
                Rev2mDistanceSensor.class, Params.ANALOG_SENSOR_NAME);
        }
        else
        {
            analogSensor = null;
        }

        FtcServoClaw.Params clawParams = new FtcServoClaw.Params()
            .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(Params.OPEN_POS, Params.OPEN_TIME, Params.CLOSE_POS, Params.CLOSE_TIME);

        if (analogSensor != null)
        {
            clawParams.setAnalogSensorTrigger(
                this::getSensorData, Params.ANALOG_TRIGGER_INVERTED, Params.SENSOR_TRIGGER_THRESHOLD);
        }
        else if (Params.USE_DIGITAL_SENSOR)
        {
            clawParams.setDigitalInputTrigger(Params.DIGITAL_SENSOR_NAME, Params.DIGITAL_TRIGGER_INVERTED);
        }

        claw = new FtcServoClaw(Params.SUBSYSTEM_NAME, clawParams).getClaw();
        claw.open();
    }   //Claw

    /**
     * This method returns the created Servo Claw object.
     *
     * @return created claw.
     */
    public TrcServoClaw getClaw()
    {
        return claw;
    }   //getClaw

    /**
     * This method returns the current sensor value if it has one.
     *
     * @return sensor value if there is a sensor, 0 if there is none.
     */
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
    }   //getSensorData

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        claw.cancel();
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // No zero calibration needed.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        claw.open();
    }   //resetState

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum)
    {
        dashboard.displayPrintf(
            lineNum++, "%s: pos=%s, hasObject=%s, sensor=%s, autoActive=%s",
            Params.SUBSYSTEM_NAME, claw.isClosed()? "closed": "open", claw.hasObject(),
            analogSensor != null? claw.getSensorValue(): claw.getSensorState(), claw.isAutoActive());
        return lineNum;
    }   //updateStatus

}   //class Claw
