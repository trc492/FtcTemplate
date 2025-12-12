/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode.indicators;

import ftclib.driverio.FtcGobildaIndicatorLight;
import ftclib.driverio.FtcRevBlinkin;
import trclib.drivebase.TrcDriveBase;
import trclib.driverio.TrcGobildaIndicatorLight;
import trclib.driverio.TrcPriorityIndicator;
import trclib.driverio.TrcRevBlinkin;
import trclib.robotcore.TrcDbgTrace;

/**
 * This class encapsulates the LED controller to provide a priority indicator showing the status of the robot.
 */
public class LEDIndicator
{
    private static final String moduleName = LEDIndicator.class.getSimpleName();
    // LED device names.
    public static final String STATUS_LED_NAME = "StatusLED";
    public static final String COLOR_BLOB_LED_NAME = "ColorBlobLED";
    // LED pattern names.
    public static final String RED_BLOB = "RedBlob";
    public static final String BLUE_BLOB = "BlueBlob";
    public static final String RED_APRILTAG = "RedAprilTag";
    public static final String BLUE_APRILTAG = "BlueAprilTag";
    public static final String NOT_FOUND = "NotFound";
    public static final String SEARCHING_RED_APRILTAG = "SearchingRedAprilTag";
    public static final String SEARCHING_BLUE_APRILTAG = "SearchingBlueAprilTag";
    public static final String DRIVE_FIELD_MODE = "FieldMode";
    public static final String DRIVE_ROBOT_MODE = "RobotMode";
    public static final String DRIVE_INVERTED_MODE = "InvertedMode";
    public static final String OFF_PATTERN = "Off";

    public final TrcPriorityIndicator.Pattern[] statusLEDPatternPriorities = new TrcPriorityIndicator.Pattern[]
        {
            // Highest priority.
            new TrcPriorityIndicator.Pattern(RED_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidRed, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(BLUE_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidBlue, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(NOT_FOUND, TrcRevBlinkin.RevLedPattern.SolidYellow, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(SEARCHING_RED_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidRed, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(SEARCHING_BLUE_APRILTAG, TrcRevBlinkin.RevLedPattern.SolidBlue, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(DRIVE_FIELD_MODE, TrcRevBlinkin.RevLedPattern.SolidAqua, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(DRIVE_ROBOT_MODE, TrcRevBlinkin.RevLedPattern.SolidWhite, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(DRIVE_INVERTED_MODE, TrcRevBlinkin.RevLedPattern.SolidOrange, 0.5, 0.0),
            new TrcPriorityIndicator.Pattern(OFF_PATTERN, TrcRevBlinkin.RevLedPattern.SolidBlack)
            // Lowest priority.
        };

    public final TrcPriorityIndicator.Pattern[] colorBlobLEDPatternPriorities = new TrcPriorityIndicator.Pattern[]
        {
            // Highest priority.
            new TrcPriorityIndicator.Pattern(RED_BLOB, TrcGobildaIndicatorLight.GobildaLedPattern.Red, 0.25, 0.25),
            new TrcPriorityIndicator.Pattern(BLUE_BLOB, TrcGobildaIndicatorLight.GobildaLedPattern.Blue, 0.25, 0.25),
            new TrcPriorityIndicator.Pattern(OFF_PATTERN, TrcGobildaIndicatorLight.GobildaLedPattern.Black)
            // Lowest priority.
        };

    public final TrcDbgTrace tracer;
    private TrcPriorityIndicator statusIndicator = null;
    private TrcPriorityIndicator colorBlobIndicator = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param indicatorNames specifies an array of indicator hardware names, one for each LED device.
     */
    public LEDIndicator(String[] indicatorNames)
    {
        tracer = new TrcDbgTrace();

        for (String indicatorName: indicatorNames)
        {
            switch (indicatorName)
            {
                case STATUS_LED_NAME:
                    if (statusIndicator != null)
                    {
                        throw new IllegalArgumentException("statusIndicator already exists.");
                    }
                    else
                    {
                        tracer.traceInfo(moduleName, "Creating " + indicatorName);
                        statusIndicator = new FtcRevBlinkin(indicatorName);
                        statusIndicator.setPatternPriorities(statusLEDPatternPriorities);
                        statusIndicator.reset();
                    }
                    break;

                case COLOR_BLOB_LED_NAME:
                    if (colorBlobIndicator != null)
                    {
                        throw new IllegalArgumentException("colorBlobIndicator already exists.");
                    }
                    else
                    {
                        tracer.traceInfo(moduleName, "Creating " + indicatorName);
                        colorBlobIndicator = new FtcGobildaIndicatorLight(indicatorName);
                        colorBlobIndicator.setPatternPriorities(colorBlobLEDPatternPriorities);
                        colorBlobIndicator.reset();
                    }
                    break;
            }
        }
    }   //LEDIndicator

    /**
     * This method turns all LED indicator patterns off.
     */
    public void reset()
    {
        if (statusIndicator != null)
        {
            statusIndicator.reset();
        }
    }   //reset

    /**
     * This method sets the statusLED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (statusIndicator != null)
        {
            switch (orientation)
            {
                case INVERTED:
                    statusIndicator.setPatternState(DRIVE_INVERTED_MODE, true);
                    statusIndicator.setPatternState(DRIVE_ROBOT_MODE, false);
                    statusIndicator.setPatternState(DRIVE_FIELD_MODE, false);
                    break;

                case ROBOT:
                    statusIndicator.setPatternState(DRIVE_INVERTED_MODE, false);
                    statusIndicator.setPatternState(DRIVE_ROBOT_MODE, true);
                    statusIndicator.setPatternState(DRIVE_FIELD_MODE, false);
                    break;

                case FIELD:
                    statusIndicator.setPatternState(DRIVE_INVERTED_MODE, false);
                    statusIndicator.setPatternState(DRIVE_ROBOT_MODE, false);
                    statusIndicator.setPatternState(DRIVE_FIELD_MODE, true);
                    break;
            }
        }
    }   //setDriveOrientation

    /**
     * This method sets the statusLED pattern ON or OFF.
     *
     * @param patternName specifies the name of the LED pattern to turn on.
     */
    public void setStatusPattern(String patternName, boolean on)
    {
        if (statusIndicator != null)
        {
            statusIndicator.setPatternState(patternName, on);
        }
    }   //setStatusPattern

    /**
     * This method clears all vision detected states.
     */
    public void setAprilTagPatternsOff()
    {
        if (statusIndicator != null)
        {
            statusIndicator.setPatternState(RED_APRILTAG, false);
            statusIndicator.setPatternState(BLUE_APRILTAG, false);
            statusIndicator.setPatternState(NOT_FOUND, false);
        }
    }   //setAprilTagPatternsOff

    /**
     * This method sets the ColorBlob LED pattern state.
     *
     * @param colorBlobName specifies the colorblob name.
     * @param on specifies true to turn it ON, false to turn if OFF.
     */
    public void setColorBlobPatternState(String colorBlobName, boolean on)
    {
        if (colorBlobIndicator != null)
        {
            colorBlobIndicator.setPatternState(colorBlobName, on);
        }
    }   //setColorBlobPatternState

}   //class LEDIndicator
