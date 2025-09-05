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

package teamcode.subsystems;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.driverio.FtcGobildaIndicatorLight;
import ftclib.driverio.FtcRevBlinkin;
import trclib.drivebase.TrcDriveBase;
import trclib.driverio.TrcGobildaIndicatorLight;
import trclib.driverio.TrcPriorityIndicator;
import trclib.driverio.TrcRevBlinkin;

/**
 * This class encapsulates the LED controller to provide a priority indicator showing the status of the robot.
 */
public class LEDIndicator
{
    // LED pattern names.
    public static final String RED_BLOB = "RedBlob";
    public static final String BLUE_BLOB = "BlueBlob";
    public static final String FOUND_NOTHING = "FoundNothing";
    public static final String APRIL_TAG = "AprilTag";
    public static final String DRIVE_ORIENTATION_FIELD = "FieldMode";
    public static final String DRIVE_ORIENTATION_ROBOT = "RobotMode";
    public static final String DRIVE_ORIENTATION_INVERTED = "InvertedMode";
    public static final String OFF_PATTERN = "Off";

    private final TrcPriorityIndicator<?> indicator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param indicatorName specifies the indicator hardware name.
     * @param ledType specifies the LED type.
     */
    public LEDIndicator(String indicatorName, FtcRobotDrive.LEDType ledType)
    {
        if (ledType == FtcRobotDrive.LEDType.GobildaLEDIndicator)
        {
            // LED Patterns are sorted in decreasing priority order.
            final TrcGobildaIndicatorLight.Pattern[] ledPatternPriorities = {
                // Highest priority.
                new TrcGobildaIndicatorLight.Pattern(RED_BLOB, TrcGobildaIndicatorLight.Color.Red),
                new TrcGobildaIndicatorLight.Pattern(BLUE_BLOB, TrcGobildaIndicatorLight.Color.Blue),
                new TrcGobildaIndicatorLight.Pattern(FOUND_NOTHING, TrcGobildaIndicatorLight.Color.Cyan),
                new TrcGobildaIndicatorLight.Pattern(APRIL_TAG, TrcGobildaIndicatorLight.Color.Green),
                new TrcGobildaIndicatorLight.Pattern(DRIVE_ORIENTATION_FIELD, TrcGobildaIndicatorLight.Color.Violet),
                new TrcGobildaIndicatorLight.Pattern(DRIVE_ORIENTATION_ROBOT, TrcGobildaIndicatorLight.Color.White),
                new TrcGobildaIndicatorLight.Pattern(DRIVE_ORIENTATION_INVERTED, TrcGobildaIndicatorLight.Color.Orange),
                new TrcGobildaIndicatorLight.Pattern(OFF_PATTERN, TrcGobildaIndicatorLight.Color.Black)
                // Lowest priority.
            };
            indicator = new FtcGobildaIndicatorLight(indicatorName);
            ((FtcGobildaIndicatorLight) indicator).setPatternPriorities(ledPatternPriorities);
        }
        else if (ledType == FtcRobotDrive.LEDType.RevBlinkin)
        {
            // LED Patterns are sorted in decreasing priority order.
            final TrcRevBlinkin.Pattern[] ledPatternPriorities = {
                // Highest priority.
                new TrcRevBlinkin.Pattern(RED_BLOB, TrcRevBlinkin.RevLedPattern.SolidRed),
                new TrcRevBlinkin.Pattern(BLUE_BLOB, TrcRevBlinkin.RevLedPattern.SolidBlue),
                new TrcRevBlinkin.Pattern(FOUND_NOTHING, TrcRevBlinkin.RevLedPattern.SolidAqua),
                new TrcRevBlinkin.Pattern(APRIL_TAG, TrcRevBlinkin.RevLedPattern.SolidGreen),
                new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_FIELD, TrcRevBlinkin.RevLedPattern.SolidViolet),
                new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_ROBOT, TrcRevBlinkin.RevLedPattern.SolidWhite),
                new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_INVERTED, TrcRevBlinkin.RevLedPattern.SolidOrange),
                new TrcRevBlinkin.Pattern(OFF_PATTERN, TrcRevBlinkin.RevLedPattern.SolidBlack)
                // Lowest priority.
            };
            indicator = new FtcRevBlinkin(indicatorName);
            ((FtcRevBlinkin) indicator).setPatternPriorities(ledPatternPriorities);
        }
        else
        {
            indicator = null;
        }
    }   //LEDIndicator

    /**
     * This method sets the pattern ON for a period of time and turns off automatically afterwards.
     *
     * @param patternName specifies the name of the LED pattern to turn on.
     */
    public void setDetectedPattern(String patternName)
    {
        if (indicator != null)
        {
            indicator.setPatternState(patternName, true, 0.5);
        }
    }   //setDetectedPattern

    /**
     * This method sets the LED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (indicator != null)
        {
            switch (orientation)
            {
                case INVERTED:
                    indicator.setPatternState(DRIVE_ORIENTATION_INVERTED, true);
                    indicator.setPatternState(DRIVE_ORIENTATION_ROBOT, false);
                    indicator.setPatternState(DRIVE_ORIENTATION_FIELD, false);
                    break;

                case ROBOT:
                    indicator.setPatternState(DRIVE_ORIENTATION_INVERTED, false);
                    indicator.setPatternState(DRIVE_ORIENTATION_ROBOT, true);
                    indicator.setPatternState(DRIVE_ORIENTATION_FIELD, false);
                    break;

                case FIELD:
                    indicator.setPatternState(DRIVE_ORIENTATION_INVERTED, false);
                    indicator.setPatternState(DRIVE_ORIENTATION_ROBOT, false);
                    indicator.setPatternState(DRIVE_ORIENTATION_FIELD, true);
                    break;
            }
        }
    }   //setDriveOrientation

}   //class LEDIndicator
