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

import ftclib.driverio.FtcGamepad;
import ftclib.driverio.FtcGamepadRumble;
import trclib.driverio.TrcPriorityIndicator;

/**
 * This class encapsulates the gamepad rumble to provide a priority indicator showing the status of the robot.
 */
public class RumbleIndicator
{
    public static final String ENDGAME_DEADLINE = "EndGameDeadline";
    public static final FtcGamepadRumble.RumblePattern endGameRumble =
        new FtcGamepadRumble.RumblePattern("EndGameRumble", 1.0, 1.0, 0.5);
    public static final TrcPriorityIndicator.Pattern[] rumblePatternPriorities =
    {
        // Highest priority.
        new TrcPriorityIndicator.Pattern(ENDGAME_DEADLINE, endGameRumble)
        // Lowest priority.
    };

    private final TrcPriorityIndicator indicator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param indicatorName specifies the indicator hardware name.
     * @param gamepad specifies the gamepad to use for the rumble.
     */
    public RumbleIndicator(String indicatorName, FtcGamepad gamepad)
    {
        indicator = new FtcGamepadRumble(indicatorName, gamepad.gamepad);
        indicator.setPatternPriorities(rumblePatternPriorities);
    }   //RumbleIndicator

    /**
     * This method sets the rumble pattern ON for a period of time and turns off automatically afterwards.
     *
     * @param patternName specifies the name of the rumble pattern to turn on.
     */
    public void setRumblePattern(String patternName)
    {
        if (indicator != null)
        {
            indicator.setPatternState(patternName, true);
        }
    }   //setRumblePattern

}   //class RumbleIndicator
