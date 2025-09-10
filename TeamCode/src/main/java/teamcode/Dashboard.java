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

package teamcode;

import com.acmerobotics.dashboard.config.Config;

import trclib.controller.TrcPidController;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Dashboard
{
    private static Double nextDashboardUpdateTime =  null;

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     *
     * @param robot specifies the robot object.
     * @param lineNum specifies the first Dashboard line for printing status.
     * @return next available dashboard line.
     */
    public static int updateDashboard(Robot robot, int lineNum)
    {
        double currTime = TrcTimer.getCurrentTime();
        boolean slowLoop = nextDashboardUpdateTime == null || currTime >= nextDashboardUpdateTime;

        if (slowLoop)
        {
            nextDashboardUpdateTime = currTime + RobotParams.Robot.DASHBOARD_UPDATE_INTERVAL;
        }

        if (RobotParams.Preferences.showDriveBase)
        {
            lineNum = robot.robotBase.updateStatus(lineNum, slowLoop);
        }

        if (RobotParams.Preferences.showVision && robot.vision != null)
        {
            lineNum = robot.vision.updateStatus(lineNum, slowLoop);
        }

        if (RobotParams.Preferences.showSubsystems)
        {
            lineNum = TrcSubsystem.updateStatusAll(lineNum, slowLoop);
        }

        return lineNum;
    }   //updateDashboard

    @Config
    public static class Vision
    {
        public static double[] colorThresholds = new double[6];
        public static TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams();
    }   //class Vision

    @Config
    public static class Drive
    {
        public static double xTarget = 0.0;
        public static double yTarget = 0.0;
        public static double turnTarget = 0.0;
        public static double drivePower = 1.0;
        public static double turnPower = 1.0;
        public static double driveTime = 0.0;
        public static TrcPidController.PidCoefficients xPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);
        public static TrcPidController.PidCoefficients yPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);
        public static TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);
        public static double maxVelocity = 0.0;
        public static double maxAcceleration = 0.0;
        public static double maxDeceleration = 0.0;
    }   //class Drive

    @Config
    public static class Subsystem
    {
        public static String subsystemName = "";
        // 7 doubles: Kp, Ki, Kd, Kf, iZone, PIDTolerance, GravityCompPower
        public static double[] tuneParams = new double[7];
    }   //class Subsystem

}   //class Dashboard
