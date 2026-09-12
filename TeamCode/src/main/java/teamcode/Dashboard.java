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

import teamcode.subsystems.DriveBase;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
import trclib.driverio.TrcGameController;
import trclib.vision.TrcOpenCvColorBlobPipeline;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Dashboard
{
    @Config
    public static class DashboardParams
    {
        public static boolean updateDashboardEnabled = RobotParams.Preferences.updateDashboard;
        public static FtcAuto.Alliance alliance = FtcAuto.Alliance.Blue;
        public static FtcAuto.AutoChoices autoChoices = FtcAuto.autoChoices;
    }   //class DashboardParams

    @Config
    public static class SubsystemDrivebase
    {
        public static TrcDriveBase.BaseParams driveBaseParams = DriveBase.MecanumRobotInfo.baseParams;
        public static TrcGameController.DriveMode driveMode = TrcGameController.DriveMode.Arcade;
        public static TrcDriveBase.DriveOrientation driveOrientation  = TrcDriveBase.DriveOrientation.Robot;
        public static double driveSlowScale = 0.3;
        public static double driveNormalScale = 1.0;
        public static double turnSlowScale = 0.3;
        public static double turnNormalScale = 0.5;
    }   //class SubsystemDrivebase

    @Config
    public static class SubsystemVision
    {
        public static TrcOpenCvColorBlobPipeline.PipelineParams colorBlobVision = Vision.colorBlobPipelineParams;
    }   //class SubsystemVision

    @Config
    public static class TuneSubsystem
    {
        public static String subsystemName = "";
        public static TrcPidController.PidCoefficients pidCoeffs = null;
        public static double pidTolerance = 0.0;
        public static boolean useSoftwarePid = false;
        public static double gravityPower = 0.0;
        public static double input = 0.0;
        public static double target = 0.0;
    }   //class TuneSubsystem

    @Config
    public static class TuneShootTable
    {
        public static double targetDistance = 0.0;
        public static double shootMotor1Velocity = 2000.0;  // in RPM
        public static double panAngle = 0.0;                // in degrees
        public static double tiltAngle = 26.0;              // in degrees
    }   //class TuneShootTable
}   //class Dashboard
