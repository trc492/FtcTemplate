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
        public static String tuneSubsystemName = "";
        public static FtcAuto.Alliance alliance = FtcAuto.Alliance.BLUE_ALLIANCE;
        public static FtcAuto.AutoChoices autoChoices = FtcAuto.autoChoices;
    }   //class DashboardParams

    @Config
    public static class Subsystem_Drivebase
    {
        public static TrcDriveBase.BaseParams driveBaseParams = DriveBase.MecanumRobotInfo.baseParams;
        public static TrcGameController.DriveMode driveMode = TrcGameController.DriveMode.ArcadeMode;
        public static TrcDriveBase.DriveOrientation driveOrientation  = TrcDriveBase.DriveOrientation.ROBOT;
        public static double driveSlowScale = 0.3;
        public static double driveNormalScale = 1.0;
        public static double turnSlowScale = 0.3;
        public static double turnNormalScale = 0.5;
    }   //class Subsystem_Drivebase

    @Config
    public static class Subsystem_Vision
    {
        public static TrcOpenCvColorBlobPipeline.PipelineParams colorBlobVision = Vision.colorBlobPipelineParams;
    }   //class Subsystem_Vision

}   //class Dashboard
