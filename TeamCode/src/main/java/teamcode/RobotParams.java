/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.os.Environment;

import teamcode.subsystems.RobotBase;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.pathdrive.TrcPose2D;

/**
 * This class contains robot constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotBase.RobotType robotType       = RobotBase.RobotType.MecanumRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useLoopPerformanceMonitor   = true;
        public static final boolean useBatteryMonitor           = false;
        // Driver feedback
        // Status Update: Dashboard Update may affect robot loop time, don't do it when in competition.
        public static final boolean updateDashboard             = !inCompetition;
        public static final boolean showDriveBase               = false;
        public static final boolean showPidDrive                = false;
        public static final boolean showVision                  = false;
        public static final boolean showSubsystems              = true;
        public static final boolean useRumble                   = false;
        // Vision
        public static final boolean useVision                   = false;
        public static final boolean useWebCam                   = false;    // false to use Android phone camera.
        public static final boolean useBuiltinCamBack           = false;    // For Android Phone as Robot Controller.
        public static final boolean useLimelightVision          = false;
        public static final boolean useCameraStreamProcessor    = false;
        public static final boolean useWebcamAprilTagVision     = false;
        public static final boolean useColorBlobVision          = false;
        public static final boolean useSolvePnp                 = false;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = false;
        // Drive Base
        public static final boolean useDriveBase                = false;
        public static final boolean usePinpointOdometry         = false;
        public static final boolean useSparkfunOTOS             = false;
        // Subsystems
        public static final boolean useSubsystems               = false;
        // Tuning
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean tuneDriveBase               = false;
    }   //class Preferences

    /**
     * This class contains miscellaneous robot info.
     */
    public static class Robot
    {
//        public static final String TEAM_FOLDER_PATH             =
//            AppUtil.getDefContext().getExternalFilesDir(null).getPath() + "/FIRST/ftcTeam";
        public static final String TEAM_FOLDER_PATH             =
            Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftcTeam";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final double DASHBOARD_UPDATE_INTERVAL    = 0.2;      // in msec
        public static final String ROBOT_CODEBASE               = "Robot2025";
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;
        public static final double ROBOT_LENGTH                 = 18.0;
        public static final double ROBOT_WIDTH                  = 18.0;
    }   //class Robot

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        public static final boolean fieldIsMirrored             = false;
        // AprilTag locations.
        public static final TrcPose2D[] APRILTAG_POSES          = new TrcPose2D[] {
            new TrcPose2D(0.0, 0.0, 0.0),   // TagId 1
            new TrcPose2D(0.0, 0.0, 0.0),   // TagId 2
            new TrcPose2D(0.0, 0.0, 0.0),   // TagId 3
            new TrcPose2D(0.0, 0.0, 0.0)    // TagId 4
        };
        // Robot start locations.
        // Game elapsed times.
        public static final double AUTO_PERIOD                      = 30.0;     // 30 seconds auto period
        public static final double TELEOP_PERIOD                    = 120.0;    // 2 minutes teleop period
        public static final double PARKING_TIME                     = 10.0;
        public static final double ENDGAME_DEADLINE                 = TELEOP_PERIOD - PARKING_TIME;
    }   //class Game

    /**
     * This class contains field dimension constants. Generally, these should not be changed.
     */
    public static class Field
    {
        public static final double FULL_FIELD_INCHES            = 141.24;
        public static final double HALF_FIELD_INCHES            = FULL_FIELD_INCHES/2.0;
        public static final double FULL_TILE_INCHES             = FULL_FIELD_INCHES/6.0;
    }   //class Field

    /**
     * This class contains Gobilda motor parameters.
     */
    public static class Gobilda
    {
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-71-2-1-ratio-24mm-length-8mm-rex-shaft-84-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_84_ENC_PPR        =
            (((1.0 + 46.0/17.0) * (1.0 + 46.0/17.0) * (1.0 + 46.0/11.0)) * 28.0);
        public static final double MOTOR_5203_84_MAX_RPM        = 84.0;
        public static final double MOTOR_5203_84_MAX_VEL_PPS    =
            MOTOR_5203_84_ENC_PPR * MOTOR_5203_84_MAX_RPM / 60.0;     // 2789.661 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_312_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
        public static final double MOTOR_5203_312_MAX_RPM       = 312.0;
        public static final double MOTOR_5203_312_MAX_VEL_PPS   =
            MOTOR_5203_312_ENC_PPR * MOTOR_5203_312_MAX_RPM / 60.0;     // 2795.9872 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_435_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
        public static final double MOTOR_5203_435_MAX_RPM       = 435.0;
        public static final double MOTOR_5203_435_MAX_VEL_PPS   =
            MOTOR_5203_435_ENC_PPR * MOTOR_5203_435_MAX_RPM / 60.0;     // 2787.9135 pps
    }   //class Gobilda

}   //class RobotParams
