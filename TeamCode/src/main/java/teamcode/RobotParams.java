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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;

import org.openftc.easyopencv.OpenCvCameraRotation;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.drivebase.FtcSwerveDrive;
import ftclib.motor.FtcMotorActuator.MotorType;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcGameController;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcPidController.PidCoefficients;
import trclib.vision.TrcHomographyMapper;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains Gobilda motor parameters.
     */
    public static class Gobilda
    {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
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
     * This class contains season specific game element information.
     */
    public static class Game
    {
        public static final TrcPose2D[] APRILTAG_POSES          = new TrcPose2D[] {
            new TrcPose2D(0.0, 0.0, 0.0),   // TagId 1
            new TrcPose2D(0.0, 0.0, 0.0),   // TagId 2
            new TrcPose2D(0.0, 0.0, 0.0),   // TagId 3
            new TrcPose2D(0.0, 0.0, 0.0)    // TagId 4
        };
    }   //class Game

    /**
     * This class contains miscellaneous robot info.
     */
    public static class System
    {
        public static final String TEAM_FOLDER_PATH             =
            Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftcTeam";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String DEF_ROBOT_NAME               = "Robot3543";
    }   //class System

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // This is useful for developing Vision code where all you need is a Control Hub and camera.
        VisionOnly,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Swerve Drive Base Robot
        SwerveRobot
    }   //enum RobotType

    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.MecanumRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useLoopPerformanceMonitor   = true;
        public static final boolean useBatteryMonitor           = false;
        // Status Update: Status Update may affect robot loop time, don't do it when in competition.
        public static final boolean doStatusUpdate              = !inCompetition;
        // Vision
        public static final boolean useVision                   = false;
        public static final boolean useWebCam                   = false;
        public static final boolean useBuiltinCamBack           = false;    // For Android Phone as Robot Controller.
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean useAprilTagVision           = false;
        public static final boolean useColorBlobVision          = false;
        public static final boolean useTensorFlowVision         = false;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = false;
        public static final boolean showSubsystems              = true;
        // Drive Base
        public static final boolean useDriveBase                = false;
        public static final boolean useExternalOdometry         = false;
        // Subsystems
        public static final boolean useSubsystems               = false;
    }   //class Preferences

    //
    // Robot Parameters.
    //

    public static class VisionOnlyParams extends FtcRobotDrive.RobotInfo
    {
        public VisionOnlyParams()
        {
            robotName = "VisionOnly";
            // Vision
            webCam1Name = "Webcam 1";
            webCam2Name = "Webcam 2";
            camImageWidth = 640;
            camImageHeight = 480;
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Camera location On Robot
            camXOffset = 0.0;       //Camera right offset from robot centroid
            camYOffset = 2.0;       //Camera forward offset from robot centroid
            camZOffset = 9.75;      //Camera height offset from floor
            camTiltDown = 15.00;    //Camera tilt down angle from horizontal in deg
            camPose = new TrcPose2D(camXOffset, camYOffset, 0.0);
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                     // Camera Top Left
                camImageWidth -1, 120.0,                        // Camera Top Right
                0.0, camImageHeight - 1,                        // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);         // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - robotLength/2.0 - camYOffset,  // World Top Left
                11.4375, 44.75 - robotLength/2.0 - camYOffset,  // World Top Right
                -2.5625, 21.0 - robotLength/2.0 - camYOffset,   // World Bottom Left
                2.5626, 21.0 - robotLength/2.0 - camYOffset);   // World Bottom Right
        }   //VisionOnlyParams

    }   //class VisionOnlyParams

    /**
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public DifferentialParams()
        {
            robotName = "DifferentialRobot";
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.UP;
            hubUsbDirection = UsbFacingDirection.FORWARD;
            // Robot Dimensions
            robotLength = 17.0;
            robotWidth = 17.0;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            // Odometry Wheels
            odWheelScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;    // 0.00105687652708656383937269814237 in/count
            xOdWheelOffsetX = 0.0;
            xOdWheelOffsetY = -168.0 * TrcUtil.INCHES_PER_MM;
            yLeftOdWheelOffsetX = -144.0 * TrcUtil.INCHES_PER_MM;
            yLeftOdWheelOffsetY = -12.0 * TrcUtil.INCHES_PER_MM;
            yRightOdWheelOffsetX = 144.0 * TrcUtil.INCHES_PER_MM;
            yRightOdWheelOffsetY = -12.0 * TrcUtil.INCHES_PER_MM;
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Robot Drive
            driveMode = TrcGameController.DriveMode.ArcadeMode;
            driveOrientation = DriveOrientation.ROBOT;
            driveSlowScale = 0.3;
            driveNormalScale = 1.0;
            turnSlowScale = 0.3;
            turnNormalScale = 0.6;
            // Vision
            webCam1Name = "Webcam 1";
            webCam2Name = "Webcam 2";
            camImageWidth = 640;
            camImageHeight = 480;
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Camera location On Robot
            camXOffset = 0.0;       //Camera right offset from robot centroid
            camYOffset = 2.0;       //Camera forward offset from robot centroid
            camZOffset = 9.75;      //Camera height offset from floor
            camTiltDown = 15.00;    //Camera tilt down angle from horizontal in deg
            camPose = new TrcPose2D(camXOffset, camYOffset, 0.0);
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                     // Camera Top Left
                camImageWidth -1, 120.0,                        // Camera Top Right
                0.0, camImageHeight - 1,                        // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);         // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - robotLength/2.0 - camYOffset,  // World Top Left
                11.4375, 44.75 - robotLength/2.0 - camYOffset,  // World Top Right
                -2.5625, 21.0 - robotLength/2.0 - camYOffset,   // World Bottom Left
                2.5626, 21.0 - robotLength/2.0 - camYOffset);   // World Bottom Right
            blinkinName = "blinkin";
        }   //DifferentialParams

    }   //class DifferentialParams

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public MecanumParams()
        {
            robotName = "MecanumRobot";
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.UP;
            hubUsbDirection = UsbFacingDirection.FORWARD;
            // Robot Dimensions
            robotLength = 17.0;
            robotWidth = 17.0;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            // Odometry Wheels
            odWheelScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;    // 0.00105687652708656383937269814237 in/count
            xOdWheelOffsetX = 0.0;
            xOdWheelOffsetY = -168.0 * TrcUtil.INCHES_PER_MM;
            yLeftOdWheelOffsetX = -144.0 * TrcUtil.INCHES_PER_MM;
            yLeftOdWheelOffsetY = -12.0 * TrcUtil.INCHES_PER_MM;
            yRightOdWheelOffsetX = 144.0 * TrcUtil.INCHES_PER_MM;
            yRightOdWheelOffsetY = -12.0 * TrcUtil.INCHES_PER_MM;
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Robot Drive
            driveMode = TrcGameController.DriveMode.ArcadeMode;
            driveOrientation = DriveOrientation.ROBOT;
            driveSlowScale = 0.3;
            driveNormalScale = 1.0;
            turnSlowScale = 0.3;
            turnNormalScale = 0.6;
            // Vision
            webCam1Name = "Webcam 1";
            webCam2Name = "Webcam 2";
            camImageWidth = 640;
            camImageHeight = 480;
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Camera location On Robot
            camXOffset = 0.0;       //Camera right offset from robot centroid
            camYOffset = 2.0;       //Camera forward offset from robot centroid
            camZOffset = 9.75;      //Camera height offset from floor
            camTiltDown = 15.00;    //Camera tilt down angle from horizontal in deg
            camPose = new TrcPose2D(camXOffset, camYOffset, 0.0);
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                     // Camera Top Left
                camImageWidth -1, 120.0,                        // Camera Top Right
                0.0, camImageHeight - 1,                        // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);         // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - robotLength/2.0 - camYOffset,  // World Top Left
                11.4375, 44.75 - robotLength/2.0 - camYOffset,  // World Top Right
                -2.5625, 21.0 - robotLength/2.0 - camYOffset,   // World Bottom Left
                2.5626, 21.0 - robotLength/2.0 - camYOffset);   // World Bottom Right
            blinkinName = "blinkin";
        }   //MecanumParams

    }   //class MecanumParams

    /**
     * This class contains the Swerve Drive Base Parameters.
     */
    public static class SwerveParams extends FtcSwerveDrive.SwerveInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public SwerveParams()
        {
            robotName = "SwerveRobot";
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.UP;
            hubUsbDirection = UsbFacingDirection.FORWARD;
            // Robot Dimensions
            robotLength = 17.0;
            robotWidth = 17.0;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            // Odometry Wheels
            odWheelScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;    // 0.00105687652708656383937269814237 in/count
            xOdWheelOffsetX = 0.0;
            xOdWheelOffsetY = -168.0 * TrcUtil.INCHES_PER_MM;
            yLeftOdWheelOffsetX = -144.0 * TrcUtil.INCHES_PER_MM;
            yLeftOdWheelOffsetY = -12.0 * TrcUtil.INCHES_PER_MM;
            yRightOdWheelOffsetX = 144.0 * TrcUtil.INCHES_PER_MM;
            yRightOdWheelOffsetY = -12.0 * TrcUtil.INCHES_PER_MM;
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = yDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = yDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Robot Drive
            driveMode = TrcGameController.DriveMode.ArcadeMode;
            driveOrientation = DriveOrientation.ROBOT;
            driveSlowScale = 0.3;
            driveNormalScale = 1.0;
            turnSlowScale = 0.3;
            turnNormalScale = 0.6;
            // Vision
            webCam1Name = "Webcam 1";
            webCam2Name = "Webcam 2";
            camImageWidth = 640;
            camImageHeight = 480;
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Camera location On Robot
            camXOffset = 0.0;       //Camera right offset from robot centroid
            camYOffset = 2.0;       //Camera forward offset from robot centroid
            camZOffset = 9.75;      //Camera height offset from floor
            camTiltDown = 15.00;    //Camera tilt down angle from horizontal in deg
            camPose = new TrcPose2D(camXOffset, camYOffset, 0.0);
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                     // Camera Top Left
                camImageWidth -1, 120.0,                        // Camera Top Right
                0.0, camImageHeight - 1,                        // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);         // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - robotLength/2.0 - camYOffset,  // World Top Left
                11.4375, 44.75 - robotLength/2.0 - camYOffset,  // World Top Right
                -2.5625, 21.0 - robotLength/2.0 - camYOffset,   // World Bottom Left
                2.5626, 21.0 - robotLength/2.0 - camYOffset);   // World Bottom Right
            blinkinName = "blinkin";
            // Steer Encoders
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderInverted = new boolean[] {};
            steerEncoderZeros = new double[] {0.474812, 0.467663, 0.541338, 0.545340};
            steerZerosFilePath = System.TEAM_FOLDER_PATH + "/SteerCalibration.txt";
            // Steer Motors
            steerMotorType = MotorType.CRServo;
            steerMotorNames = new String[] {"lfSteerServo", "rfSteerServo", "lbSteerServo", "rbSteerServo"};
            steerMotorInverted = new boolean[] {true, true, true, true};
            steerMotorPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);
            steerMotorPidTolerance = 1.0;
            // Swerve Modules
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
        }   //SwerveParams

    }   //class SwerveParams

    //
    // Subsystems.
    //

}   //class RobotParams
