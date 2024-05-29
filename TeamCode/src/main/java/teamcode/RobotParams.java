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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcDriveBase.DriveOrientation;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcGamepad;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    public static enum RobotType
    {
        MecanumRobot,
        SwerveRobot,
        NoRobot
    }   //RobotType

    /**
     * This class contains robot preferences. It controls enabling/disabling of various robot features.
     */
    public static class Preferences
    {
        // Global config
        public static RobotType robotType = RobotType.MecanumRobot;
        public static boolean inCompetition = false;

        // Miscellaneous
        public static boolean useTraceLog = true;
        public static boolean useLoopPerformanceMonitor = true;
        public static boolean useBlinkin = false;
        public static boolean useBatteryMonitor = false;
        public static boolean doStatusUpdate = !inCompetition;
        // Vision
        public static boolean useWebCam = true;
        public static boolean hasWebCam2 = false;
        public static boolean useBuiltinCamBack = false;
        public static boolean tuneColorBlobVision = false;
        public static boolean useAprilTagVision = false;
        public static boolean useColorBlobVision = false;
        public static boolean useTensorFlowVision = false;
        public static boolean showVisionView = !inCompetition;
        public static boolean showVisionStat = false;
        // Drive Base
        public static boolean useExternalOdometry = false;
        public static boolean doSwervePhysicalAlignment = false;
        public static boolean swerveDualServoSteering = true;
        // Subsystems
        public static boolean useSubsystems = false;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "Robotxxxx";
    public static final RevHubOrientationOnRobot.LogoFacingDirection hubLogoDirection =
        RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection hubUsbDirection =
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static final String TEAM_FOLDER_PATH                 =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftcxxxx";
    public static final String LOG_FOLDER_PATH                  = TEAM_FOLDER_PATH + "/tracelogs";
    public static final String STEERING_CALIBRATION_DATA_FILE   = "SteerCalibration.txt";
    //
    // Hardware names.
    //
    // Miscellaneous.
    public static final String HWNAME_IMU                       = "imu";
    public static final String HWNAME_WEBCAM1                   = "Webcam 1";
    public static final String HWNAME_WEBCAM2                   = "Webcam 2";
    public static final String HWNAME_BLINKIN                   = "blinkin";
    // Drive Base.
    public static final String HWNAME_LFDRIVE_MOTOR             = "lfDriveMotor";
    public static final String HWNAME_RFDRIVE_MOTOR             = "rfDriveMotor";
    public static final String HWNAME_LBDRIVE_MOTOR             = "lbDriveMotor";
    public static final String HWNAME_RBDRIVE_MOTOR             = "rbDriveMotor";
    public static final String HWNAME_LFSTEER_SERVO             = "lfSteerServo";
    public static final String HWNAME_RFSTEER_SERVO             = "rfSteerServo";
    public static final String HWNAME_LBSTEER_SERVO             = "lbSteerServo";
    public static final String HWNAME_RBSTEER_SERVO             = "rbSteerServo";
    public static final String HWNAME_LFSTEER_ENCODER           = "lfSteerEncoder";
    public static final String HWNAME_RFSTEER_ENCODER           = "rfSteerEncoder";
    public static final String HWNAME_LBSTEER_ENCODER           = "lbSteerEncoder";
    public static final String HWNAME_RBSTEER_ENCODER           = "rbSteerEncoder";
    // Subsystems.

    //
    // Field dimensions.
    //
    public static final double FULL_FIELD_INCHES                = 141.24;
    public static final double HALF_FIELD_INCHES                = FULL_FIELD_INCHES/2.0;
    public static final double FULL_TILE_INCHES                 = FULL_FIELD_INCHES/6.0;
    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                     = 17.0;
    public static final double ROBOT_WIDTH                      = 17.0;
    public static final double DRIVE_BASE_LENGTH                = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
    public static final double DRIVE_BASE_WIDTH                 = 16.0;
    //
    // Game related locations.
    //
    public static final TrcPose2D[] APRILTAG_POSES              = new TrcPose2D[] {
        new TrcPose2D(0.0, 0.0, 0.0),   // TagId 1
        new TrcPose2D(0.0, 0.0, 0.0),   // TagId 2
        new TrcPose2D(0.0, 0.0, 0.0),   // TagId 3
        new TrcPose2D(0.0, 0.0, 0.0)    // TagId 4
    };
    //
    // Vision subsystem.
    //
    public static final int CAM_IMAGE_WIDTH                     = 640;
    public static final int CAM_IMAGE_HEIGHT                    = 480;
    public static final OpenCvCameraRotation CAM_ORIENTATION    = OpenCvCameraRotation.UPRIGHT;
    // Camera location on robot.
    public static final double CAM_X_OFFSET                     = 0.0;  //Camera right offset from robot centroid
    public static final double CAM_Y_OFFSET                     = 2.0;  //Camera forward offset from robot centroid
    public static final double CAM_Z_OFFSET                     = 9.75; //Camera height offset from floor
    public static final double CAM_TILT_DOWN                    = 15.00;//Camera tilt down angle from horizontal in deg
    public static final TrcPose2D CAM_POSE                      = new TrcPose2D(CAM_X_OFFSET, CAM_Y_OFFSET, 0.0);
//    // Camera: Micorosoft Lifecam HD 3000 v1/v2
//    public static final double WEBCAM_FX                        = 678.154;  // in pixels
//    public static final double WEBCAM_FY                        = 678.170;  // in pixels
//    public static final double WEBCAM_CX                        = 318.135;  // in pixels
//    public static final double WEBCAM_CY                        = 228.374;  // in pixels
//    // Camera: Logitech C270
//    public static final double WEBCAM_FX                        = 822.317;  // in pixels
//    public static final double WEBCAM_FY                        = 822.317;  // in pixels
//    public static final double WEBCAM_CX                        = 319.495;  // in pixels
//    public static final double WEBCAM_CY                        = 242.502;  // in pixels
//    // Camera: Logitech C310
//    public static final double WEBCAM_FX                        = 821.993;  // in pixels
//    public static final double WEBCAM_FY                        = 821.993;  // in pixels
//    public static final double WEBCAM_CX                        = 330.489;  // in pixels
//    public static final double WEBCAM_CY                        = 248.997;  // in pixels
//    // Camera: Logitech C920
//    public static final double WEBCAM_FX                        = 622.001;  // in pixels
//    public static final double WEBCAM_FY                        = 622.001;  // in pixels
//    public static final double WEBCAM_CX                        = 319.803;  // in pixels
//    public static final double WEBCAM_CY                        = 241.251;  // in pixels

    // Measurement unit: pixels
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y      = 120.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X     = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y     = 120.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X   = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y   = CAM_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X  = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y  = CAM_IMAGE_HEIGHT - 1;
    // Measurement unit: inches
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X       = -12.5625;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y       = 48.0 - ROBOT_LENGTH/2.0 - CAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X      = 11.4375;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y      = 44.75 - ROBOT_LENGTH/2.0 - CAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X    = -2.5625;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y    = 21.0 - ROBOT_LENGTH/2.0 - CAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X   = 2.5626;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y   = 21.0 - ROBOT_LENGTH/2.0 + CAM_Y_OFFSET;

    public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);
    //
    // Motor Odometries.
    //
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_312_ENCODER_PPR     = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
    public static final double GOBILDA_5203_312_RPM             = 312.0;
    public static final double GOBILDA_5203_312_MAX_VELOCITY_PPS=
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.9872 pps
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_435_ENCODER_PPR     = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
    public static final double GOBILDA_5203_435_RPM             = 435.0;
    public static final double GOBILDA_5203_435_MAX_VELOCITY_PPS=
        GOBILDA_5203_435_ENCODER_PPR*GOBILDA_5203_435_RPM/60.0; // 2787.9135 pps
    //
    // DriveBase subsystem.
    //
    public static DriveOrientation DEF_DRIVE_ORIENTATION        = DriveOrientation.FIELD;
    public static final double LFSTEER_ZERO_POS                 = 0.474812;
    public static final double RFSTEER_ZERO_POS                 = 0.467663;
    public static final double LBSTEER_ZERO_POS                 = 0.541338;
    public static final double RBSTEER_ZERO_POS                 = 0.545340;

    public static final boolean LFDRIVE_INVERTED                = true;
    public static final boolean RFDRIVE_INVERTED                = false;
    public static final boolean LBDRIVE_INVERTED                = true;
    public static final boolean RBDRIVE_INVERTED                = false;
    public static final boolean LFSTEER_INVERTED                = true;
    public static final boolean RFSTEER_INVERTED                = true;
    public static final boolean LBSTEER_INVERTED                = true;
    public static final boolean RBSTEER_INVERTED                = true;
    public static final double STEER_SERVO_KP                   = 0.01;
    public static final double STEER_SERVO_KI                   = 0.0;
    public static final double STEER_SERVO_KD                   = 0.0;
    public static final double STEER_SERVO_KF                   = 0.0;
    public static final double STEER_SERVO_IZONE                = 0.0;
    public static final double STEER_SERVO_TOLERANCE            = 0.5;

    public static final boolean DRIVE_WHEEL_BRAKE_MODE_ON       = true;
    public static final double TURN_POWER_LIMIT                 = 0.5;
    public static final double DRIVE_POWER_SCALE_SLOW           = 0.5;
    public static final double DRIVE_POWER_SCALE_NORMAL         = 1.0;
    public static final double TURN_POWER_SCALE_SLOW            = 0.5;
    public static final double TURN_POWER_SCALE_NORMAL          = 1.0;
    // Optii Odometry Wheel:
    public static final double ODWHEEL_DIAMETER                 = 35.0 * TrcUtil.INCHES_PER_MM;
    public static final double ODWHEEL_CPR                      = 4096.0;
    public static final double ODWHEEL_INCHES_PER_COUNT         = Math.PI*ODWHEEL_DIAMETER/ODWHEEL_CPR;
    // Scale = 0.00105687652708656383937269814237 inches/count
    public static final double YLEFT_ODWHEEL_X_OFFSET           = -144.0 * TrcUtil.INCHES_PER_MM;
    public static final double YLEFT_ODWHEEL_Y_OFFSET           = -12.0 * TrcUtil.INCHES_PER_MM;
    public static final double YRIGHT_ODWHEEL_X_OFFSET           = 144.0 * TrcUtil.INCHES_PER_MM;
    public static final double YRIGHT_ODWHEEL_Y_OFFSET          = -12.0 * TrcUtil.INCHES_PER_MM;
    public static final double X_ODWHEEL_X_OFFSET               = 0.0;
    public static final double X_ODWHEEL_Y_OFFSET               = -168.0 * TrcUtil.INCHES_PER_MM;
    public static final FtcGamepad.DriveMode ROBOT_DRIVE_MODE   = FtcGamepad.DriveMode.ARCADE_MODE;
    //
    // Velocity controlled constants.
    //
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS     = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    public static final TrcPidController.PidCoefficients xPosPidCoeff =
        new TrcPidController.PidCoefficients(0.095, 0.0, 0.001, 0.0);
    public static final double XPOS_TOLERANCE                   = 1.0;
    public static final double XPOS_INCHES_PER_COUNT            = 0.01924724265461924299065420560748;
    public static final Double X_RAMP_RATE                      = null;//10.0;

    public static final TrcPidController.PidCoefficients yPosPidCoeff =
        new TrcPidController.PidCoefficients(0.06, 0.0, 0.002, 0.0);
    public static final double YPOS_TOLERANCE                   = 1.0;
    public static final double YPOS_INCHES_PER_COUNT            = 0.02166184604662450653409090909091;
    public static final Double Y_RAMP_RATE                      = null;//10.0;

    public static final TrcPidController.PidCoefficients turnPidCoeff =
        new TrcPidController.PidCoefficients(0.02, 0.08, 0.003, 0.0, 30.0);
    public static final double TURN_TOLERANCE                   = 1.0;
    public static final Double TURN_RAMP_RATE                   = null;//10.0;
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    public static final double ROBOT_MAX_VELOCITY               = 23.0;     // measured maximum from drive speed test.
    public static final double ROBOT_MAX_ACCELERATION           = 500.0;    // measured maximum from drive speed test.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final TrcPidController.PidCoefficients velPidCoeff  =
        new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/ROBOT_MAX_VELOCITY);
    public static final double PPD_FOLLOWING_DISTANCE           = 6.0;
    public static final double PPD_POS_TOLERANCE                = 1.0;
    public static final double PPD_POS_ERR_RATE_THRESHOLD       = 1.0;
    public static final double PPD_TURN_TOLERANCE               = 1.0;
    public static final double PPD_TURN_ERR_RATE_THRESHOLD      = 1.0;

}   //class RobotParams
