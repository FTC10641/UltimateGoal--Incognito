//package org.firstinspires.ftc.teamcode.roadRunner.realsense;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.acmerobotics.roadrunner.drive.DriveSignal;
//import com.acmerobotics.roadrunner.drive.MecanumDrive;
//import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
//import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.profile.MotionProfile;
//import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
//import com.acmerobotics.roadrunner.profile.MotionState;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.ReadWriteFile;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.teamcode.roadRunner.util.AnalogGyroSensor;
//import org.firstinspires.ftc.teamcode.roadRunner.util.command.PacketAction;
//import org.firstinspires.ftc.teamcode.roadRunner.realsense.T265LocalizerRR;
//import org.firstinspires.ftc.teamcode.roadRunner.util.FieldConstants;
//import org.firstinspires.ftc.teamcode.roadRunner.util.pipeline.openCvPipeLines;
//import org.firstinspires.ftc.teamcode.roadRunner.util.roadrunner.DashboardUtil;
//import org.firstinspires.ftc.teamcode.roadRunner.util.roadrunner.LynxModuleUtil;
//import org.firstinspires.ftc.teamcode.roadRunner.util.homar.Button;
//import org.firstinspires.ftc.teamcode.roadRunner.util.ThrowerUtil;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.io.File;
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.*;
//
///**
// * FTC 6547's Robot for the 2020-21 FTC Ultimate Goal Season
// */
//@Config
//public class DriveTrainRealsense extends MecanumDrive {
//
//    public static boolean USE_REALSENSE = true;
//    public static PIDCoefficients X_PID = new PIDCoefficients(3, 0, 0);
//    public static PIDCoefficients Y_PID = new PIDCoefficients(4, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0, 0);
//
//    public static boolean INTERRUPT_TRAJECTORIES_WITH_GAMEPAD = false;
//
//    public DriveTrainRealsense(double kV, double kA, double kStatic, double trackWidth, double wheelBase, double lateralMultiplier) {
//        super(kV, kA, kStatic, trackWidth, wheelBase, lateralMultiplier);
//    }
//
//    public enum Mode {
//        IDLE,
//        TURN,
//        FOLLOW_TRAJECTORY
//    }
//
//    //don't touch this unless you know what it does
//    /**
//     * Used when add extra data to the packet that is created when the update() method is called.
//     * Do not use if you don't know what it does
//     */
//    private PacketAction packetAction = (packet, fieldOverlay) -> {
//        //do nothing
//    };
//
//    /**
//     * The file name used when saving the robot's position
//     */
//    public static String POS_FILE_NAME = "pos.txt";
//
//    public OpenCvCamera webCam;
//
//    private FtcDashboard dashboard;
//    public NanoClock clock;
//
//    /**
//     * The mode the robot is in when update() is called
//     * IDLE will not autonomous move the bot
//     * FOLLOW_TRAJECTORY follows a trajectory if followTrajectory() or followTrajectorySync() was called
//     * TURN turns the robot if turn(), turnSync(), or any other turn method is called
//     */
//    public Mode mode = Mode.FOLLOW_TRAJECTORY;
//
//    public PIDFController turnController;
//    private MotionProfile turnProfile;
//    private double turnStart;
//
//    public DriveConstraints constraints;
//    private TrajectoryFollower follower;
//
//    private List<Pose2d> poseHistory;
//
//    /**
//     * Drive train motors
//     */
//    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
//    private List<DcMotorEx> motors;
//
//    private BNO055IMU imu;
//    double imuZeroVal = 0;
//
//    /**
//     * Servo that grabs and releases the wobble goal
//     */
//    public Servo wobbleGoalGrabber;
//    /**
//     * Moves the wobble goal grabber up and down
//     * Also known as the Wobvator
//     */
//    public Servo wobbleGoalElevator;
//    /**
//     * Servo that launches the rings
//     */
//    public Servo indexer;
//    /**
//     * Intake motor
//     */
//    public DcMotorEx intake;
//    /**
//     * Both of the robot thrower motors
//     */
//    public DcMotorEx thrower1, thrower2;
//
//    public Servo distanceSensorServoX;
//    public AnalogInput distanceSensorX;
//
//    public Servo distanceSensorServoY;
//    public AnalogInput distanceSensorY;
//
//    public AnalogGyroSensor gyroSensor;
//
//    /**
//     * Robot lights, located on the bottom of the robot
//     */
//    public RevBlinkinLedDriver lights;
//
//    private Pose2d lastPoseOnTurn;
//
//    /**
//     * The opmode the is currently running
//     */
//    private OpMode opMode = null;
//
//    public Button a1,a2, b1,b2,x1,x2,y1,y2,dpadUp1,dpadUp2,dpadDown1, dpadDown2, dpadLeft1, dpadLeft2,dpadRight1, dpadRight2,leftBumper1,leftBumper2,rightBumper1,rightBumper2,start1,start2, rightTrigger1, rightTrigger2, leftTrigger1, leftTrigger2;
//
//    double currentHeadingV=0;
//    double currentHeadingA=0;
//
//    boolean inTaking = false;
//    boolean outTaking = false;
//
//    /**
//     * Time between each time update() is called
//     */
//    private final ElapsedTime timeBetweenUpdates = new ElapsedTime();
//
//    /**
//     * Time between last time a vufoira image was detected
//     */
//    private final ElapsedTime timeBetweenVuforiaTargetDetection = new ElapsedTime(99);
//
//    /**
//     * Robot's thrower target velocity.
//     */
//    double targetVelocity = 0;
//    double leewayVelo = 360;
//
//
//    //Vuforia Stuff
//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//    private static final boolean PHONE_IS_PORTRAIT = false  ;
//
//    @SuppressWarnings("SpellCheckingInspection")
//    private static final String VUFORIA_KEY =
//            "AcA49uX/////AAABmeu99zB3Z0UMtBs+8wKvO+wqH9r+mWnpErlw09BRR+xRyjMJYpow6ZrtHOUAJSedLLrnIoaq2dGAjjHmnCEcqVHnd0YYm7aXeDGnwgAJHsYGU3e7whsv01hBic/gBbuoCqPb7cGk4ZXpkw3FNAYu889wonaHzeIOMQqQrZMtGyQ96E3Lk4/JHtTZDiWfJzdcFDg/LpR+tslv2WKXQlZNKQg581oZZ+GUVW0RbHAXJRcCkHrPgBg1yzuKIqmrwWblPscHtLuFXSJfkuk3C6K8Kp6JvJsTn7JvQGu5Ph9hzaZV4SwN3w4csZMNw5amuZv20f8lkzcLQkN0uT9uTJnwF4uRmc/JHzYlT1DWJJXewPQU";
//
//    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
//    // We will define some constants and conversions here
//    private static final float mmPerInch        = 25.4f;
//    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
//
//    // Constants for perimeter targets
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField  = 36 * mmPerInch;
//
//    // Class Members
//    private OpenGLMatrix lastLocation = null;
//    private VuforiaLocalizer vuforia = null;
//
//    /**
//     * This is the webcam we are to use. As with other hardware devices such as motors and
//     * servos, this device is identified using the robot configuration tool in the FTC application.
//     */
//    WebcamName webcamName = null;
//
//    private boolean targetVisible = false;
//    private float phoneXRotate    = 0;
//    private float phoneYRotate    = 0;
//    private final float phoneZRotate    = 0;
//
//    private VuforiaTrackables targetsUltimateGoal;
//
//    private List<VuforiaTrackable> allTrackables;
//
//    /**
//     * Automatically changed when startVuforia() and stopVuforia() methods are called
//     */
//    private boolean useVuforia = false;
//
//    private Vector2d lastVuforiaPos = new Vector2d();
//
//    //end of Vuforia Stuff
//
//    public FieldConstants.PowerShots[] redPowerShots = new FieldConstants.PowerShots[] {FieldConstants.PowerShots.RED_RIGHT, FieldConstants.PowerShots.RED_CENTER, FieldConstants.PowerShots.RED_LEFT};
//
//    public static double POWER_SHOT_VELO_ADD = 0;
//
//    private boolean messageDisplayed = false;
//
//
//    public DriveTrainRealsense(OpMode opMode) {
//        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
//        USE_REALSENSE = true;
//        this.opMode = opMode;
//        initRobot();
//    }
//    public DriveTrainRealsense(OpMode opMode, boolean resetRealsense) {
//        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
//        //this.USE_REALSENSE = USE_REALSENSE;
//        this.opMode = opMode;
//        initRobot(resetRealsense);
//    }
//
//    /**
//     * Called in constructor, initialises the bot
//     */
//    private void initRobot() {initRobot(true);}
//
//    /**
//     * Called in constructor, initializes the bot
//     * @param resetRealsense determines if the realsense should reset it's position or not
//     */
//    private void initRobot(boolean resetRealsense) {
////        if (!USE_REALSENSE) {
////            RobotLog.setGlobalWarningMessage("NOT USING REALSENSE, USING DEFAULT LOCALIZER", "");
////        }
//
//        opMode.telemetry.log().add("Initializing Robot");
//        HardwareMap hardwareMap=opMode.hardwareMap;
//
//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);
//
//        clock = NanoClock.system();
//
//        mode = Mode.IDLE;
//
//        turnController = new PIDFController(HEADING_PID);
//        turnController.setInputBounds(0, 2 * Math.PI);
//
//        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
//        follower = new HolonomicPIDVAFollower(X_PID, Y_PID, HEADING_PID,
//                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
//        //this is new, if follower is weird, mess with admissible error and timeout
//
//        poseHistory = new ArrayList<>();
//
//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
//
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        // TODO: adjust the names of the following hardware devices to match your configuration
//
//        //don't use IMU, using realsense instead
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);
//        zeroIMU();
//
//        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
//        // upward (normal to the floor) using a command like the following:
//        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
//
//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
//        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//
//        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
//
//        //new code
//        for (DcMotorEx motor : motors) {
//            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            motor.setMotorType(motorConfigurationType);
//        }
//
//        if (RUN_USING_ENCODER) {
//            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
//            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
//        }
//
//        // Reverse motors
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //set the Realsense as the localizer
//        if (USE_REALSENSE) {
//            try {
//                opMode.telemetry.log().add("Initializing Realsense....This may take some time");
//                setLocalizer(new T265LocalizerRR(hardwareMap, resetRealsense));
//            } catch (Exception e) {
//                RobotLog.setGlobalWarningMessage("FAILED TO INIT REALSENSE", "FAILED TO INIT REALSENSE");
//            }
//        }
//        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
//
//        initOtherHardware();
//        initGamepads(); //for tele-op
//    }
//    private void initOtherHardware() {
//
//        wobbleGoalGrabber = opMode.hardwareMap.get(Servo.class, "wob");
//        indexer = opMode.hardwareMap.get(Servo.class, "indexer");
//        wobbleGoalElevator = opMode.hardwareMap.get(Servo.class, "wobvator");
//        distanceSensorX = opMode.hardwareMap.get(AnalogInput.class, "distance0");
//        distanceSensorServoX = opMode.hardwareMap.get(Servo.class, "dServo");
//        distanceSensorServoY = opMode.hardwareMap.get(Servo.class, "dServoY");
//        distanceSensorY = opMode.hardwareMap.get(AnalogInput.class, "distanceY");
//
//        gyroSensor = new AnalogGyroSensor(opMode.hardwareMap.get(AnalogInput.class, "gyro"));
//        gyroSensor.zeroGyro();
//
//        thrower1 = opMode.hardwareMap.get(DcMotorEx.class, "thrower");
//        thrower2 = opMode.hardwareMap.get(DcMotorEx.class, "thrower2");
//        try {
//            intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
//        } catch (Exception e) {
//            RobotLog.setGlobalWarningMsg((RobotCoreException) e, "WILL CONTINUE AS NORMAL");
//        }
//
//        lights = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "lights");
//        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK); //turn off lights
//
//        raiseWobvator();
//        openIndexer();
//        grabWobbleGoal();
//
//        thrower1.setDirection(DcMotorSimple.Direction.REVERSE);
//        thrower2.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        thrower1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        thrower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    /**
//     * Initialize openCV.  May conflict with Vuforia if that is also initialized
//     */
//    public void initOpenCV() {
//        opMode.telemetry.log().add("Initializing OpenCV");
//        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
//
//        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
//        webCam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        webCam.openCameraDevice();//open camera
//        //set pipeline
//        //ringDetectionPipeLine = new openCvPipeLines.RingDetectionPipeLine();
//        webCam.setPipeline(new openCvPipeLines.RingDetectionPipeLine());
//
//        //webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
//    }
//
//    /**
//     * Initialize Vuforia.  May conflict with OpenCV if that is also initialized
//     */
//    public void initVufoira() {
//
//        webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//
//        // We also indicate which camera on the RC we wish to use.
//        parameters.cameraName = webcamName;
//
//        //Make sure extended tracking is disabled for this example.
//        parameters.useExtendedTracking = false;
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Load the data sets for the trackable objects. These particular data
//        // sets are stored in the 'assets' part of our application.
//        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
//        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
//        blueTowerGoalTarget.setName("Blue Tower Goal Target");
//        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
//        redTowerGoalTarget.setName("Red Tower Goal Target");
//        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
//        redAllianceTarget.setName("Red Alliance Target");
//        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
//        blueAllianceTarget.setName("Blue Alliance Target");
//        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
//        frontWallTarget.setName("Front Wall Target");
//
//        // For convenience, gather together all the trackable objects in one easily-iterable collection */
//        allTrackables = new ArrayList<>();
//        allTrackables.addAll(targetsUltimateGoal);
//
//        /*
//         * In order for localization to work, we need to tell the system where each target is on the field, and
//         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
//         * Transformation matrices are a central, important concept in the math here involved in localization.
//         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
//         * for detailed information. Commonly, you'll encounter transformation matrices as instances
//         * of the {@link OpenGLMatrix} class.
//         *
//         * If you are standing in the Red Alliance Station looking towards the center of the field,
//         *     - The X axis runs from your left to the right. (positive from the center to the right)
//         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
//         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
//         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
//         *
//         * Before being transformed, each target image is conceptually located at the origin of the field's
//         *  coordinate system (the center of the field), facing up.
//         */
//
//        //Set the position of the perimeter targets with relation to origin (center of field)
//        redAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        blueAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//        frontWallTarget.setLocation(OpenGLMatrix
//                .translation(-halfField, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
//        blueTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//        redTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        //
//        // Create a transformation matrix describing where the phone is on the robot.
//        //
//        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
//        // Lock it into Portrait for these numbers to work.
//        //
//        // Info:  The coordinate frame for the robot looks the same as the field.
//        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
//        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
//        //
//        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
//        // pointing to the LEFT side of the Robot.
//        // The two examples below assume that the camera is facing forward out the front of the robot.
//
//        // We need to rotate the camera around it's long axis to bring the correct camera forward.
//        if (CAMERA_CHOICE == BACK) {
//            phoneYRotate = -90;
//        } else {
//            phoneYRotate = 90;
//        }
//
//        // Rotate the phone vertical about the X axis if it's in portrait mode
//        if (PHONE_IS_PORTRAIT) {
//            phoneXRotate = 90 ;
//        }
//
//        // Next, translate the camera lens to where it is on the robot.
//        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
//        final float CAMERA_FORWARD_DISPLACEMENT  = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
//        final float CAMERA_VERTICAL_DISPLACEMENT = 10.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
//        final float CAMERA_LEFT_DISPLACEMENT     = -4.0f * mmPerInch;     // eg: Camera is ON the robot's center line
//
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//        //  Let all the trackable listeners know where the phone is
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//        }
//    }
//    /**
//     * Initialize Gamepads using the HOMAR FTC Library
//     * The HOMAR FTC Library enables methods such as onPress() on onRelease()
//     */
//    private void initGamepads() //set the buttons to their values
//    {
//        a1 = new Button();
//        a2 = new Button();
//        b1 = new Button();
//        b2 = new Button();
//        x1 = new Button();
//        x2 = new Button();
//        y1 = new Button();
//        y2 = new Button();
//        dpadDown1 = new Button();
//        dpadDown2 = new Button();
//        dpadLeft1 = new Button();
//        dpadLeft2 = new Button();
//        dpadRight1 = new Button();
//        dpadRight2 = new Button();
//        dpadUp1 = new Button();
//        dpadUp2 = new Button();
//        rightBumper1 = new Button();
//        rightBumper2 = new Button();
//        leftBumper1 = new Button();
//        leftBumper2 = new Button();
//        start1=new Button();
//        start2=new Button();
//        leftTrigger1 = new Button();
//        leftTrigger2 = new Button();
//        rightTrigger1 = new Button();
//        rightTrigger2 = new Button();
//    }
//
//    /**
//     * Must be called at the beginning of each loop if gamepads will be used
//     */
//    public void updateGamepads() //update the gamepad buttons from HOMAR-FTC-Library for tele-op
//    {
//        Gamepad gamepad1 = opMode.gamepad1;
//        Gamepad gamepad2 = opMode.gamepad2;
//        a1.input(gamepad1.a);
//        a2.input(gamepad2.a);
//        b1.input(gamepad1.b);
//        b2.input(gamepad2.b);
//        x1.input(gamepad1.x);
//        x2.input(gamepad2.x);
//        y1.input(gamepad1.y);
//        y2.input(gamepad2.y);
//        dpadUp1.input(gamepad1.dpad_up);
//        dpadUp2.input(gamepad2.dpad_up);
//        dpadRight1.input(gamepad1.dpad_right);
//        dpadRight2.input(gamepad2.dpad_right);
//        dpadDown1.input(gamepad1.dpad_down);
//        dpadDown2.input(gamepad2.dpad_down);
//        dpadLeft1.input(gamepad1.dpad_left);
//        dpadLeft2.input(gamepad2.dpad_left);
//        leftBumper1.input(gamepad1.left_bumper);
//        leftBumper2.input(gamepad2.left_bumper);
//        rightBumper1.input(gamepad1.right_bumper);
//        rightBumper2.input(gamepad2.right_bumper);
//        leftTrigger1.input(gamepad1.left_trigger>=.7);
//        leftTrigger2.input(gamepad2.left_trigger>=.7);
//        rightTrigger1.input(gamepad1.right_trigger>=.7);
//        rightTrigger2.input(gamepad2.right_trigger>=.7);
//    }
//
//    public void startOpenCV() {
//        webCam.openCameraDeviceAsync(() -> webCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN));
//    }
//    public void stopOpenCV() {
//        webCam.stopStreaming();
//    }
//    public openCvPipeLines.RingCount getRingCount() {
//        return openCvPipeLines.RingDetectionPipeLine.getRingCount();
//    }
//    public void startVuforia() {
//        useVuforia = true;
//        targetsUltimateGoal.activate();
//    }
//    public void stopVuforia() {
//        useVuforia = false;
//        targetsUltimateGoal.deactivate();
//    }
//
//    /**
//     * initVuforia() and startVuforia() must be called before this method is called
//     * @return The robot's position according to Vuforia
//     */
//    public Vector2d getPosFromVuforia() {
//        Vector2d vuforiaPos;
//
//        // check all the trackable targets to see which one (if any) is visible.
//        targetVisible = false;
//        for (VuforiaTrackable trackable : allTrackables) {
//            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                opMode.telemetry.addData("Visible Target", trackable.getName());
//                targetVisible = true;
//
//                // getUpdatedRobotLocation() will return null if no new information is available since
//                // the last time that call was made, or if the trackable is not currently visible.
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//                break;
//            }
//        }
//
//        // Provide feedback as to where the robot is located (if we know).
//        if (targetVisible) {
//            // express position (translation) of robot in inches.
//            VectorF translation = lastLocation.getTranslation();
//            opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//            // express the rotation of the robot in degrees.
//            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//            opMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//
//            vuforiaPos = new Vector2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
//
//            opMode.telemetry.addData("Vuforia pos", vuforiaPos);
//            return vuforiaPos;
//        }
//        else {
//            opMode.telemetry.addData("Visible Target", "none");
//            return null;
//        }
//    }
//
//    /**
//     * @return robot's raw position according to Vuforia
//     */
//    public Vector2d getRawVuforiaImagePos() {
//        for (VuforiaTrackable trackable : allTrackables) {
//            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                targetVisible = true;
//                try {
//                    VectorF vuforiaRawPosition = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedVuforiaCameraFromTarget().getTranslation();
//                    double distanceInches = Math.hypot(vuforiaRawPosition.get(0), vuforiaRawPosition.get(2)) / mmPerInch;
////                    opMode.telemetry.addData("VUFORIA LOCATION ", vuforiaRawPosition);
////                    opMode.telemetry.addData("DISTANCE (INCHES)", distanceInches);
//                    return new Vector2d(vuforiaRawPosition.get(2) / mmPerInch, vuforiaRawPosition.get(2) / mmPerInch);
//                } catch (Exception ignored){
//                    RobotLog.v("Failed to read Raw vuforia Pos");
//                    return null;
//                }
//
//            }
//        }
//        return null;
//    }
//
//    /**
//     * @param vec1
//     * @param vec2
//     * @param maxDistance how big can the difference between the positions be
//     * @return if both positions are different from eachother, returns true.  Else it returns false
//     */
//    public boolean isBigJump(Vector2d vec1, Vector2d vec2, double maxDistance) {
//        double deltaX = vec1.getX() - vec2.getX();
//        double deltaY = vec1.getY() - vec2.getY();
//        double distance = Math.hypot(deltaX, deltaY);
//        return distance > maxDistance;
//    }
//    /**
//     * @param pose1 position 1
//     * @param pose2 position 2
//     * @param maxDistance how big can the difference between the positions be
//     * @return if both positions are far apart from each other, returns true.  Else it returns false
//     */
//    public boolean isBigJump(Pose2d pose1, Pose2d pose2, double maxDistance) {
//        double deltaX = pose1.getX() - pose2.getX();
//        double deltaY = pose1.getY() - pose2.getY();
//        double distance = Math.hypot(deltaX, deltaY);
//        return distance > maxDistance;
//    }
//
//    /**
//     * Writes a file to be read later
//     * @param filename the filename of
//     * @param number number to write in the file
//     */
//    public void writeFile(String filename, double number)
//    {
//        try {
//            File file = AppUtil.getInstance().getSettingsFile(filename);
//            ReadWriteFile.writeFile(file, Double.toString(number));
//            opMode.telemetry.log().add("saved " + number + " in " + filename);
//        }
//        catch(Exception e)
//        {
//            opMode.telemetry.log().add("Unable to write " + number + " in " + filename);
//        }
//    }
//
//    /**
//     * Writes a file to be read later
//     * @param filename the filename of
//     * @param str what to write in the file
//     */
//    public void writeFile(String filename, String str)
//    {
//        try {
//            File file = AppUtil.getInstance().getSettingsFile(filename);
//            ReadWriteFile.writeFile(file, str);
//            opMode.telemetry.log().add("saved \"" + str + "\" in " + filename);
//        }
//        catch(Exception e)
//        {
//            opMode.telemetry.log().add("Unable to write \"" + str + "\" in " + filename);
//        }
//    }
//
//    /**
//     * Reads a file
//     * @param filename name of the file
//     * @return the number read from the file
//     */
//    public double readFile(String filename)
//    {
//        try {
//            double output=0;
//            File file= AppUtil.getInstance().getSettingsFile(filename);
//            output = Double.parseDouble(ReadWriteFile.readFile(file).trim());
//            opMode.telemetry.log().add("read " + output + " in " + filename);
//            return output;
//        }
//        catch (Exception e)
//        {
//            opMode.telemetry.log().add("Unable to read " + filename + ", returning 0");
//            return 0;
//        }
//    }
//    /**
//     * Reads a file
//     * @param filename name of the file
//     * @return the string read from the file
//     */
//    public String readFileString(String filename)
//    {
//        try {
//            String output;
//            File file= AppUtil.getInstance().getSettingsFile(filename);
//            output = ReadWriteFile.readFile(file).trim();
//            opMode.telemetry.log().add("read " + output + " in " + filename);
//            return output;
//        }
//        catch (Exception e)
//        {
//            opMode.telemetry.log().add("Unable to read " + filename + ", returning 0");
//            return "";
//        }
//    }
//
//    /**
//     * saves the robot's position in to a file to be read later
//     * @param vector2d robot's position
//     */
//    public void savePos(Vector2d vector2d) {savePos(new Pose2d(vector2d.getX(), vector2d.getY(), Math.toRadians(0)));}
//
//    /**
//     * saves the robot's position in to a file to be read later
//     * @param pose2d robot's position
//     */
//    public void savePos(Pose2d pose2d) {
//        double x = pose2d.getX();
//        double y = pose2d.getY();
//        double heading = pose2d.getHeading();
//        StringBuilder stringBuilder = new StringBuilder();
//        String posFile = stringBuilder.append(x).append(",").append(y).append(",").append(heading).toString();
//        RobotLog.v("saved POS as " + posFile + " to " + POS_FILE_NAME);
//        writeFile(POS_FILE_NAME, posFile);
//    }
//
//    /**
//     * @return reads the robot's position
//     */
//    public Pose2d readPos() {
//        String posString = readFileString(POS_FILE_NAME);
//        String[] nums = posString.split(",");
//        Pose2d pos = new Pose2d(Double.parseDouble(nums[0]), Double.parseDouble(nums[1]), Double.parseDouble(nums[2]));
//        return pos;
//    }
//    public TrajectoryBuilder trajectoryBuilder() {
//        return new TrajectoryBuilder(getPoseEstimate(), constraints);
//    }
//    public TrajectoryBuilder trajectoryBuilder(boolean reversed) {
//        return new TrajectoryBuilder(getPoseEstimate(), reversed, constraints);
//    }
//    public TrajectoryBuilder trajectoryBuilder(boolean reversed, DriveConstraints constraints) {
//        return new TrajectoryBuilder(getPoseEstimate(), reversed, constraints);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
//        return new TrajectoryBuilder(startPose, constraints);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
//        return new TrajectoryBuilder(startPose, reversed, constraints);
//    }
//
//    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
//        return new TrajectoryBuilder(startPose, startHeading, constraints);
//    }
//
//    public void turn(double angle) {
//        turn(angle, 0, 0, 0);
//    }
//    public void turn(double angle, double vi, double ai, double ji) {
//        double heading = getPoseEstimate().getHeading();
//
//        lastPoseOnTurn = getPoseEstimate();
//
//        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(heading, vi, ai, ji),
//                new MotionState(heading + angle, 0, 0, 0),
//                constraints.maxAngVel,
//                constraints.maxAngAccel,
//                constraints.maxAngJerk
//        );
//
//        turnStart = clock.seconds();
//        mode = Mode.TURN;
//    }
//
//    /**
//     * @param angle make the robot turn
//     */
//    public void turnSync(double angle) {
//        turn(angle);
//        waitForIdle();
//    }
//
//    /**
//     * @param target target position to turn to
//     * @return the angle to turn to turn toward the target position
//     */
//    public double turnTowardsAngle(Vector2d target) {return turnTowardsAngle(target,new Vector2d());}
//
//    /**
//     * @param target target position to turn to
//     * @param start starting position to turn to
//     * @return the angle to turn to turn toward the target position
//     *
//     * Use this with turnRelativeSync()
//     */
//    public double turnTowardsAngle(Vector2d target, Pose2d start) {return turnTowardsAngle(target, new Vector2d(start.getX(), start.getY()));}
//
//    /**
//     * @param target target position to turn to
//     * @param start starting position to turn to
//     * @return the angle to turn to turn toward the target position
//     *
//     * Use this with turnRelativeSync()
//     */
//    public double turnTowardsAngle(Vector2d target, Vector2d start) {
//        double deltaX = target.getX() - start.getX();
//        double deltaY = target.getY() - start.getY();
//
//        double angle = Math.atan2(deltaY, deltaX);
//        return (Double.isNaN(angle)) ? 0 : angle;
//    }
//
//    /**
//     * Turns to an angle relative to the field.
//     * @param angle
//     */
//    public void turnRelativeSync(double angle)
//    {
//        double target=angle-getPoseEstimate().getHeading();
//        //target-=Math.toRadians(90);
//        if (Math.abs(target)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
//        {
//            target+=(target>=0) ? Math.toRadians(-360) : Math.toRadians(360);
//        }
//        opMode.telemetry.log().add("inputted Angle: " + angle + " , turning to: " + target);
//        RobotLog.d("Turning Relative to heading " + angle + ", amount turning: " + target);
//        turnSync(target);
//    }
//    public void turnRelative(double angle)
//    {
//        double target=angle-getPoseEstimate().getHeading();
//        //target-=Math.toRadians(90);
//        if (Math.abs(target)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
//        {
//            target+=(target>=0) ? Math.toRadians(-360) : Math.toRadians(360);
//        }
//        opMode.telemetry.log().add("inputted Angle: " + angle + " , turning to: " + target);
//        RobotLog.d("Turning Relative to heading " + angle + ", amount turning: " + target);
//        turn(target);
//    }
//    public void followTrajectory(Trajectory trajectory) {
//        follower.followTrajectory(trajectory);
//        mode = Mode.FOLLOW_TRAJECTORY;
//    }
//
//    public void followTrajectorySync(Trajectory trajectory) {
//        followTrajectory(trajectory);
//        waitForIdle();
//    }
//
//    public Pose2d getLastError() {
//        switch (mode) {
//            case FOLLOW_TRAJECTORY:
//                return follower.getLastError();
//            case TURN:
//                return new Pose2d(0, 0, turnController.getLastError());
//            case IDLE:
//                return new Pose2d();
//        }
//        throw new AssertionError();
//    }
//
//    /**
//     * Updates the robot's position and packet telemetry.
//     * Uses setPacketAction() to add information to the packet
//     */
//    public void update() {
//
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas fieldOverlay = packet.fieldOverlay();
//
//        updatePoseEstimate();
//
//        Pose2d currentPose = getPoseEstimate();
//
//        if (!isInsideField(currentPose)) {
//            RobotLog.setGlobalWarningMessage("BOT IS OUTSIDE FIELD AT POSITION %s", currentPose.toString());
//            messageDisplayed = true;
//        } else if (messageDisplayed) {
//            RobotLog.clearGlobalWarningMsg();
//            messageDisplayed = false;
//        }
//
//        if (useVuforia) {
//            Vector2d vuforiaPos = getPosFromVuforia();
//            Vector2d rawVuforiaPos = getRawVuforiaImagePos();
//            //if the distance is too big, do not use Vuforia.
//            double distanceFromImage;
//            try {distanceFromImage = Math.hypot(rawVuforiaPos.getX(), rawVuforiaPos.getY()); }
//            catch (Exception ignored) {distanceFromImage = 9999;}
//            packet.addLine("Distance from Image: " + distanceFromImage);
//
//            if (vuforiaPos != null && vuforiaPos.getX() != 0 && vuforiaPos.getY() != 0 && distanceFromImage < 50 && (!isBigJump(vuforiaPos, lastVuforiaPos, 10) || timeBetweenVuforiaTargetDetection.seconds() > 2)) {
//                lastVuforiaPos = vuforiaPos;
//
//                Vector2d averagePos = vuforiaPos.plus(new Vector2d(currentPose.getX(), currentPose.getY())).div(2);
//                packet.addLine("Average POS: " + averagePos.toString());
//                Pose2d newPose = new Pose2d(averagePos.getX(), averagePos.getY(), 0);
//                setPoseEstimate(newPose);
//                currentPose = getPoseEstimate();
//                timeBetweenVuforiaTargetDetection.reset();
//            }
//        }
//
//        Pose2d lastError = getLastError();
//
//        poseHistory.add(currentPose);
//
//        packet.put("mode", mode);
//
//        packet.put("x", currentPose.getX());
//        packet.put("y", currentPose.getY());
//        packet.put("heading", currentPose.getHeading());
//
//        packet.put("xError", lastError.getX());
//        packet.put("yError", lastError.getY());
//        packet.put("headingError", lastError.getHeading());
//
//        //draw power shot positions
//        fieldOverlay.setFill("#FF0000");
//        fieldOverlay.fillCircle(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y, 1);
//        fieldOverlay.fillCircle(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y, 1);
//        fieldOverlay.fillCircle(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y, 1);
//
//        //fieldOverlay.fillCircle(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y, 8);
//
//        if (dpadLeft1.onPress()) mode = Mode.IDLE;
//
//        //if a gamepadStick moves, stop the roadrunner stuff.
//        if (INTERRUPT_TRAJECTORIES_WITH_GAMEPAD) if (isStickMoved(opMode.gamepad1.left_stick_x,opMode.gamepad1.left_stick_y) || isStickMoved(opMode.gamepad1.right_stick_x, opMode.gamepad1.right_stick_y)) mode = Mode.IDLE;
//
//        switch (mode) {
//            case IDLE:
//                // do nothing
//                break;
//            case TURN: {
//                double t = clock.seconds() - turnStart;
//
//                MotionState targetState = turnProfile.get(t);
//
//                turnController.setTargetPosition(targetState.getX());
//
//                double correction = turnController.update(currentPose.getHeading());
//                if (Double.isNaN(correction)) correction = 0;
//
//                double targetOmega = targetState.getV();
//                double targetAlpha = targetState.getA();
//                setDriveSignal(new DriveSignal(new Pose2d(
//                        0, 0, targetOmega + correction
//                ), new Pose2d(
//                        0, 0, targetAlpha
//                )));
//
//                // RobotLog.v("Target Omega: " + targetOmega + "Correction: " + correction);
//
//                currentHeadingV = targetOmega + correction;
//                currentHeadingA = targetAlpha;
//
//                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());
//
//                fieldOverlay.setStroke("#4CAF50");
//                DashboardUtil.drawRobot(fieldOverlay, newPose);
//
//                if (t >= turnProfile.duration()) {
//                    mode = Mode.IDLE;
//                    setDriveSignal(new DriveSignal());
//                }
//
//                break;
//            }
//            case FOLLOW_TRAJECTORY: {
//                DriveSignal driveSignal = follower.update(currentPose);
//                setDriveSignal(driveSignal);
//
//                Trajectory trajectory = follower.getTrajectory();
//
//                fieldOverlay.setStrokeWidth(1);
//                fieldOverlay.setStroke("#4CAF50");
//                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
//                double t = follower.elapsedTime();
//                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));
//
//                fieldOverlay.setStroke("#3F51B5");
//                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
//
//                if (!follower.isFollowing()) {
//                    mode = Mode.IDLE;
//                    setDriveSignal(new DriveSignal());
//                }
//
//                break;
//            }
//        }
//
//        fieldOverlay.setStroke("#3F51B5");
//        DashboardUtil.drawRobot(fieldOverlay, currentPose);
//
//        DashboardUtil.drawLine(fieldOverlay, currentPose);
//
//        packetAction.addToPacket(packet, fieldOverlay);
//
//        packet.addLine("Time between last update: " + timeBetweenUpdates.milliseconds() + " milliseconds");
//
//        dashboard.sendTelemetryPacket(packet);
//
//        runAtAllTimes();
//
//        timeBetweenUpdates.reset();
//    }
//
//    public void waitForIdle() {
//        while (!Thread.currentThread().isInterrupted() && isBusy()) {
//            update();
//        }
//    }
//
//    /**
//     * Exactly what you think
//     */
//    public void doPowerShotsTheClassicAndBetterWay() {
//        RobotLog.v("Launching Power Shot 1");
//        turnRelativeSync(turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), getPoseEstimate()));
//        turnRelativeSync(turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y), getPoseEstimate()));
//        //throw ring
//        setThrowerVelocity(getThrowerVelocityFromPositionPowerShot(getDistanceFromPowerShot(redPowerShots[0], getPoseEstimate()), AngleUnit.DEGREES), AngleUnit.DEGREES);
//        //setThrowerToTarget(getPoseEstimate());
//        //wait for launch speed to be ready
//        while (!isReadyToThrow()) {updateLightsBasedOnThrower();}
//        launchRing();
//        RobotLog.v("Thrower motor 0 VELO (when launched): " + (getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
//        ((LinearOpMode) opMode).sleep(750);
//        openIndexer();
//        //prepare to throw next ring
//        RobotLog.v("Launching Power Shot 2");
//        turnRelativeSync(turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), getPoseEstimate()));
//        turnRelativeSync(turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y), getPoseEstimate()));
//        //throw ring
//        setThrowerVelocity(getThrowerVelocityFromPositionPowerShot(getDistanceFromPowerShot(redPowerShots[1], getPoseEstimate()), AngleUnit.DEGREES), AngleUnit.DEGREES);
//        //setThrowerToTarget(getPoseEstimate());
//        //wait for launch speed to be ready
//        ((LinearOpMode) opMode).sleep(250);
//        while (!isReadyToThrow()) {updateLightsBasedOnThrower();}
//        launchRing();
//        RobotLog.v("Thrower motor 0 VELO (when launched): " + (getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
//        ((LinearOpMode) opMode).sleep(500);
//        openIndexer();
//        //prepare to throw next ring
//        RobotLog.v("Launching Power Shot 3");
//        turnRelativeSync(turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), getPoseEstimate()));
//        turnRelativeSync(turnTowardsAngle(new Vector2d(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y), getPoseEstimate()));
//        //throw ring
//        //setThrowerToTarget(getPoseEstimate());
//        setThrowerVelocity(getThrowerVelocityFromPositionPowerShot(getDistanceFromPowerShot(redPowerShots[0], getPoseEstimate()), AngleUnit.DEGREES), AngleUnit.DEGREES);
//        //wait for launch speed to be ready
//        while (!isReadyToThrow()) {updateLightsBasedOnThrower();}
//        ((LinearOpMode) opMode).sleep(250);
//        launchRing();
//        RobotLog.v("Thrower motor 0 VELO (when launched): " + (getThrowerVelocity(AngleUnit.DEGREES)[0] / 360) + "REV/s");
//        ((LinearOpMode) opMode).sleep(500);
//        openIndexer();
//        //stop thrower
//        setThrowerVelocity(0);
//    }
//
//    public void doRedPowerShots() {doRedPowerShots(getPoseEstimate());}
//    public void doRedPowerShots(Pose2d robotPos) {
//
//        for (FieldConstants.PowerShots redPowerShot : redPowerShots) {
//            doSingularPowerShot(redPowerShot, getPoseEstimate());
//        }
//
//    }
//    public void doSingularPowerShot(FieldConstants.PowerShots powerShot, Pose2d robotPos) {
//        double distFromPowerShot = getDistanceFromPowerShot(powerShot, robotPos);
//        setThrowerVelocity(getThrowerVelocityFromPositionPowerShot(distFromPowerShot, DEGREES), DEGREES);
//
//        //turn twice to make sure the robot is facing the right direction
//        for (int i = 0; i < 3; i++) turnTowardPowerShot(powerShot, getPoseEstimate());
//
//        //make sure velocity is correct
//        distFromPowerShot = getDistanceFromPowerShot(powerShot, getPoseEstimate());
//        RobotLog.v("Pshot: DISTANCE FROM POWERSHOT: " + distFromPowerShot + " for " + powerShot.name());
//        double throwerVelocity = getThrowerVelocityFromPositionPowerShot(distFromPowerShot, DEGREES);
//        RobotLog.v("Pshot: THROWER SPEED FOR " + powerShot.name() + ": " + throwerVelocity + " + deg/sec");
//        setThrowerVelocity(throwerVelocity, DEGREES);
//
//        waitUntilReadyToThrow();
//        RobotLog.v("Pshot: LAUNCHING AT " + getThrowerVelocity(DEGREES)[0] + " deg/sec");
//        launchRing();
//        ((LinearOpMode) opMode).sleep(500);
//        openIndexer();
//
//    }
//    public void turnTowardPowerShot(FieldConstants.PowerShots powerShot) {turnTowardPowerShot(powerShot, getPoseEstimate());}
//    public void turnTowardPowerShot(FieldConstants.PowerShots powerShot, Pose2d robotPos) {
//        double angleToTurnTo = turnTowardsAngle(powerShot.getPowerShotPosition(), robotPos);
//        turnRelativeSync(angleToTurnTo);
//    }
//
//    /**
//     * Sets both of the throwing motors to a desired ticks per second.
//     * @param ticksPerSecond ticks per second to set the motor
//     */
//    public void setThrowerVelocity(double ticksPerSecond) {
//        if (ticksPerSecond == 0) {
//            thrower2.setPower(0);
//            thrower1.setPower(0);
//            targetVelocity = 0;
//            return;
//        }
//        try {
//            targetVelocity = ticksPerSecond;
//            thrower1.setVelocity(ticksPerSecond);
//            thrower2.setVelocity(ticksPerSecond);
//        } catch (Exception e) {
//            RobotLog.e("SETTING tick VELOCITY FAILED");
//        }
//    }
//
//    /**
//     * Set's both of the thrower motor's velocity to something.
//     * @param angularRate desired speed
//     * @param angleUnit (Degrees or Radians)
//     */
//    public void setThrowerVelocity(double angularRate, AngleUnit angleUnit) {
//        if (angularRate == 0) {
//            thrower2.setPower(0);
//            thrower1.setPower(0);
//            targetVelocity = 0;
//            return;
//        }
//        try {
//            targetVelocity = angularRate;
//            thrower1.setVelocity(angularRate, angleUnit);
//            thrower2.setVelocity(angularRate, angleUnit);
//        } catch (Exception e) {
//            RobotLog.e("SETTING angle VELOCITY FAILED");
//        }
//        if (angleUnit == AngleUnit.DEGREES) RobotLog.v("Setting Thrower Velocity to " + (angularRate/360) +"REV/s");
//        if (angleUnit == AngleUnit.RADIANS) RobotLog.v("Setting Thrower Velocity to " + (angularRate/(2*Math.PI)) +"REV/s");
//    }
//    /**
//     * @return Robot's thrower Velocity in ticks per second for both motors.
//     */
//    public double[] getThrowerVelocity() {
//        try {return new double[] {thrower1.getVelocity(), thrower2.getVelocity()}; }
//        catch (Exception e) {
//            return new double[] {0,0};
//        }
//    }
//    /**
//     * @param angleUnit
//     * @return Robot's thrower Velocity for both motors.
//     */
//    public double[] getThrowerVelocity(AngleUnit angleUnit) {
//        try {
//            return new double[] {thrower1.getVelocity(angleUnit), thrower2.getVelocity(angleUnit)};
//        } catch (Exception e) {
//            return new double[] {0,0};
//        }
//    }
//
//    /**
//     * stops both motors on the robot thrower
//     */
//    public void stopThrower() {
//        thrower1.setPower(0);
//        thrower2.setPower(0);
//    }
//
//    /**
//     * @return determines if the thrower is at the targetVelocity.  Must use degrees at the moment
//     */
//    public boolean isReadyToThrow() {
//        double[] velocities = getThrowerVelocity(AngleUnit.DEGREES);
//        double minVelo = targetVelocity - leewayVelo;
//        double maxVelo = targetVelocity + leewayVelo;
//        boolean isMotor0AtTarget = velocities[0] > minVelo && velocities[0] < maxVelo;
//        boolean isMotor1AtTarget = velocities[1] > minVelo && velocities[1] < maxVelo;
//
//        return  isMotor0AtTarget || isMotor1AtTarget;
//    }
//
//    /**
//     * Waits until the thrower is ready to launch the ring
//     */
//    public void waitUntilReadyToThrow() {
//        while (!isReadyToThrow() && ((LinearOpMode) opMode).opModeIsActive()) {
//            updateLightsBasedOnThrower();
//            update();
//        }
//    }
//
//    /**
//     * Gets the thrower velocity that will make the ring hit the goal, given
//     * the robot is facing the goal.
//     * @param currentPos Current Position of the robot
//     * @return The speed in terms of rev/s to launch the ring to the target goal
//     */
//    public double getThrowerVelocityFromPosition(Pose2d currentPos) {
//        //get distance based
//        double targetY = ThrowerUtil.getTargetY(currentPos, FieldConstants.RED_GOAL_X);
//        double dist = Math.hypot(currentPos.getX() - FieldConstants.RED_GOAL_X, currentPos.getY() - targetY);
//        double revPerSec = getThrowerVelocityFromPosition(dist);
//        return revPerSec;
//    }
//
//    /**
//     * @param currentPos Current Position of the robot
//     * @param angleUnit The desired unit of speed
//     * @return The speed to launch the ring to the target goal
//     */
//    public double getThrowerVelocityFromPosition(Pose2d currentPos, AngleUnit angleUnit) {
//        //get distance based
//        double targetY = ThrowerUtil.getTargetY(currentPos, FieldConstants.RED_GOAL_X);
//        double dist = Math.hypot(currentPos.getX() - FieldConstants.RED_GOAL_X, currentPos.getY() - targetY);
//        double unitsPerSec = getThrowerVelocityFromPosition(dist, angleUnit);
//        RobotLog.v("TARGET Y: " + targetY + ", DIST: " + dist + ", UNITS PER SEC: " + unitsPerSec);
//        return unitsPerSec;
//    }
//
//    /**
//     * Uses a equation from cubic regression from values we gathered from launching the ring a lot
//     * @param dist
//     * @param angleUnit (Degrees or Radians)
//     * @return The speed to launch the ring to the target goal
//     * will return NaN if the angleUnit is invalid
//     */
//    public double getThrowerVelocityFromPosition(double dist, AngleUnit angleUnit) {
//
//        double revPerSec = (-0.0001001 * Math.pow(dist, 3)) + (0.03045 * Math.pow(dist, 2)) - (2.859 * dist) + 132;
//        if (angleUnit == DEGREES) return revPerSec*360;
//        else if (angleUnit == RADIANS) return revPerSec*2*Math.PI;
//        return Double.NaN;
//    }
//
//    /**
//     * Uses a equation from cubic regression from values we gathered from launching the ring a lot
//     * @param dist Distance from goal
//     * @return The speed in terms of rev/s to launch the ring to the target goal
//     */
//    public double getThrowerVelocityFromPosition(double dist) {
//        //revPerSecond
//        return (-0.0001001 * Math.pow(dist, 3)) + (0.03045 * Math.pow(dist, 2)) - (2.859 * dist) + 132.4;
//    }
//
//    /**
//     * Uses a equation from cubic regression from values we gathered from launching the ring a lot
//     * to get the target velocity to launch the ring into a powerShot
//     * @param dist Distance from power shot
//     * @return The speed in terms of rev/s to launch the ring to the target power shot
//     */
//    public double getThrowerVelocityFromPositionPowershot(double dist) {
//        //revPerSecond
//        return  (3.686e-05 * Math.pow(dist, 3)) - (0.008383*Math.pow(dist, 2)) + (0.7675 * dist) + 17.52 + POWER_SHOT_VELO_ADD;
//    }
//
//    /**
//     * Uses a equation from cubic regression from values we gathered from launching the ring a lot
//     * Uses a equation from cubic regression from values we gathered from launching the ring a lot
//     * @param dist distance from power shot
//     * @param angleUnit (Degrees or Radians)
//     * @return The speed to launch the ring to the target goal
//     * will return NaN if the angleUnit is invalid
//     */
//    public double getThrowerVelocityFromPositionPowerShot(double dist, AngleUnit angleUnit) {
//
//        double revPerSec  =  (3.686e-05 * Math.pow(dist, 3)) - (0.008383*Math.pow(dist, 2)) + (0.7675 * dist) + 17.52 + POWER_SHOT_VELO_ADD;
//        RobotLog.v("setting thrower velocity to " + revPerSec + " REV/s with powershot distance of " + dist + " inches");
//        if (angleUnit == DEGREES) return revPerSec*360;
//        else if (angleUnit == RADIANS) return revPerSec*2*Math.PI;
//        return Double.NaN;
//    }
//
//    /**
//     * @param powershot powershot object
//     * @return get's the distance from the robot to a powershot
//     */
//    public double getDistanceFromPowerShot(FieldConstants.PowerShots powershot) {return getDistanceFromPowerShot(powershot, getPoseEstimate());}
//
//    /**
//     * @param powershot powershot object
//     * @param robotPos position of robot
//     * @return get's the distance from the robot to a powershot
//     */
//    public double getDistanceFromPowerShot(FieldConstants.PowerShots powershot, Pose2d robotPos) {
//        double deltaX = powershot.getPowerShotPosition().getX() - robotPos.getX();
//        double deltaY = powershot.getPowerShotPosition().getY() - robotPos.getY();
//        RobotLog.v("DISTANCE FROM " + powershot.name() + " POWERSHOT: " + Math.hypot(deltaX, deltaY) + "inches");
//        return Math.hypot(deltaX, deltaY);
//    }
//
//    /**
//     * Updates the robot's lights based on the thrower's current speed compared to the target speed
//     * Lights are RED if the thrower isn't going fast enough
//     * Lights are YELLOW if the thrower is too fast
//     * Lights are GREEN when the current speed is matches (close enough) to the target speed
//     */
//    public void updateLightsBasedOnThrower() {
//        if (targetVelocity == 0 || !isReadyToThrow()) {
//            double currentV = getThrowerVelocity(DEGREES)[0];
//            if (targetVelocity > currentV) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            else lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//        } else lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//    }
//    public boolean isInsideField(Pose2d pose2d) {
//        return Math.abs(pose2d.getY()) < FieldConstants.TOP_OF_FIELD && Math.abs(pose2d.getX()) < FieldConstants.TOP_OF_FIELD;
//    }
//
//    /**
//     * launches the ring by moving the "indexer" servo to push the ring into the launching,
//     * provided there is a ring loaded in the robot
//     */
//    public void launchRing() {
//        try {
//            indexer.setPosition(.55);
//        } catch (Exception e) {
//            RobotLog.e("LAUNCH RING FAILED");
//        }
//    }
//
//    /**
//     * opens the indexer, the servo that launches the rings
//     */
//    public void openIndexer() {
//        try {
//            indexer.setPosition(.2);
//        } catch (Exception e) {
//            RobotLog.e("OPEN INDEXER FAILED");
//        }
//    }
//    /**
//     * moves the indexer between it's launched and open positions
//     * Never know when you need this.
//     */
//    public void midIndexer(){
//        try {
//            indexer.setPosition(.38);
//        } catch (Exception e) {
//            RobotLog.e("MID INDEXER FAILED");
//        }
//    }
//
//    /**
//     * grabs the wobble goal using the wobble goal grabber
//     */
//    public void grabWobbleGoal() {
//
//        try {
//            wobbleGoalGrabber.setPosition(.13);
//        } catch (Exception e) {
//            RobotLog.e("GRAB WOB GOAL FAILED");
//        }
//    }
//    /**
//     * releases the wobble goal using the wobble goal grabber
//     */
//    public void releaseWobbleGoal() {
//        try {
//            wobbleGoalGrabber.setPosition(.45);
//        } catch (Exception e) {
//            RobotLog.e("RELEASE WOB GOAL FAILED");
//        }
//    }
//
//    public void openWobbleGrabberHalfway() {
//        wobbleGoalGrabber.setPosition(.3);
//    }
//
//    /**
//     * Lower's the wobble goal grabber so it can grab/release wobble goals
//     * May cause the wobble goal to touch the flow
//     */
//    public void lowerWobvator() {
//        try {
//            wobbleGoalElevator.setPosition(.355);
//        } catch (Exception e) {
//            RobotLog.e("LOWER WOBVATOR FAILED");
//        }
//    }
//    /**
//     * Lower's the wobble goal grabber but not all the way so the wobble goal that is has grabbed does not risk touching the floor
//     */
//    public void lowerWobvatorByNotAllTheWay() {
//        try {
//            wobbleGoalElevator.setPosition(.38);
//        } catch (Exception e) {
//            RobotLog.e("LOWER WOBVATOR but not all the way FAILED");
//        }
//    }
//    /**
//     * Raises the wobble goal grabber as much as possible
//     */
//    public void raiseWobvator() {
//        try {
//            wobbleGoalElevator.setPosition(.42);
//        } catch (Exception e) {
//            RobotLog.e("RAISE WOBVATOR FAILED");
//        }
//    }
//
//    public void intake() {
//        intake.setPower(1);
//        inTaking = true;
//        outTaking = false;
//    }
//    public void outtake() {
//        intake.setPower(-1);
//        outTaking = true;
//        inTaking = false;
//    }
//    public void stopIntake() {
//        intake.setPower(0);
//        outTaking = false;
//        inTaking = false;
//    }
//    public boolean isIntaking() {
//        return inTaking;
//    }
//    public boolean isOutTaking() {
//        return outTaking;
//    }
//
//    /**
//     * This method allows a servo to be controlled by a gamepad stick
//     * @param servo the servo to be updated
//     * @param gamepadStick a gamepad stick number, between -1 and 1
//     * @param speed sensitivity of gamepad stick.  Use the value 1 if your confused
//     * @param max servo maximum position
//     * @param min servo minimum position
//     */
//    public void updateServo(Servo servo, double gamepadStick, double speed, double max, double min)
//    {
//        if (Math.abs(gamepadStick) < .2) return;
//        double posToAdd = gamepadStick*speed;
//        //opMode.telemetry.addData("changing pos by ",posToAdd);
//        double servoCurrentPos = servo.getPosition();
//        double targetPos = posToAdd + servoCurrentPos;
//        //if ((servoCurrentPos > min && gamepadStick > 0) || (servoCurrentPos < max && gamepadStick < 0))
//        //{
//        if (targetPos > max) servo.setPosition(max);
//        else if (targetPos < min) servo.setPosition(min);
//        else {
//            servo.setPosition(targetPos);
//        }
//        //}
//    }
//
//    /**
//     * This method allows a servo to be controlled by a gamepad stick
//     * @param servo the servo to be updated
//     * @param gamepadStick a gamepad stick number, between -1 and 1
//     * @param speed sensitivity of gamepad stick.  Use the value 1 if your confused
//     */
//    public void updateServo(Servo servo, double gamepadStick, double speed)
//    {
//        updateServo(servo, gamepadStick, speed, 0, 1);
//    }
//
//    /**
//     * @param distanceSensor a Maxbotix Ultrasonic Sensor
//     * @return the distance the Maxbotix Ultrasonic Sensor in inches
//     */
//    public double getDistance(AnalogInput distanceSensor) {
//        //convert voltage to inches
//        return distanceSensor.getVoltage()*213.45759532208388;
//    }
//
//    /**
//     * @return target Velocity of the Robot's thrower
//     */
//    public double getTargetVelocity() {
//        return targetVelocity;
//    }
//    public void runAtAllTimes() //anything in here runs at all times during auton because this method is ran during roadRunner's state machine
//    {
//
//    }
//
//    /**
//     * @param x gamepad stick X
//     * @param y gamepad stick y
//     * @return true if the gamepad stick is moved, false if the gamepad stick is still.
//     */
//    public boolean isStickMoved(double x, double y) {
//        return Math.abs(x) > .3 || Math.abs(y) > .3;
//    }
//
//    public boolean isBusy() {
//        return mode != Mode.IDLE;
//    }
//
//    public void setMode(DcMotor.RunMode runMode) {
//        for (DcMotorEx motor : motors) {
//            motor.setMode(runMode);
//        }
//    }
//
//    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
//        for (DcMotorEx motor : motors) {
//            motor.setZeroPowerBehavior(zeroPowerBehavior);
//        }
//    }
//
//    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
//        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
//        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
//    }
//
//    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
//        for (DcMotorEx motor : motors) {
//            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
//                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
//            ));
//        }
//    }
//
//    @NonNull
//    @Override
//    public List<Double> getWheelPositions() {
//        List<Double> wheelPositions = new ArrayList<>();
//        for (DcMotorEx motor : motors) {
//            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
//        }
//        return wheelPositions;
//    }
//
//    public List<Double> getWheelVelocities() {
//        List<Double> wheelVelocities = new ArrayList<>();
//        for (DcMotorEx motor : motors) {
//            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
//        }
//        return wheelVelocities;
//    }
//    @Override
//    public void setMotorPowers(double v, double v1, double v2, double v3) {
//        leftFront.setPower(v);
//        leftRear.setPower(v1);
//        rightRear.setPower(v2);
//        rightFront.setPower(v3);
//    }
//    public void stopRobot()
//    {
//        leftFront.setPower(0);
//        leftRear.setPower(0);
//        rightFront.setPower(0);
//        rightRear.setPower(0);
//    }
//
//    /**
//     * @return Angle of the Intel T265 Realsense
//     * (unless boolean variable USE_REALSENSE is false, then return the IMU angle)
//     */
//    @Override
//    public double getRawExternalHeading() {
//        if (USE_REALSENSE)
//            return T265LocalizerRR.getHeading();
//        else
//            return imu.getAngularOrientation().firstAngle;
//    }
//
//    /**
//     * @return the IMU angle of the robot
//     */
//    public double getRawIMUangle() {
//        return norm(imu.getAngularOrientation().firstAngle - imuZeroVal);
//    }
//    public void zeroIMU() {
//        imuZeroVal = imu.getAngularOrientation().firstAngle;
//    }
//
//    public AnalogGyroSensor getAnalogGyroSensor() {
//        return gyroSensor;
//    }
//    private double norm(double angle) {
//        while (angle > Math.toRadians(360)) angle-=360;
//        while (angle > Math.toRadians(0)) angle+=360;
//        return angle;
//    }
//    public double getCurrentHeadingA() {
//        return currentHeadingA;
//    }
//
//    public double getCurrentHeadingV() {
//        return currentHeadingV;
//    }
//
//    public void setPacketAction(PacketAction packetAction) {
//        this.packetAction = packetAction;
//    }
//
//
////    /**
////     * Stores data about a ring that is launched
////     */
////    public class Ring {
////
////        private Vector2d position;
////        private double startTime = 0;
////        private double launchTime = 0;
////
////        private double changeXby = 0;
////        private double changeYby = 0;
////        private double xDist = 0;
////        private double yDist = 0;
////
////        public Ring() {
////            position = new Vector2d(0,0);
////            startTime = System.currentTimeMillis();
////        }
////
////        public Ring(Vector2d position) {
////            this.position = position;
////            startTime = System.currentTimeMillis();
////        }
////
////        public void beginLaunch() {
////            startTime = System.currentTimeMillis();
////        }
////        public void update() {
////            launchTime = System.currentTimeMillis() - startTime;
////        }
////        public Canvas addRingToFieldOverlay(Canvas fieldOverlay) {
////            return fieldOverlay.strokeCircle(0,0,2);
////        }
////    }
//}