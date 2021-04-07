package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 - This houses all of our hardware
 - in here we initialize all of the hardware
 */

public class HardWareMap{

        //setting up variables for each motor
        public DcMotor backRight=null, backLeft=null, frontRight=null, frontLeft = null; //drive motors

        public DcMotorEx shootyMcShootShoot=null, shooter2 = null; //shooting motors

        public DcMotor intake=null;//intake motor

        public DcMotor horizontal; //odometry wheel

        public DcMotor wobbleLift=null;


        public Servo indexer;//indexer that is used to push the ring out of the hopper into the shooter
        public Servo hopper;//holds the rings before shooting
        public Servo intakeServo;//holds up the intake bar before match
        public Servo wobbleGrab, upperSupper;

        public DigitalChannel topSwitch = null; //Stops the lift when it's all the way up
        public DigitalChannel bottomSwitch = null; ////Stops the lift when it's all the way down

        public RevBlinkinLedDriver blinkinLedDriver = null;



        public static final double COUNTS_PER_MOTOR_REV =  383.6 ;
        public static final double DRIVE_GEAR_REDUCTION = 1;
        public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

        public static final double COUNTS_PER_MOTOR_REV2 =  8192;
        public static final double DRIVE_GEAR_REDUCTION2 = 1;
        public static final double WHEEL_DIAMETER_INCHES2 = 1.37795;     // For figuring circumference
        public static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) /
            (WHEEL_DIAMETER_INCHES2 * Math.PI);

        public final double COUNTS_PER_ROTATION = 28;

//-     constant for the hopper shooter position
        public final double SHOOT_POSITION = .525;
        public final double INTAKE_POSITION = 0;

//-     Constants for wobble grabber
        public final double OPEN = .65;
        public final double CLOSED = .3;


        HardwareMap hwMap = null;

        public void initHardware(HardwareMap ahwMap) {
            hwMap = ahwMap;



        /*
        -naming hardware for the configuration file on the Robot Controller
         */
            backRight = ahwMap.get(DcMotor.class, "rightB"); //port 2 control hub, has horizontal encoder
            backLeft = ahwMap.get(DcMotor.class, "leftB"); //port 1 control hub, has Left Vertical Encoder
            frontRight = ahwMap.get(DcMotor.class, "rightF"); //port 3 control hub, has Right Vertical Encoder
            frontLeft = ahwMap.get(DcMotor.class, "leftF"); //port 0 control hub

//-         odometry wheels
//
//            verticalLeft = ahwMap.dcMotor.get("rightF"); //port 0 on control hub
//            verticalRight = ahwMap.dcMotor.get("vertR"); //port 2 on expansion hub
            horizontal = ahwMap.dcMotor.get("wobLift"); //port 3 on expansion Hub

            shootyMcShootShoot = ahwMap.get(DcMotorEx.class, "shoot1"); //port 0 expansion hub
            shooter2 = ahwMap.get(DcMotorEx.class, "shoot2"); //port 1 expansion hub

            intake = ahwMap.get(DcMotor.class, "intake"); //port 2 expansion hub

            wobbleLift = ahwMap.get(DcMotor.class, "wobLift"); //port 3 expansion hub

            hopper = ahwMap.get(Servo.class, "hop"); //port 0 on control hub
            indexer = ahwMap.get(Servo.class, "index"); //port 1 on control hub
            intakeServo = ahwMap.get(Servo.class, "intS"); //port 2 on control hub
            wobbleGrab = ahwMap.get(Servo.class, "wobGrab"); //port 4 on control hub
            upperSupper = ahwMap.get(Servo.class, "uppy"); //port 5 on control hub


            blinkinLedDriver = ahwMap.get(RevBlinkinLedDriver.class, "LED"); //servo port 3 on control hub

            topSwitch = ahwMap.get(DigitalChannel.class,"top"); //Digital Channel port 0-1 on control hub
            bottomSwitch = ahwMap.get(DigitalChannel.class,"bot"); //Digital Channel port 0-1 on control hub


            backRight.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            frontLeft.setPower(0);

            shootyMcShootShoot.setPower(0);
            shooter2.setPower(0);

            intake.setPower(0);

            wobbleLift.setPower(0);

            //- reversing the left motors so it can drive straight
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);

            shooter2.setDirection(DcMotor.Direction.REVERSE);
            shootyMcShootShoot.setDirection(DcMotor.Direction.REVERSE);

            intake.setDirection(DcMotor.Direction.REVERSE);


            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shootyMcShootShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            shootyMcShootShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//-         adding brakes to the motor so they immediately stop when told
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
}
