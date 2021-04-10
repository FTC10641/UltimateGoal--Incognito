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

        //-setting up variables for each motor
        public DcMotor backRight = null, backLeft = null, frontRight = null, frontLeft = null; //-drive motors

        public DcMotorEx shootyMcShootShoot = null, shooter2 = null; //-shooting motors

        public DcMotor intake = null; //-intake motor

        public DcMotor wobbleLift=null;


        public Servo indexer; //-indexer that is used to push the ring out of the hopper into the shooter
        public Servo hopper; //-holds the rings before shooting
        public Servo intakeServo; //-holds up the intake bar before match
        public Servo wobbleGrab; //-the wobble grab is the servo that grabs the wobble

        public DigitalChannel topSwitch = null; //-Stops the lift when it's all the way up
        public DigitalChannel bottomSwitch = null; //-Stops the lift when it's all the way down

        public RevBlinkinLedDriver blinkinLedDriver = null; //- This Driver allows us to control the LEDs


        public static final double COUNTS_PER_MOTOR_REV =  383.6; //-How many ticks the motor feedsback per revolution
        public static final double DRIVE_GEAR_REDUCTION = 1; //-We use 1:1 gears for the drive train
        public static final double WHEEL_DIAMETER_INCHES = 4.0; //- For figuring circumference
        public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / //-Calculating how many ticks it takes to travel an inch
            (WHEEL_DIAMETER_INCHES * Math.PI);

        public final double COUNTS_PER_ROTATION = 28; //-Counts per rotation of the shooter motors

//-     Constants for the hopper shooter position
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
        -We have also added the placement of where it is on the hub for easy trouble shooting
         */
            backRight = ahwMap.get(DcMotor.class, "rightB"); //port 2 control hub
            backLeft = ahwMap.get(DcMotor.class, "leftB"); //port 1 control hub
            frontRight = ahwMap.get(DcMotor.class, "rightF"); //port 3 control hub
            frontLeft = ahwMap.get(DcMotor.class, "leftF"); //port 0 control hub

            shootyMcShootShoot = ahwMap.get(DcMotorEx.class, "shoot1"); //port 0 expansion hub
            shooter2 = ahwMap.get(DcMotorEx.class, "shoot2"); //port 1 expansion hub

            intake = ahwMap.get(DcMotor.class, "intake"); //port 2 expansion hub

            wobbleLift = ahwMap.get(DcMotor.class, "wobLift"); //port 3 expansion hub

            hopper = ahwMap.get(Servo.class, "hop"); //port 0 on control hub
            indexer = ahwMap.get(Servo.class, "index"); //port 1 on control hub
            intakeServo = ahwMap.get(Servo.class, "intS"); //port 2 on control hub
            wobbleGrab = ahwMap.get(Servo.class, "wobGrab"); //port 4 on control hub

            blinkinLedDriver = ahwMap.get(RevBlinkinLedDriver.class, "LED"); //servo port 3 on control hub

            topSwitch = ahwMap.get(DigitalChannel.class,"top"); //Digital Channel port 0-1 on control hub
            bottomSwitch = ahwMap.get(DigitalChannel.class,"bot"); //Digital Channel port 0-1 on control hub

//-         Setting the motors to 0 Power at Init
            backRight.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            frontLeft.setPower(0);

            shootyMcShootShoot.setPower(0);
            shooter2.setPower(0);

            intake.setPower(0);

            wobbleLift.setPower(0);

//-         reversing the left motors because the left side is mirrored
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);

//-         Reversing the shooter motors so that out is positive
            shooter2.setDirection(DcMotor.Direction.REVERSE);
            shootyMcShootShoot.setDirection(DcMotor.Direction.REVERSE);

//-         Reversing the intake motor so that in is positive direction
            intake.setDirection(DcMotor.Direction.REVERSE);

//-         This resets the encoders at the init so that we start our OpMode with encoders values at zero
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//-         Since we are using the Drive encoders we set the motors to "RUN_USING_ENCODERS"
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//-         Resetting the encoders of the shooter motors so we start the OpMode with encoders values at zero
            shootyMcShootShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//-         Setting the shooter motors to RUN_USING_ENCODER so that we're able to read the encoder value
            shootyMcShootShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//-         adding brakes to the motor so they immediately stop when told
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
}
