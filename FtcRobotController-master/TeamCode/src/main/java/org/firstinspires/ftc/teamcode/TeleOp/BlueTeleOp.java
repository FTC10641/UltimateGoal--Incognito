package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.SubSystems.HardWareMap;
import org.firstinspires.ftc.teamcode.SubSystems.Methods;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors;

import java.io.File;

/**
 - This houses all the controls that we use during TeleOp for the blue side of the field
 */

@TeleOp(name="BLUE", group="Linear Opmode")
public class BlueTeleOp extends LinearOpMode {
    HardWareMap robot = new HardWareMap();
    Methods method = new Methods();
    Sensors sensors = new Sensors();
    ElapsedTime time = new ElapsedTime();

    //- making the variable for the turning for the powershots
    private boolean turned;


    private boolean shooter, endGameShoot; //- setting the variables for the 2 differents RPM speeds
    //- These are the variables to calculate the RPM purely for telemetry
    private double currentTick, currentTick2;
    private double currentRPM, currentRPM2;
    private double lastTick = 0, lastTick2 = 0;
    private double currentTime;
    private double lastTime;

    //- This reads a file that has the gyro angle from the end of autonomous
//- with this value we can face the goal (0 heading) depite the robot not being straight at the end of autonmous
    File autonomousGyroFile = AppUtil.getInstance().getSettingsFile("autonomousGyroFile.txt");
    double autoZValue;

    //- naming the patterns for the LEDs
    private RevBlinkinLedDriver.BlinkinPattern endGameRPM, teleOpRPM, atRest;

    @Override
    public void runOpMode() throws InterruptedException {
        //- telemetry that tells the driver the robotis initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//-     Initializing the hardwareMap, sensors, and method classes so that we can use them in this class
        robot.initHardware(hardwareMap);
        sensors.initSensors(hardwareMap);
        method.initMethods(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        //- setting colors patterns to each name of pattern
        //- powerShot RPM = flashing rainbows, highGoal RPM = yellow light chase, atRest = flashing red
        endGameRPM = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
        teleOpRPM = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;
        atRest = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;

        waitForStart(); //- waiting for the human to press start on the opmode
        while (opModeIsActive()) {
            robot.intakeServo.setPosition(0);//-drops the intake bar in case it didn't drop in auto, specifically if we didn't run auto

            /**
             - tries to get the value of the ZValue from the end of auto
             - catches if unable to read and automatically sets to 0
             */
            try {
                autoZValue = Double.parseDouble(ReadWriteFile.readFile(autonomousGyroFile).trim());
            } catch (Exception e) {
                autoZValue = 0;
            }

//-         Turns the robot at increments to easily score the powershots
            if (gamepad1.triangle && !turned){
                method.PIDRotate(sensors.getZAngle()+4,.5);
                turned = true;
            }
            else if(gamepad1.triangle && turned){
                method.PIDRotate(sensors.getZAngle()+5, .5);
            }//end of endgame turning
            else if (gamepad1.right_bumper){
                method.PIDRotate(0 - autoZValue,.5);
            }


//-         this moves the indexer so the robot shoots a ring
            if(gamepad1.cross){
                robot.indexer.setPosition(.4); //-shoot position
            }
            else {
                robot.indexer.setPosition(.55); //-rest position
            }

//-         Shooting ring Position
            if (gamepad1.square){
                robot.hopper.setPosition(robot.SHOOT_POSITION);
            }


        /*
         - Controls the shooter
         - The square will turn the shooter on for the high goal RPM
         - The touchpad will turn the shooter on for the powershots RPM
         - The triangle will turn the shooter off
         */
            if (gamepad2.square){
                shooter = true;
            } else if(gamepad2.triangle){
                shooter = false;
                endGameShoot = false;
            } else if (gamepad2.touchpad){
                endGameShoot= true;
                shooter = false;
            }
            if (shooter){
                method.shooterSpeed(4300, 4200);
                robot.blinkinLedDriver.setPattern(teleOpRPM);
            } else if (endGameShoot){
                robot.blinkinLedDriver.setPattern(endGameRPM);
                method.shooterSpeed(4000, 3900);
            }
            else {
                method.shooterSpeed(0,0);
                robot.blinkinLedDriver.setPattern(atRest);
            }
//-         Calculating the currenct RPM to feed back to the phone
            currentTick = robot.shootyMcShootShoot.getCurrentPosition();
            currentTick2 = robot.shooter2.getCurrentPosition();
            currentTime = runtime.time();
            currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
            currentRPM2 = method.calcRPM(lastTick2, lastTime, currentTick2, currentTime);
            lastTick = currentTick;
            lastTick2 = currentTick2;
            lastTime = currentTime;

//-         intake going in and setting the hopper to the intake position
            if (gamepad1.left_bumper){
                robot.intake.setPower(1);
                robot.hopper.setPosition(robot.INTAKE_POSITION);
            }
//-         intake going out
            else if (LeftTrigger()){
                robot.intake.setPower(-1);
            }
//-         intake off for when neither buttons are being pressed
            else {
                robot.intake.setPower(0);
            }

//-         initing the power for the motors, we will use this later to drive
            double frontRightPower;
            double backRightPower;
            double frontLeftPower;
            double backLeftPower;

//-         using the sticks and setting them to the forward/reverse, strafe, and turn movements respectively
//-         also multipling the turn variable by 4/5 because the robot turns too fast so this makes it easier for the driver to maneuver
            double strafe = -gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x*4/5;

//-         setting the Power variables to a range which allows a wider range of speeds
            frontLeftPower = Range.clip(drive + turn - strafe, -1, 1);
            backLeftPower = Range.clip(drive + turn + strafe, -1, 1);
            backRightPower = Range.clip(drive - turn - strafe, -1, 1);
            frontRightPower = Range.clip(drive - turn + strafe, -1, 1);

//-         adds the option to press a button and make the robot drive half speed
            robot.frontLeft.setPower(gamepad2.dpad_down ? frontLeftPower/2 : frontLeftPower);
            robot.backLeft.setPower(gamepad2.dpad_down ? backLeftPower/2 : backLeftPower);
            robot.frontRight.setPower(gamepad2.dpad_down ? frontRightPower/2 : frontRightPower);
            robot.backRight.setPower(gamepad2.dpad_down ? backRightPower/2 : backRightPower);

//-         making a power variable for the lift motor
            double liftPower;
//-         adding a range so there's a wider range of speeds that the lift can go
            liftPower = Range.clip(-gamepad2.right_trigger + gamepad2.left_trigger,-1.0,1.0);

//-         setting it so that when the driver hits the trigger and the limit switch is not pressed the lift will move
            if (LeftTrigger2() && robot.topSwitch.getState()){ //-down on lift
                robot.wobbleLift.setPower(liftPower);
            }
            else if (RightTrigger() && robot.bottomSwitch.getState()){ //-up on lift
                robot.wobbleLift.setPower(liftPower);
            }
            else {
                robot.wobbleLift.setPower(0);
            }

//-         this is the controls to the wobble grabber, cross is close, circle is open
            if (gamepad2.cross){
                robot.wobbleGrab.setPosition(robot.CLOSED); //-closed
            }
            else if (gamepad2.circle){
                robot.wobbleGrab.setPosition(robot.OPEN); //-open
            }

//-         this is the telemetry for teleOP
            telemetry.addData("AutoZValue", autoZValue); //-will read the Angle from the end of auto
            telemetry.addData("ZAngle: ", sensors.getZAngle());//- will read the current angle the robot is facing
            telemetry.addData("Shooter RPM = ", currentRPM);//-Reads the RPM of shooter motor one
            telemetry.addData("Shooter RPM2 = ", currentRPM2);//-Reads the RPM of shooter motor two
            telemetry.update();
        }
    }

    //- Since the triggers are float variables but we just want them to read true/false we use these methods to change them to boolean variables
    boolean LeftTrigger(){
        if (gamepad1.left_trigger >= 0.1){
            return (true);}
        else return (false);}

    boolean LeftTrigger2(){
        if (gamepad2.left_trigger >= 0.1){
            return (true);}
        else return (false);}

    boolean RightTrigger(){
        if (gamepad2.right_trigger >= 0.1){
            return (true);}
        else return (false);}

}
