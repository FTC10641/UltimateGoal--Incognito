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

@TeleOp(name="BLUE", group="Linear Opmode")
public class BlueTeleOp extends LinearOpMode {
    HardWareMap robot = new HardWareMap();
    Methods method = new Methods();
    Sensors sensors = new Sensors();
    ElapsedTime time = new ElapsedTime();


    boolean toggle = true;
    boolean toggle2 = true;

    private boolean turned;



    private boolean shooter, endGameShoot;
    private double currentTick, currentTick2;
    private double currentRPM, currentRPM2;
    private double lastTick = 0, lastTick2 = 0;
    private double currentTime;
    private double lastTime;

    File autonomousGyroFile = AppUtil.getInstance().getSettingsFile("autonomousGyroFile.txt");
    double autoZValue;

    private RevBlinkinLedDriver.BlinkinPattern endGameRPM, teleOpRPM, atRest;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.initHardware(hardwareMap);
        sensors.initSensors(hardwareMap);
        method.initMethods(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        endGameRPM = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        teleOpRPM = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;
        atRest = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;

        waitForStart();
        while (opModeIsActive()) {
        robot.intakeServo.setPosition(0);

        /**
         + tries to get the value of the ZValue from the end of auto
         + catches if unable to read and automatically sets to 0
         */
        try {
            autoZValue = Double.parseDouble(ReadWriteFile.readFile(autonomousGyroFile).trim());
        } catch (Exception e) {
            autoZValue = 0;
        }

        if(gamepad2.dpad_down){
            robot.upperSupper.setPosition(0);
        } else{
            robot.upperSupper.setPosition(1);
        }

//-         Turns the robot at increments to easily score the powershots
            if (gamepad1.triangle && !turned){
                method.PIDRotate(sensors.getZAngle()+5,.5);
                turned = true;
            }
            else if(gamepad1.triangle && turned){
                method.PIDRotate(sensors.getZAngle()+5, .5);
            }//end of endgame turning
            else if (gamepad1.right_bumper){
                method.PIDRotate(0 - autoZValue,.5);
            }


//-     this moves the indexer so the robot shoots a ring
        if(gamepad1.cross){
            robot.indexer.setPosition(.4); //-shoot position
        }
        else {
            robot.indexer.setPosition(.55); //-rest position
        }

//-     these cover all the positions for the hopper
        if (gamepad1.square){
            robot.hopper.setPosition(robot.SHOOT_POSITION); //-Shooting ring Position
        }


        /*
         - Control the shooter
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
                method.shooterSpeed(3700, 3500);
                robot.blinkinLedDriver.setPattern(teleOpRPM);
            } else if (endGameShoot){
                robot.blinkinLedDriver.setPattern(endGameRPM);
                method.shooterSpeed(3300, 3200);
            }
            else {
                method.shooterSpeed(0,0);
                robot.blinkinLedDriver.setPattern(atRest);
            }
//        if (gamepad2.square) {
//            shooter = true;
//            shooter2 = true;
//        } else if (gamepad2.triangle) {
//            shooter = false;
//            shooter2 = false;
//            endGameShoot = false;
//            endGameShoot2 = false;
//        }
//        else if (gamepad2.touchpad){
//            shooter = false;
//            shooter2 = false;
//            endGameShoot= true;
//            endGameShoot2 = true;
//        }
        currentTick = robot.shootyMcShootShoot.getCurrentPosition();
        currentTick2 = robot.shooter2.getCurrentPosition();
        currentTime = runtime.time();
        currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
        currentRPM2 = method.calcRPM(lastTick2, lastTime, currentTick2, currentTime);
        lastTick = currentTick;
        lastTick2 = currentTick2;
        lastTime = currentTime;
//        if (shooter) {
//            shooterPower = method.shooterPower(shooterPower, currentRPM, 3600);
//            robot.blinkinLedDriver.setPattern(defaultPattern);
//        } else if(endGameShoot){
//            robot.blinkinLedDriver.setPattern(endGameRPM);
//            shooterPower = method.shooterPower(shooterPower, currentRPM, 3300);
//        }
//        else {
//            shooterPower = 0;
//        }

//-     Setting the RPM for the Second Shooter Motor
//        if (shooter2) {
//            shooterPower2 = method.shooterPower(shooterPower2, currentRPM2, 3500);
//        } else if (endGameShoot2){
//            shooterPower2 = method.shooterPower(shooterPower2, currentRPM2, 3200);
//        }
//        else {
//            shooterPower2 = 0;
//        }
//        robot.shootyMcShootShoot.setPower(shooterPower);
//        robot.shooter2.setPower(shooterPower2);

        if (gamepad1.left_bumper){
            robot.intake.setPower(1);
            robot.hopper.setPosition(robot.INTAKE_POSITION);
        }
        else if (LeftTrigger()){
            robot.intake.setPower(-1);
        }
        else {
            robot.intake.setPower(0);
        }

        double frontRightPower;
        double backRightPower;
        double frontLeftPower;
        double backLeftPower;

        double strafe = gamepad1.left_stick_x;
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        frontLeftPower = Range.clip(drive + turn - strafe, -1, 1);
        backLeftPower = Range.clip(drive + turn + strafe, -1, 1);
        backRightPower = Range.clip(drive - turn - strafe, -1, 1);
        frontRightPower = Range.clip(drive - turn + strafe, -1, 1);

        double liftPower;
        liftPower = Range.clip(-gamepad2.right_trigger + gamepad2.left_trigger,-1.0,1.0);

        if (LeftTrigger2() && robot.topSwitch.getState()){ //-down on lift
            robot.wobbleLift.setPower(liftPower);
        }
        else if (RightTrigger() && robot.bottomSwitch.getState()){ //-up on lift
            robot.wobbleLift.setPower(liftPower);
        }
        else {
            robot.wobbleLift.setPower(0);
        }


        if (gamepad2.dpad_down){
            robot.frontLeft.setPower(-frontLeftPower/2);
            robot.backLeft.setPower(-backLeftPower/2);
            robot.frontRight.setPower(-frontRightPower/2);
            robot.backRight.setPower(-backRightPower/2);
        }
        else{
            robot.frontLeft.setPower(-frontLeftPower);
            robot.backLeft.setPower(-backLeftPower);
            robot.frontRight.setPower(-frontRightPower);
            robot.backRight.setPower(-backRightPower);
        }

        if (gamepad2.cross){
            robot.wobbleGrab.setPosition(robot.CLOSED); //-closed
        }
        else if (gamepad2.circle){
            robot.wobbleGrab.setPosition(robot.OPEN); //-open
        }

        telemetry.addData("AutoZValue", autoZValue);
        telemetry.addData("ZAngle: ", sensors.getZAngle());
        telemetry.addData("Shooter RPM = ", currentRPM);
        telemetry.addData("Shooter RPM2 = ", currentRPM2);
        telemetry.addData("Current Tick 2", currentTick2);
        telemetry.update();
        }
    }
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
