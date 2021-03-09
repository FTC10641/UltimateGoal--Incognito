package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

/*
? Red is needs to be tested
- Green is notes about the code
+ Orange is just info
*/

//@Disabled
@TeleOp(name="Testing/TroubleShoot", group="Linear Opmode")
public class TeleTest extends LinearOpMode {
    HardWareMap robot = new HardWareMap();
    Methods method = new Methods();
    Sensors sensors = new Sensors();
    ElapsedTime time = new ElapsedTime();


    boolean turned;

    private boolean shooter, shooter2, shooter3 = false, endGameShoot, endGameShoot2;
    private double currentTick, currentTick2;
    private double currentRPM, currentRPM2;
    private double lastTick = 0, lastTick2 = 0;
    private double shooterPower, shooterPower2;
    private double currentTime;
    private double lastTime;

    File autonomousGyroFile = AppUtil.getInstance().getSettingsFile("autonomousGyroFile.txt");
    double autoZValue = 0;

    private RevBlinkinLedDriver.BlinkinPattern endGameRPM, defaultPattern;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.initHardware(hardwareMap);
        sensors.initSensors(hardwareMap);
        method.initMethods(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        endGameRPM = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        defaultPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;

        waitForStart();
        while (opModeIsActive()) {
//            robot.intakeServo.setPosition(0); //dropping the intake arm

            if (gamepad2.dpad_down){
                robot.intakeServo.setPosition(0);
            }

            /**
            + tries to get the value of the ZValue from the end of auto
            + catches if unable to read and automatically sets to 0
            */
            try {
                autoZValue = Double.parseDouble(ReadWriteFile.readFile(autonomousGyroFile).trim());
            } catch (Exception e) {
                autoZValue = 0;
            }

//-         Tele Endgame Shots
            if (gamepad1.triangle && !turned){
                method.PIDRotate(sensors.getZAngle()+5,.5);
                turned = true;
            }
            else if(gamepad1.triangle && turned){
                method.PIDRotate(sensors.getZAngle()+4, .5);
            }//end of endgame turning
            else if (gamepad1.right_bumper){
                method.PIDRotate(0 - autoZValue,.5);
            }


//-         this moves the indexer so the robot shoots a ring
            if(gamepad1.cross){
                robot.indexer.setPosition(.4); //shoot position
            }
            else {
                robot.indexer.setPosition(.55); //rest position
            }


            /*
             - Controls the shooter
             */
            if (gamepad2.square){
                shooter3 = true;
            } else if(gamepad2.triangle){
                shooter3 = false;
            }
            if (shooter3){
                method.shooterSpeed(3600, 3400);
            } else {
                robot.shootyMcShootShoot.setPower(0);
                robot.shootyMcShootShoot.setPower(0);
            }

//            if (gamepad2.dpad_right) {
//                shooter = true;
//                shooter2 = true;
//            } else if (gamepad2.triangle) {
//                shooter = false;
//                shooter2 = false;
//                endGameShoot = false;
//                endGameShoot2 = false;
//            }
//            else if (gamepad2.touchpad){
//                shooter = false;
//                shooter2 = false;
//                endGameShoot= true;
//                endGameShoot2 = true;
//            }
            currentTick = robot.shootyMcShootShoot.getCurrentPosition();
            currentTick2 = robot.shooter2.getCurrentPosition();
            currentTime = runtime.time();
            currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
            currentRPM2 = method.calcRPM(lastTick2, lastTime, currentTick2, currentTime);
            lastTick = currentTick;
            lastTick2 = currentTick2;
            lastTime = currentTime;
//            if (shooter) {
//                shooterPower = method.shooterPower(shooterPower, currentRPM, 3750);
//                robot.blinkinLedDriver.setPattern(defaultPattern);
//            } else if(endGameShoot){
//                robot.blinkinLedDriver.setPattern(endGameRPM);
//                shooterPower = method.shooterPower(shooterPower, currentRPM, 3500);
//            }
//            else {
//                shooterPower = 0;
//            }
//
//            if (shooter2) {
//                shooterPower2 = method.shooterPower(shooterPower2, currentRPM2, 3500);
//            } else if (endGameShoot2){
//                shooterPower2 = method.shooterPower(shooterPower, currentRPM, 3250);
//            }
//            else {
//                shooterPower2 = 0;
//            }
//            robot.shootyMcShootShoot.setPower(shooterPower);
//            robot.shooter2.setPower(shooterPower2);


            if (gamepad1.left_bumper){ //turns on intake motor and sets hoppers to load position
                robot.intake.setPower(1);
                robot.hopper.setPosition(0);
            }
            else if (LeftTrigger()){ //Turns on the out-taking in case rings get stuck
                robot.intake.setPower(-1);
            } else { //sets intake power 0
                robot.intake.setPower(0);
            }

            if (gamepad1.square){//sets hopper in shoot position
                robot.hopper.setPosition(robot.SHOOT_POSITION);
            }


            /* These are for testing*/
            if(gamepad2.dpad_down){
                robot.hopper.setPosition(0);//- Intake position
            }
            else if(gamepad2.dpad_left){
                robot.hopper.setPosition(.49);//- Shoot position
            }


            if (gamepad2.cross){
                robot.wobbleGrab.setPosition(robot.CLOSED); //- Closed Position
            }
            else if (gamepad2.circle){
                robot.wobbleGrab.setPosition(robot.OPEN); //- Open Position
            }


            double liftPower;
            liftPower = Range.clip(-gamepad2.right_trigger + gamepad2.left_trigger,-1.0,1.0);

            if (LeftTrigger2() && robot.topSwitch.getState()){ //- Will Move Lift Down
                robot.wobbleLift.setPower(liftPower);
            }
            else if (RightTrigger() && robot.bottomSwitch.getState()){ //- Will Move Lift Up
                robot.wobbleLift.setPower(liftPower);
            }
            else {
                robot.wobbleLift.setPower(0);
            }


            telemetry.addData("AutoZValue", autoZValue); //- Will return the ZValue from the end of autonomous

            telemetry.addData("ZAngle: ", sensors.getZAngle());//- Will return current ZValue
            telemetry.addData("Shooter RPM = ", currentRPM);//- Will Return RPM of shooter motor 1
            telemetry.addData("Shooter RPM2 = ", currentRPM2);//- Will Return RPM of shooter motor 1

//            telemetry.addData("Vertical Left Tick = ", robot.verticalLeft.getCurrentPosition());
//            telemetry.addData("Vertical Right Tick = ", robot.verticalRight.getCurrentPosition());
//            telemetry.addData("Horizontal Tick = ", robot.horizontal.getCurrentPosition());
//
//            telemetry.addData("Forward Inches AVG = ", (robot.verticalRight.getCurrentPosition()+robot.verticalLeft.getCurrentPosition())/2/robot.COUNTS_PER_INCH);
//            telemetry.addData("Forward Inches = ", robot.verticalRight.getCurrentPosition()/robot.COUNTS_PER_INCH);
//
//            telemetry.addData("Horiz Inches = ", robot.horizontal.getCurrentPosition() / robot.COUNTS_PER_INCH);
//
//            telemetry.addData("Shooter1 Velocity = ",(robot.shootyMcShootShoot.getVelocity()) / 28 *60);
//            telemetry.addData("Shooter2 Velocity = ", (robot.shooter2.getVelocity()) / 28 *60);
            telemetry.addData("Current Tick", currentTick); //- Shooter Encoder val
            telemetry.addData("Current Tick 2", currentTick2); //- Shooter2 Encoder val
            telemetry.addData("Shooter Power = ", shooterPower);//- Returns Current Power for shooter motor one
            telemetry.addData("Shooter Power2 = ", shooterPower2);//- Returns Current Power for shooter motor two
            telemetry.update();
        }
    }

    //- These are to check if the Trigger specified is being pressed but it does not matter how much it is getting pressed
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