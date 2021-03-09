//package org.firstinspires.ftc.teamcode.Autonomous;
//
//
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import org.firstinspires.ftc.teamcode.SubSystems.*;
//
////Wobble Auto from the Second League Meet
//@Disabled
//@Autonomous(name = "Wobble2.0")
//
//public class Wobble2 extends OpMode {
//    HardWareMap robot = new HardWareMap();
//    Vision vision = new Vision();
//    ElapsedTime time = new ElapsedTime();
//    Sensors sensors = new Sensors();
//    Methods method = new Methods();
//
//    private double currentTick, currentTick2;
//    private double currentRPM, currentRPM2;
//    private double lastTick, lastTick2;
//    private double shooterPower, shooterPower2;
//    private double currentTime;
//    private double lastTime;
//
//    private RevBlinkinLedDriver.BlinkinPattern celebrate;
//
//    ElapsedTime runtime = new ElapsedTime();
//
//    enum State {
//        Stop, PathC, PathB, PathA, Vision,  PullOutA, LaunchLineB, PullOut2B,
//        PullOutB, DeWobbleB, Park, PullOutC, Shoot,  StrafeLeftA, DeliverWobbleB,
//        CenterC, StrafeRightB, StrafeLeftC, Aim
//    }
//
//    State state;
//
//
//
//
//    @Override
//    public void init() {
//        robot.initHardware(hardwareMap);
//        vision.initVision(hardwareMap);
//        sensors.initSensors(hardwareMap);
//        method.initMethods(hardwareMap);
//
//        robot.hopper.setPosition(robot.SHOOT_POSITION);
//
//        time.reset();
//        state = State.Vision;
//    }
//
//    @Override
//    public void start(){
//        Reset();
//    }
//
//    @Override
//    public void loop() {
//        double CurrentTime = time.time();
//        telemetry.addData("time", CurrentTime);
//        telemetry.addData("ZAngle", sensors.getZAngle());
//        telemetry.addData("Current State",state.toString());
//        telemetry.addData("Current Position", vision.pipeline.position);
//
//        telemetry.addData("Shooter RPM = ", currentRPM);
//        telemetry.addData("Shooter RPM2 = ", currentRPM2);
//        telemetry.addData("Shooter Power = ", shooterPower);
//        telemetry.addData("Shooter Power2 = ", shooterPower2);
////        telemetry.addData("Current Tick", currentTick);
////        telemetry.addData("Current Tick 2", currentTick2);
//        telemetry.update();
//
//        celebrate = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
//
//        currentTick = robot.shootyMcShootShoot.getCurrentPosition();
//        currentTick2 = robot.shooter2.getCurrentPosition();
//        currentTime = runtime.time();
//        currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
//        currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
//        currentRPM2 = method.calcRPM(lastTick2, lastTime, currentTick2, currentTime);
//        lastTick = currentTick;
//        lastTick2 = currentTick2;
//        lastTime = currentTime;
//        shooterPower = method.shooterPowerForAuto(shooterPower, currentRPM, 3000);
//        shooterPower2 = method.shooterPowerForAuto(shooterPower2, currentRPM2, 2750);
//
//        robot.shootyMcShootShoot.setPower(shooterPower);
//        robot.shooter2.setPower(shooterPower2);
//
//        switch (state) {
//
//            case Vision:
//                robot.wobbleGrab.setPosition(.5);
//                if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.FOUR && CurrentTime >= 2) {
//                    Reset();
//                    state = State.PathC;
//                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.ONE && CurrentTime >=2) {
//                    Reset();
//                    state = State.PathB;
//                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.NONE && CurrentTime >= 2) {
//                    Reset();
//                    state = State.PathA;
//                }
//                break;
//
//
//            //Start of PATH A (No rings)
//            case PathA:
//                robot.wobbleGrab.setPosition(.5);
//                method.Reverse(.65);
//                if (method.DriveDone(56)) {
//                    robot.wobbleGrab.setPosition(.65);
//                    Reset();
//                    state = State.PullOutA;
//                }
//                break;
//
//            case PullOutA:
//                method.Forward(.5);
//                if (method.DriveDone(10)){
//                    Reset();
//                    state = State.StrafeLeftA;
//                }
//                break;
//
//            case StrafeLeftA:
//                method.StrafeLeft(.5);
//                if (method.StrafeDone(30)){
//                    Reset();
//                    state = State.Aim;
//                }
//                break;
//
//
//            /*Start of PATH B (one ring)*/
//            case PathB: //forward next to the box
//                robot.wobbleGrab.setPosition(.5);
//                method.Reverse(.65);
//                if (method.DriveDone(78)) {
//                    Reset();
//                    state = State.DeWobbleB;
//                }
//                break;
//
//            case DeWobbleB:
//                method.StrafeLeft(.5);
//                if (method.StrafeDone(29)){
//                    Reset();
//                    robot.wobbleGrab.setPosition(.65);
//                    state = State.PullOutB;
//                }
//                break;
//
////            case DeliverWobbleB:
////                method.Reverse(.5,8);
////                if (method.DriveDone(8)){
////                    robot.wobbleGrab.setPosition(.65);
////                    Reset();
////                    state = State.PullOutB;
////                }
////                break;
//
//            case PullOutB:
//                method.Forward(.5);
//                if (method.DriveDone(15)){
//                    Reset();
//                    state = State.PullOut2B;
//                }
//                break;
//
//            case PullOut2B:
//                method.TurnAbsolute(0, sensors.getZAngle(), -.75,.75);
//                if ((method.TurnDone(0) && CurrentTime >= 1) || CurrentTime >= 2){
//                    Reset();
//                    state = State.LaunchLineB;
//                }
//                break;
//
//            case LaunchLineB:
//                method.Reverse(.5);
//                if (method.DriveDone(12)){
//                    Reset();
//                    state = State.Aim;
//                }
//                break;
////
////            case StrafeRightB:
////                method.StrafeRight(.5,16);
////                if (method.StrafeDone(16))   {
////                    Reset();
////                    state = State.Aim;
////                }
////                break;
//
//
//            /*Start of Path C (four rings)*/
//            case PathC: //forward next to the box
//                robot.wobbleGrab.setPosition(.5);
//                method.Reverse(.5);
//                if (method.DriveDone(112)) {
//                    robot.wobbleGrab.setPosition(.65);
//                    Reset();
//                    state = State.PullOutC;
//                }
//                break;
//
//            case PullOutC:
//                method.Forward(.5);
//                if (method.DriveDone(55)){
//                    Reset();
//                    state = State.CenterC;
//                }
//                break;
//
//            case CenterC:
//                method.StrafeLeft(.5);
//                if (method.StrafeDone(28)){
//                    Reset();
//                    state = State.Aim;
//                }
//                break;
//
//
//            //these cases are for all 3 paths
//            case Aim:
//                method.TurnAbsolute(0, sensors.getZAngle(), -.75,.75);
//                if (method.TurnDone(0) && CurrentTime >= 4){
//                    Reset();
//                    state = State.Shoot;
//                }
//                break;
//
//            case Shoot:
//                if (currentRPM >= 3500) {
//                    robot.indexer.setPosition(.4);
//                    method.sleep(400);
//                    robot.indexer.setPosition(.55);
//                    method.sleep(400);
//                    if (CurrentTime >= 5) {
//                        state = State.Park;
//                        Reset();
//                    }
//                }
//                break;
//
//            case Park:
//                method.Forward(.5);
//                if (method.DriveDone(14)){
//                    Reset();
//                    state=State.Stop;
//                }
//                break;
//
//            case Stop:
//                robot.blinkinLedDriver.setPattern(celebrate);
//                robot.intakeServo.setPosition(0);
//                Reset();
//                method.Death();
//                break;
//        }
//    }
//    private void Reset() {
//        time.reset();
//        method.Kill();
//    }
//
//}
