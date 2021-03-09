//package org.firstinspires.ftc.teamcode.Testing;
//
//
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import org.firstinspires.ftc.teamcode.Autonomous.BlueWobble;
//import org.firstinspires.ftc.teamcode.Autonomous.Wobble2;
//import org.firstinspires.ftc.teamcode.SubSystems.*;
//
//@Disabled
//@Autonomous(name = "DoubleWobble")
//
//public class DoubleWobble extends OpMode {
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
//    private boolean positionA;
//    private boolean positionB;
//    private boolean positionC;
//    int i=0;
//
//    private RevBlinkinLedDriver.BlinkinPattern celebrate;
//
//    ElapsedTime runtime = new ElapsedTime();
//
//    enum State {
//        Stop, PathC, PathB, PathA,  PullOutA, LaunchLineB,
//        PullOut2B, PullOutB, DeWobbleB, ParkA, PullOutC,
//        ShootA, ShootB, ShootC, StrafeLeftA, DeliverWobbleB,
//        CenterC, StrafeRightB, AimA, AimB, AimC, StrafeRightB2,
//        wobbleNo2A, StrafeRightC, wobbleNo2C, WobbleyoinkC,
//        wobbleNo2B, WobbleyoinkB, StrafeLeftA3, ParkC, ParkB,
//        Determine, WobbleyoinkA, Wobbleplace, TurnA, Wobble2Ongoing, TurnA2, Vision,
//    }
//
//    State state;
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
//
//        }
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
//        telemetry.addData("Shooter RPM = ", currentRPM);
//        telemetry.addData("Shooter RPM2 = ", currentRPM2);
//        telemetry.addData("Drive Power = ", robot.backLeft.getPower());
//        telemetry.addData("Hopper Position = ", sensors.SHOOT_POSITION);
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
//
//            switch (state) {
//
//                case Vision:
//                    robot.wobbleGrab.setPosition(.5);
//                    if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.FOUR && CurrentTime >= 2) {
//                        Reset();
//                        state = State.PathC;
//                    } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.ONE && CurrentTime >=2) {
//                        Reset();
//                        state = State.PathB;
//                    } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.NONE && CurrentTime >= 2) {
//                        Reset();
//                        state = State.PathA;
//                    }
//                    break;
//
//                //Start of PATH A (No rings)
//                case PathA:
//                    robot.wobbleGrab.setPosition(.5);
//                    method.Reverse(.5);
//                    if (method.DriveDone(56)) {
//                        robot.wobbleGrab.setPosition(.65);
//                        Reset();
//                        state = State.PullOutA;
//                    }
//                    break;
//
//                case PullOutA:
//                    method.Forward(.5);
//                    if (method.DriveDone(8)) {
//                        Reset();
//                        state = State.AimA;
//                    }
//                    break;
//
//                case AimA:
//                    method.TurnAbsolute(14, sensors.getZAngle(), -.75, .75);
//                    if ((method.TurnDone(14) && CurrentTime >= 2)) {
//                        Reset();
//                        state = State.ShootA;
//                    }
//                    break;
//
//                case ShootA:
//                    if (currentRPM >= 3500 && CurrentTime >=1) {
//                        robot.indexer.setPosition(.4);
//                        method.sleep(700);
//                        robot.indexer.setPosition(.55);
//                        method.sleep(700);
//                        i++;
//                        if (i == 5) {
//                            state = State.TurnA;
//                            Reset();
//                        }
//                    }
//                    break;
//
//                case TurnA:
//                    method.TurnAbsolute(90, sensors.getZAngle(), -.75, .75);
//                    if (method.TurnDone(90) && CurrentTime >= 1){
//                        Reset();
//                        state = State.Wobble2Ongoing;
//                    }
//                    break;
//
//                case Wobble2Ongoing:
//                    method.Forward(.5);
//                    if (method.DriveDone(29 )){
//                        Reset();
//                        state = State.TurnA2;
//                    }
//                    break;
//
//                case TurnA2:
//                method.TurnAbsolute(0, sensors.getZAngle(), -.75, .75);
//                if (method.TurnDone(0) && CurrentTime >= 1){
//                    Reset();
//                    state = State.wobbleNo2A;
//                }
//                break;
//
//                case wobbleNo2A:
//                    method.Reverse(.15);
//                    if (method.DriveDone(25)) {
//                        robot.wobbleGrab.setPosition(0);
//                        method.sleep(800);
//                        Reset();
//                        state = State.WobbleyoinkA;
//                    }
//                    break;
//
//                case WobbleyoinkA:
//                    method.Forward(.5);
//                    if (method.DriveDone(52)) {
//                        Reset();
//                        state = State.StrafeLeftA3;
//                    }
//                    break;
//
//                case StrafeLeftA3:
//                    method.TurnAbsolute(90, sensors.getZAngle(), -.75, .75);
//                    if (method.TurnDone(90)) {
//                        Reset();
//                        state = State.Wobbleplace;
//                    }
//                    break;
//
//                case Wobbleplace:
//                    method.Reverse(.5);
//                    if (method.DriveDone(20)) {
//                        robot.wobbleGrab.setPosition(.65);
//                        method.sleep(700);
//                        Reset();
//                        state = State.ParkA;
//                    }
//                    break;
//
//                case ParkA:
//                    method.Forward(.5);
//                    robot.blinkinLedDriver.setPattern(celebrate);
//                    if (method.DriveDone(12)) {
//                        Reset();
//                        state = State.Stop;
//                    }
//                    break;
//
//
//                /*Start of PATH B (one ring)*/
//                case PathB: //forward next to the box
//                    robot.wobbleGrab.setPosition(.5);
//                    method.Reverse(.5);
//                    if (method.DriveDone(80)) {
//                        Reset();
//                        state = State.DeWobbleB;
//                    }
//                    break;
//
//                case DeWobbleB:
//                    method.TurnAbsolute(-135, sensors.getZAngle(), -.65, .65);
//                    if (method.TurnDone(-135) && CurrentTime >= .5) {
//                        Reset();
//                        state = State.DeliverWobbleB;
//                    }
//                    break;
//
//                case DeliverWobbleB:
//                    method.Reverse(.5);
//                    if (method.DriveDone(10)) {
//                        robot.wobbleGrab.setPosition(.65);
//                        Reset();
//                        state = State.PullOutB;
//                    }
//                    break;
//
//                case PullOutB:
//                    method.Forward(.5);
//                    if (method.DriveDone(8)) {
//                        Reset();
//                        state = State.PullOut2B;
//                    }
//                    break;
//
//                case PullOut2B:
//                    method.TurnAbsolute(0, sensors.getZAngle(), -.75, .75);
//                    if (method.TurnDone(0) && CurrentTime >= 1) {
//                        Reset();
//                        state = State.LaunchLineB;
//                    }
//                    break;
//
//                case LaunchLineB:
//                    method.Reverse(.5);
//                    if (method.DriveDone(25)) {
//                        Reset();
//                        state = State.StrafeRightB;
//                    }
//                    break;
//
//                case StrafeRightB:
//                    method.StrafeRight(.5);
//                    if (method.StrafeDone(16)) {
//                        Reset();
//                        state = State.AimB;
//                    }
//                    break;
//
//                case AimB:
//                    method.TurnAbsolute(0, sensors.getZAngle(), -.75, .75);
//                    if ((method.TurnDone(0) && CurrentTime >= .5)) {
//                        Reset();
//                        state = State.ShootB;
//                    }
//                    break;
//
//                case ShootB:
//                    if (currentRPM >= 3500 && CurrentTime >=1) {
//                        robot.indexer.setPosition(.4);
//                        method.sleep(700);
//                        robot.indexer.setPosition(.55);
//                        method.sleep(700);
//                        i++;
//                        if (i == 5) {
//                            state = State.StrafeRightB2;
//                            Reset();
//                        }
//                    }
//                    break;
//
//                case StrafeRightB2:
//                    method.StrafeRight(.5);
//                    if (method.StrafeDone(12)) {
//                        Reset();
//                        state = State.wobbleNo2B;
//                    }
//                    break;
//
//                case wobbleNo2B:
//                    method.Reverse(.5);
//                    if (method.DriveDone(50)) {
//                        Reset();
//                        state = State.WobbleyoinkB;
//                    }
//                    break;
//
//                case WobbleyoinkB:
//                    robot.wobbleGrab.setPosition(0);
//                    method.Forward(.5);
//                    if (method.DriveDone(80)) {
//                        Reset();
//                        state = State.ParkB;
//                    }
//
//                case ParkB:
//                    method.Forward(.5);
//                    if (method.DriveDone(14)) {
//                        Reset();
//                        state = State.Stop;
//                    }
//                    break;
//
//
//                /*Start of Path C (four rings)*/
//                case PathC: //forward next to the box
//                    robot.wobbleGrab.setPosition(.5);
//                    method.Reverse(.5);
//                    if (method.DriveDone(112)) {
//                        robot.wobbleGrab.setPosition(.65);
//                        Reset();
//                        state = State.PullOutC;
//                    }
//                    break;
//
//                case PullOutC:
//                    method.Forward(.5);
//                    if (method.DriveDone(50)) {
//                        Reset();
//                        state = State.CenterC;
//                    }
//                    break;
//
//                case CenterC:
//                    method.StrafeLeft(.5);
//                    if (method.StrafeDone(25)) {
//                        Reset();
//                        state = State.AimC;
//                    }
//                    break;
//
//                case AimC:
//                    method.TurnAbsolute(0, sensors.getZAngle(), -.75, .75);
//                    if ((method.TurnDone(0) && CurrentTime >= 1) || CurrentTime >= 2) {
//                        Reset();
//                        state = State.ShootC;
//                    }
//                    break;
//
//                case ShootC:
//                    if (currentRPM >= 3500 && CurrentTime >=1) {
//                        robot.indexer.setPosition(.4);
//                        method.sleep(700);
//                        robot.indexer.setPosition(.55);
//                        method.sleep(700);
//                        i++;
//                        if (i == 5) {
//                            state = State.StrafeRightC;
//                            Reset();
//                        }
//                    }
//                    break;
//
//                case StrafeRightC:
//                    method.StrafeRight(.5);
//                    if (method.StrafeDone(12)) {
//                        Reset();
//                        state = State.wobbleNo2C;
//                    }
//
//                case wobbleNo2C:
//                    method.Reverse(.5);
//                    if (method.DriveDone(50)) {
//                        Reset();
//                        state = State.WobbleyoinkC;
//                    }
//                    break;
//
//                case WobbleyoinkC:
//                    robot.wobbleGrab.setPosition(0);
//                    method.Forward(.5);
//                    if (method.DriveDone(80)) {
//                        Reset();
//                        state = State.ParkC;
//                    }
//                    break;
//
//                case ParkC:
//                    method.Forward(.5);
//                    if (method.DriveDone(14)) {
//                        Reset();
//                        state = State.Stop;
//                    }
//                    break;
//
//
//                //these cases are for all 3 paths
//                case Stop:
//                    robot.intakeServo.setPosition(0);
//                    Reset();
//                    method.Death();
//                    break;
//            }
//        }
//
//    private void Reset () {
//        time.reset();
//        method.Kill();
//
//        }
//    }