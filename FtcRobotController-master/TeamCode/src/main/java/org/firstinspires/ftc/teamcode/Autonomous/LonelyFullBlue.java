package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.SubSystems.*;

/*
? Red is needs to be tested
- Green is notes about the code
+ Orange is just info
*/

/*
 + Path A: Double Wobble, 3 high goals, and park. 71 pts
 + Path B: Double Wobble, 4 high goals, and park. 83 pts
 + Path C: One Wobble, 7 high goals, and park. 104 pts
 */

@Autonomous(name = "Lonely Blue")

public class LonelyFullBlue extends OpMode {
    HardWareMap robot = new HardWareMap();
    Vision vision = new Vision();
    ElapsedTime time = new ElapsedTime();
    Sensors sensors = new Sensors();
    Methods method = new Methods();

    private double currentTick, currentTick2;
    private double currentRPM, currentRPM2;
    private double lastTick, lastTick2;
    private double currentTime;
    private double lastTime;

    private boolean pathA, pathB, pathC;
    private boolean shoot2B = false, shoot2C = false, shoot3C = false, secondRun = false;
    int i = 0;

    private RevBlinkinLedDriver.BlinkinPattern celebrate, aPath, bPath, cPath;

    ElapsedTime runtime = new ElapsedTime();

    enum State {
        Stop, PathC, PathB, PathA, Vision,  PullOutA, LaunchLineB,
        PullOut2B, PullOutB, DeWobbleB, PullOutC, Shoot,  StrafeLeftA,
        StrafeRightA, TurnA2, StrafeLeftA3, BackUpB2, Aim,
        WobbleYoinkA, WobbleNo2A, ForwardToShoot, WobblePlaceA,
        ParkB, StrafeRightB2, WobbleYoinkB, Deliver2Wobble,
        Drop2Wobble, Wobble2B, PickUpWobble, AimToRing, PickUpWobble2,
        ParkA, WobbleYoinkC, ParkC,  IntakeRing2C, StrafeLeftC,
        GoAroundRingC, IntakeRing3C, IntakeRing4C, BackUpC, FaceRingC,
        StraightenC, DropWobbleC, FaceRing2C, BackUp2C, AimC, Aim2C,
        AimC3, StraightenC2, AimB, ForwardToShootB, AimB2, StraightenB3,
        AimB3, DropWobble2B, Deliver2WobbleB, StrafeSmallB, FaceTargetB
    }

    State state;


    @Override
    public void init() {
        robot.initHardware(hardwareMap);
        vision.initVision(hardwareMap);
        sensors.initSensors(hardwareMap);
        method.initMethods(hardwareMap);

        robot.hopper.setPosition(robot.SHOOT_POSITION);
        robot.wobbleGrab.setPosition(robot.OPEN);

        time.reset();
        state = State.Vision;
    }

    @Override
    public void start(){
        Reset();
    }

    @Override
    public void loop() {
        double CurrentTime = time.time();
        telemetry.addLine("PathA = RED, PathB = YELLOW, PathC = BLUE");
        telemetry.addData("time", time.time());
        telemetry.addData("ZAngle", sensors.getZAngle());
        telemetry.addData("Current State",state.toString());
        telemetry.addData("Current Position", vision.pipeline.position);
        telemetry.addData("Shooter RPM = ", currentRPM);
        telemetry.addData("Shooter RPM2 = ", currentRPM2);
        telemetry.update();

        celebrate = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        aPath = RevBlinkinLedDriver.BlinkinPattern.RED;
        bPath = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        cPath = RevBlinkinLedDriver.BlinkinPattern.YELLOW;


        currentTick = robot.shootyMcShootShoot.getCurrentPosition();
        currentTick2 = robot.shooter2.getCurrentPosition();
        currentTime = runtime.time();
        currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
        currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
        currentRPM2 = method.calcRPM(lastTick2, lastTime, currentTick2, currentTime);
        lastTick = currentTick;
        lastTick2 = currentTick2;
        lastTime = currentTime;

        method.shooterSpeed(3700, 3500);

        switch (state) {

            case Vision:
                robot.wobbleGrab.setPosition(robot.CLOSED);
                if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.FOUR && CurrentTime >= 1) {
                    pathC = true;
                    Reset();
                    state = State.PathC;
                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.ONE && CurrentTime >=2) {
                    pathB = true;
                    Reset();
                    state = State.PathB;
                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.NONE && CurrentTime >= 2) {
                    pathA = true;
                    Reset();
                    state = State.PathA;
                }
                break;


//-         Start of PATH A part one(No rings)
            case PathA:
                robot.blinkinLedDriver.setPattern(aPath);
                robot.wobbleGrab.setPosition(robot.CLOSED);
                method.Reverse(.5, 58);

                if (method.DriveDone(58)) {
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    Reset();
                    state = State.PullOutA;
                }
                break;

            case PullOutA:
                method.Forward(.5, 11);
                if (method.DriveDone(11)){
                    Reset();
                    state = State.StrafeLeftA;
                }
                break;

            case StrafeLeftA:
                method.StrafeLeft(.2,34);
                if (method.StrafeDone(34)){
                    Reset();
                    state = State.Aim;
                }
                break;


//-         Start of PATH B part one(one ring)
            case PathB: //forward next to the box
                robot.blinkinLedDriver.setPattern(bPath);
                robot.wobbleGrab.setPosition(robot.CLOSED);
                method.Reverse(.65, 53);
                if (method.DriveDone(53)) {
                    Reset();
                    state = State.StrafeSmallB;
                }
                break;

            case StrafeSmallB:
                method.StrafeLeft(.3, 10);
                if (method.StrafeDone(10)){
                    Reset();
                    state = State.DeWobbleB;
                }
                break;

            case DeWobbleB:
                method.TurnAbsolute(-155, sensors.getZAngle(), -.75,.75);
                if ((method.TurnDone(-155) && CurrentTime >= 1) || CurrentTime >= 1.5){
                    Reset();
                    state = State.PullOutB;
                }
                break;

            case PullOutB:
                method.Reverse(.5, 24);
                if (method.DriveDone(24)){
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(500);
                    Reset();
                    state = State.PullOut2B;
                }
                break;

            case PullOut2B:
                method.Forward(.5,12);
                if (method.DriveDone(12)){
                    Reset();
                    state = State.StraightenB3;
                }
                break;

            case StraightenB3:
                method.TurnAbsolute(0, sensors.getZAngle(), -.75,.75);
                if ((method.TurnDone(0) && CurrentTime >= 1) || CurrentTime >= 1.5){
                    Reset();
                    state = State.LaunchLineB;
                }
                break;

            case LaunchLineB:
                method.Reverse(.5,10);
                if (method.DriveDone(10)){
                    Reset();
                    state = State.AimB2;
                }
                break;

            case AimB2:
                method.TurnAbsolute(14, sensors.getZAngle(), -.75,.75);
                if ((method.TurnDone(14) && CurrentTime >= 1) || CurrentTime >= 1.5){
                    Reset();
                    state = State.Shoot;
                }
                break;

//-         Start of Path C part one(four rings)
            case PathC: //forward next to the box
                robot.blinkinLedDriver.setPattern(cPath);
                robot.wobbleGrab.setPosition(robot.CLOSED);
                method.Reverse(.6, 108);
                if (method.DriveDone(108)) {
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(500);
                    Reset();
                    state = State.DropWobbleC;
                }
                break;

            case DropWobbleC:
                method.Forward(.6, 10);
                if (method.DriveDone(10)){
                    Reset();
                    state = State.StraightenC;
                }
                break;

            case StraightenC:
                method.TurnAbsolute(-28, sensors.getZAngle(), .75,-.75);
                if (method.TurnDone(-28) && CurrentTime >= 2){
                    Reset();
                    state = State.PullOutC;
                }
                break;

            case PullOutC:
                method.Reverse(.5, 42);
                if (method.DriveDone(42)){
                    Reset();
                    state = State.AimC3;
                }
                break;

            case AimC3:
                method.TurnAbsolute(5, sensors.getZAngle(), -.65,.65);
                i=0;
                if (method.TurnDone(5) && CurrentTime >= 1.5){
                    Reset();
                    robot.hopper.setPosition(robot.SHOOT_POSITION);
                    method.sleep(400);
                    state = State.Shoot;
                }
                break;


//-         these cases are for all 3 paths
            case Aim:
                method.TurnAbsolute(1, sensors.getZAngle(), -.65,.65);
                i=0;
                if (method.TurnDone(1) && CurrentTime >= 2){
                    Reset();
                    if (pathA) {
                        state = State.ForwardToShoot;
                    } else {
                        robot.hopper.setPosition(robot.SHOOT_POSITION);
                        method.sleep(400);
                        state = State.Shoot;
                    }
                }
                break;

            case ForwardToShoot:
                method.Forward(.5,6);
                if (method.DriveDone(6)){
                    Reset();
                    state = State.Shoot;
                }
                break;

            case Shoot:
                robot.hopper.setPosition(robot.SHOOT_POSITION);
                if (currentRPM >= 3200) {
                    robot.indexer.setPosition(.4);
                    method.sleep(100);
                    robot.indexer.setPosition(.55);
                    method.sleep(100);
                    i++;
                    if (i == 5 && pathA) {
                        i = 0;
                        Reset();
                        state = State.StrafeRightA;
                    } else if (i == 5 && pathB){
                        i = 0;
                        Reset();
                        state = State.StrafeRightB2;
                    } else if (i == 5 && pathC){
                        i = 0;
                        Reset();
                        state = State.StraightenC2;
                    } else if(i == 2 && shoot2B){
                        i = 0;
                        Reset();
                        state = State.PickUpWobble2;
                    } else if (i == 4 && shoot2C){
                        i = 0;
                        Reset();
                        robot.hopper.setPosition(0);
                        state = State.BackUpC;
                    } else if (i == 5 && shoot3C){
                        i = 0;
                        Reset();
                        robot.hopper.setPosition(0);
                        state = State.ParkC;
                    }
                }
                break;

            case StrafeRightA:
                method.StrafeRight(.3, 8);
                if (method.StrafeDone(8)){
                    Reset();
                    state = State.TurnA2;
                }
                break;

            case TurnA2:
                method.TurnAbsolute(0, sensors.getZAngle(), -.75, .75);
                if (method.TurnDone(0) && CurrentTime >= 1){
                    Reset();
                    state = State.WobbleNo2A;
                }
                break;

            case WobbleNo2A:
                method.Reverse(.15, 31);
                if (method.DriveDone(31)) {
                    robot.wobbleGrab.setPosition(robot.CLOSED);
                    method.sleep(800);
                    Reset();
                    state = State.WobbleYoinkA;
                }
                break;

            case WobbleYoinkA:
                method.Forward(.5, 54);
                if (method.DriveDone(54)) {
                    Reset();
                    state = State.StrafeLeftA3;
                }
                break;

            case StrafeLeftA3:
                method.TurnAbsolute(90, sensors.getZAngle(), -.75, .75);
                if (method.TurnDone(90)) {
                    Reset();
                    state = State.WobblePlaceA;
                }
                break;

            case WobblePlaceA:
                method.Reverse(.5, 17);
                if (method.DriveDone(17)) {
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(700);
                    Reset();
                    state = State.ParkA;
                }
                break;

            case ParkA:
                method.Forward(.5, 12);
                if (method.DriveDone(12)){
                    Reset();
                    state = State.Stop;
                }
                break;
            //- End of Path A Part2


            //- Start of Path B Part2
            case StrafeRightB2:
                method.Reverse(.5, 28);
                if (method.DriveDone(28)) {
                    Reset();
                    state = State.Wobble2B;
                }
                break;

            case Wobble2B:
                method.TurnAbsolute(48, sensors.getZAngle(), .75,-75);
                if (method.TurnDone(48)){
                    robot.hopper.setPosition(0);
                    robot.intakeServo.setPosition(0);
                    Reset();
                    state = State.PickUpWobble;
                }
                break;

            case PickUpWobble:
                robot.intake.setPower(1);
                method.Forward(.5, 25);
                if (method.DriveDone(25)){
                    Reset();
                    state = State.AimB3;
                }
                break;

            case AimB3:
                method.TurnAbsolute(-11, sensors.getZAngle(), .75,-75);
                if (method.TurnDone(-11)){
                    Reset();
                    robot.hopper.setPosition(robot.SHOOT_POSITION);
                    state = State.PickUpWobble2;
                }
                break;

            case PickUpWobble2:
                method.Reverse(.2, 24);
                if (method.DriveDone(24)){
                    robot.wobbleGrab.setPosition(robot.CLOSED);
                    method.sleep(600);
                    Reset();
                    state = State.FaceTargetB;
                }
                break;

            case FaceTargetB:
                method.TurnAbsolute(14, sensors.getZAngle(), .75,-75);
                if (method.TurnDone(14)){
                    Reset();
                    state = State.Deliver2WobbleB;
                }
                break;

            case Deliver2WobbleB:
                method.Forward(.5, 60);
                if (method.DriveDone(60)){
                    Reset();
                    state = State.DropWobble2B;
                }
                break;

            case DropWobble2B:
                method.TurnAbsolute(140,sensors.getZAngle(),.5,-.5);
                if (method.TurnDone(140) && CurrentTime >= 1.5){
                    Reset();
                    state = State.BackUpB2;
                }
                break;

            case BackUpB2:
                method.Reverse(.5, 10);
                if (method.DriveDone(10)){
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(500);
                    Reset();
                    state = State.ParkB;
                }
                break;

            case AimToRing:
                method.TurnAbsolute(-34,sensors.getZAngle(),.75,-.75);
                if (method.TurnDone(-34)){
                    Reset();
                    state = State.WobbleYoinkB;
                }
                break;

            case WobbleYoinkB:
                robot.intake.setPower(1);
                robot.hopper.setPosition(0);
                method.Forward(.65, 26);
                if ((method.DriveDone(26) && CurrentTime >= 1.45)) {
                    robot.intake.setPower(0);

                    Reset();
                    state = State.AimB;
                }
                break;

            case AimB:
                method.TurnAbsolute(3, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(3)){
                    Reset();
                    robot.hopper.setPosition(robot.SHOOT_POSITION);
                    method.sleep(400);
                    state = State.ForwardToShootB;
                }
                break;

            case ForwardToShootB:
                method.Forward(.5, 5);
                if (method.DriveDone(5)){
                    Reset();
                    state = State.Shoot;
                }
                break;

            case Deliver2Wobble:
                method.Forward(.5, 34);
                if (method.DriveDone(34)){
                    Reset();
                    state = State.Drop2Wobble;
                }
                break;

            case Drop2Wobble:
                method.TurnAbsolute(134, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(134)){
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(500);
                    Reset();
                    state = State.ParkB;
                }
                break;

            case ParkB:
                method.Forward(.5, 15);
                if (method.DriveDone(15)){
                    Reset();
                    state = State.Stop;
                }
                break;
            //- End of Path B Part2

//-         Start of Path C

            case StraightenC2:
                method.TurnAbsolute(0, sensors.getZAngle(), -.65,.65);
                i=0;
                if (method.TurnDone(0) && CurrentTime >= 2){
                    state = State.StrafeLeftC;
                }
                break;

            case StrafeLeftC:
                method.StrafeLeft(.45, 16);
                if (method.StrafeDone(16)){
                    Reset();
                    state = State.GoAroundRingC;
                }
                break;

            case GoAroundRingC:
                method.Reverse(.5,32);
                robot.intakeServo.setPosition(0);
                if (method.DriveDone(32)){
                    robot.hopper.setPosition(0);
                    Reset();
                    state = State.WobbleYoinkC;
                }
                break;

            case WobbleYoinkC:
                method.StrafeRight(.5, 14);
                if (method.DriveDone(14)) {
                    Reset();
                    state = State.FaceRingC;
                }
                break;

            case FaceRingC:
                method.TurnAbsolute(20, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(20) && CurrentTime >= 1.5){
                    Reset();
                    state = State.IntakeRing2C;
                }
                break;

            case IntakeRing2C:
                robot.intake.setPower(1);
                method.Forward(.5, 20);
                if (method.DriveDone(20)){
                    Reset();
                    pathC = false;
                    shoot2C = true;
                    state = State.AimC;
                }
                break;

            case AimC:
                method.TurnAbsolute(4, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(4) && CurrentTime >= 1.5){
                    robot.hopper.setPosition(robot.SHOOT_POSITION);
                    method.sleep(400);
                    Reset();
                    state = State.Shoot;
                }
                break;

            case BackUpC:
                robot.intake.setPower(-1);
                method.Reverse(.5, 2);
                if (method.DriveDone(2)){
                    Reset();
                    state = State.FaceRing2C;
                }
                break;

            case FaceRing2C:
                method.TurnAbsolute(15, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(15) && CurrentTime >= 1){
                    Reset();
                    state = State.IntakeRing3C;
                }
                break;

            case IntakeRing3C:
                robot.intake.setPower(1);
                method.Forward(.5,10);
                if (method.DriveDone(10) || CurrentTime >= 4){
                    Reset();
                    state = State.IntakeRing4C;
                }
                break;

            case IntakeRing4C:
                robot.intake.setPower(1);
                method.Forward(.5,10);
                if (method.DriveDone(10) || CurrentTime >= 4){
                    Reset();
                    state = State.BackUp2C;
                }
                break;

            case BackUp2C:
                method.Reverse(.5,8);
                if (method.DriveDone(8) || CurrentTime >= 4){
                    Reset();
                    shoot2C = false;
                    shoot3C = true;
                    state = State.Aim2C;
                }
                break;

            case Aim2C:
                method.TurnAbsolute(2, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(2) && CurrentTime >= 1){
                    robot.hopper.setPosition(robot.SHOOT_POSITION);
                    method.sleep(400);
                    Reset();
                    state = State.Shoot;
                }
                break;

            case ParkC:
                method.Forward(1, 22);
                if (method.DriveDone(22)){
                    Reset();
                    state = State.Stop;
                }
                break;


            case Stop:
                method.shooterSpeed(0, 0);
                robot.blinkinLedDriver.setPattern(celebrate);
                robot.intakeServo.setPosition(0);
                Reset();
                method.Death();
                break;
        }
    }
    private void Reset() {
        time.reset();
        method.Kill();
    }
}