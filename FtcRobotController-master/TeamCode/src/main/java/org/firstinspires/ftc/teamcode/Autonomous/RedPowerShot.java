package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.HardWareMap;
import org.firstinspires.ftc.teamcode.SubSystems.Methods;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@Autonomous(name = "RedPowerShot")

public class RedPowerShot extends OpMode {
    HardWareMap robot = new HardWareMap();
    Vision vision = new Vision();
    ElapsedTime time = new ElapsedTime();
    Sensors sensors = new Sensors();
    Methods method = new Methods();

    private double currentTick, currentTick2;
    private double currentRPM, currentRPM2;
    private double lastTick, lastTick2;
    private double shooterPower, shooterPower2;
    private double currentTime;
    private double lastTime;

    private boolean pathA, pathB, pathC;
    private boolean shoot2B = false, shoot2C = false, shoot3C = false, secondRun = false;
    int i = 0;

    private RevBlinkinLedDriver.BlinkinPattern celebrate, APath, BPath, CPath;

    ElapsedTime runtime = new ElapsedTime();

    enum State {
        Vision, Aim, Shoot,  Stop,

        PathA, TurnA, DeliverWobA,
        PullOutA, Turn2A, TowardWobA, PickUpWobA,
        WobbeYoinkA, TurnToDeliverA, PullOut2A,
        StraightenA, ParkA,ForwardA,
        Turn3A, Turn4A,

        PathB, FaceTargetB, AimB,
        PullOutB, StraightenB,
        Straighten2B, IntakeRingB,
        TurnToDeliverB, TurnToDeliver2B,
        ParkB,

        PathC, ParkC, StraightenC, FaceRingsC,
        ForwardToShootC, IntakeRing1C,
        AimC, BackUpC, FaceRing2C, IntakeRing3C,
        Aim2C, ForwardToDeliverC, Straighten2C, DeliverWobC,
        PullOutC, FaceTargetC, TurnToParkC, Aim1C,
        Aim2B, FaceRingB, ForwardToDeliverB, TurnToGrabB,
        DeliverWob2B, DriveToGrabPt2B,
        DriveToGrabPt1B,



    }

    FullRed.State state;


    @Override
    public void init() {
        robot.initHardware(hardwareMap);
        vision.initVision(hardwareMap);
        sensors.initSensors(hardwareMap);
        method.initMethods(hardwareMap);

        robot.hopper.setPosition(robot.SHOOT_POSITION);
        robot.wobbleGrab.setPosition(robot.OPEN);


        time.reset();
        state = FullRed.State.Vision;
    }

    @Override
    public void start(){
        Reset();
    }

    @Override
    public void loop() {
        double CurrentTime = time.time();
        telemetry.addData("time", time.time());
        telemetry.addData("ZAngle", sensors.getZAngle());
        telemetry.addData("Current State",state.toString());
        telemetry.addData("Current Position", vision.pipeline.position);

        telemetry.addData("Shooter RPM = ", currentRPM);
        telemetry.addData("Shooter RPM2 = ", currentRPM2);
        telemetry.update();

        celebrate = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

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
                    state = FullRed.State.PathC;
                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.ONE && CurrentTime >=2) {
                    pathB = true;
                    Reset();
                    state = FullRed.State.PathB;
                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.NONE && CurrentTime >= 2) {
                    pathA = true;
                    Reset();
                    state = FullRed.State.PathA;
                }
                break;


//-         Start of PATH A part one(No rings)
            case PathA:
                robot.wobbleGrab.setPosition(robot.CLOSED);
                method.Reverse(.5, 40);
                if (method.DriveDone(40)) {
                    Reset();
                    state = FullRed.State.Aim;
                }
                break;

            case ForwardA:
                method.Forward(.5, 26);
                if (method.DriveDone(26)) {
                    Reset();
                    state = FullRed.State.TurnA;
                }
                break;

            case TurnA:
                method.TurnAbsolute(-90, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(-90) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.DeliverWobA;
                }
                break;

            case DeliverWobA:
                method.Reverse(.5, 15);
                if (method.DriveDone(15)) {
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(500);
                    Reset();
                    state = FullRed.State.PullOutA;
                }
                break;

            case PullOutA:
                method.Forward(.5,3);
                if (method.DriveDone(3)){
                    Reset();
                    state = FullRed.State.Turn2A;
                }
                break;

            case Turn2A:
                method.TurnAbsolute(0, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.TowardWobA;
                }
                break;

            case TowardWobA:
                method.Reverse(.5, 46);
                if (method.DriveDone(46)){
                    Reset();
                    state = FullRed.State.Turn3A;
                }
                break;

            case Turn3A:
                method.TurnAbsolute(-2, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(-2) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.PickUpWobA;
                }
                break;

            case PickUpWobA:
                method.Reverse(.2, 12);
                if (method.DriveDone(12)){
                    robot.wobbleGrab.setPosition(robot.CLOSED);
                    method.sleep(400);
                    Reset();
                    state = FullRed.State.Turn4A;
                }
                break;

            case Turn4A:
                method.TurnAbsolute(0, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.WobbeYoinkA;
                }
                break;

            case WobbeYoinkA:
                method.Forward(.5, 47);
                if (method.DriveDone(47)){
                    Reset();
                    state = FullRed.State.TurnToDeliverA;
                }
                break;

            case TurnToDeliverA:
                method.TurnAbsolute(-90, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(-90) && CurrentTime >= 2){
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(400);
                    Reset();
                    state = FullRed.State.PullOut2A;
                }
                break;

//            case DeliverWob2A:
//                method.Reverse(.5, 4);
//                if (method.DriveDone(4)){
//                    robot.wobbleGrab.setPosition(robot.OPEN);
//                    method.sleep(500);
//                    Reset();
//                    state = State.PullOut2A;
//                }
//                break;

            case PullOut2A:
                method.Forward(.5,26);
                if (method.DriveDone(26)){
                    Reset();
                    state = FullRed.State.StraightenA;
                }
                break;

            case StraightenA:
                method.TurnAbsolute(0, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.ParkA;
                }
                break;

            case ParkA:
                method.Forward(.5,8);
                if (method.DriveDone(8)){
                    Reset();
                    state = FullRed.State.Stop;
                }
                break;

// the robot is named Incognito 3/2/21

//-         Start of PATH B part one(one ring)
            case PathB:
                robot.wobbleGrab.setPosition(robot.CLOSED);
                robot.intakeServo.setPosition(0);
                robot.hopper.setPosition(robot.INTAKE_POSITION);
                method.Reverse(.5, 20);
                if (method.DriveDone(20)) {
                    Reset();
                    state = FullRed.State.AimB;
                }
                break;

            case AimB:
                method.TurnAbsolute(4, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(4) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.Shoot;
                }
                break;

            case FaceRingB:
                robot.hopper.setPosition(robot.INTAKE_POSITION);
                method.TurnAbsolute(20, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(20) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.IntakeRingB;
                }
                break;

            case IntakeRingB:
                robot.intake.setPower(1);
                method.Forward(.5, 10);
                if (method.DriveDone(10)){
                    pathB = false;
                    shoot2B = true;
                    Reset();
                    state = FullRed.State.Aim2B;
                }
                break;

            case Aim2B:
                method.TurnAbsolute(4, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(4) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.Shoot;
                }
                break;

            case TurnToDeliverB:
                robot.hopper.setPosition(robot.INTAKE_POSITION);
                method.TurnAbsolute(15, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(15) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.ForwardToDeliverB;
                }
                break;

            case ForwardToDeliverB:
                method.Forward(.5, 55);
                if (method.DriveDone(55)){
                    Reset();
                    state = FullRed.State.TurnToDeliver2B;
                }
                break;

            case TurnToDeliver2B:
                method.TurnAbsolute(90, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(90) && CurrentTime >= 2){
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(400);
                    Reset();
                    state = FullRed.State.PullOutB;
                }
                break;

//            case DeliverWobPt1B:
//                method.Reverse(.5, 5);
//                if (method.DriveDone(5)){
//                    robot.wobbleGrab.setPosition(robot.OPEN);
//                    method.sleep(400);
//                    Reset();
//                    state = State.PullOutB;
//                }
//                break;

            case PullOutB:
                method.Forward(.5, 3);
                if (method.DriveDone(3)){
                    Reset();
                    state = FullRed.State.TurnToGrabB;
                }
                break;

            case TurnToGrabB:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.DriveToGrabPt1B;
                }
                break;

            case DriveToGrabPt1B:
                method.Reverse(.5, 55);
                if (method.DriveDone(55)){
                    Reset();
                    state = FullRed.State.DriveToGrabPt2B;
                }
                break;

            case DriveToGrabPt2B:
                method.Reverse(.2, 18);
                if (method.DriveDone(18)){
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(400);
                    Reset();
                    state = FullRed.State.FaceTargetB;
                }
                break;

            case FaceTargetB:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.DeliverWob2B;
                }
                break;

            case DeliverWob2B:
                method.Forward(.5, 75);
                if (method.DriveDone(75)){
                    Reset();
                    state = FullRed.State.StraightenB;
                }
                break;

//            case ForwardB:
//                method.Forward(.5, 12);
//                if (method.DriveDone(12)){
//                    Reset();
//                    state = State.StraightenB;
//                }
//                break;

            case StraightenB:
                method.TurnAbsolute(90, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(90) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.Straighten2B;
                }
                break;

            case Straighten2B:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.ParkB;
                }
                break;

            case ParkB:
                method.Reverse(.6, 26);
                if (method.DriveDone(26)){
                    Reset();
                    state = FullRed.State.Stop;
                }
                break;


//-         Start of Path C part one(four rings)
            case PathC:
                method.Reverse(.5, 10);
                robot.wobbleGrab.setPosition(robot.CLOSED);
                method.sleep(400);
                if (method.StrafeDone(10)){
                    Reset();
                    state = FullRed.State.StraightenC;
                }
                break;

            case StraightenC:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 1.5){
                    Reset();
                    state = FullRed.State.ForwardToShootC;
                }
                break;

            case ForwardToShootC:
                robot.intakeServo.setPosition(0);
                robot.hopper.setPosition(robot.INTAKE_POSITION);
                method.Forward(.5, 10);
                if (method.DriveDone(10)){
                    Reset();
                    state = FullRed.State.Aim1C;
                }
                break;

            case Aim1C:
                method.TurnAbsolute(7, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(7) && CurrentTime >= 1.5){
                    Reset();
                    state = FullRed.State.Shoot;
                }
                break;

            case FaceRingsC:
                robot.hopper.setPosition(robot.INTAKE_POSITION);
                method.TurnAbsolute(20, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(20) && CurrentTime >= 1.5){
                    Reset();
                    state = FullRed.State.IntakeRing1C;
                }
                break;

            case IntakeRing1C:
                robot.intake.setPower(1);
                method.Forward(.5, 20);
                if (method.DriveDone(20)) {
                    Reset();
                    pathC = false;
                    shoot2C = true;
                    state = FullRed.State.AimC;
                }
                break;

            case AimC:
                method.TurnAbsolute(1, sensors.getZAngle(), .75, .75);
                i = 0;
                if (method.TurnDone(1) && CurrentTime >= 1.5){
                    Reset();
                    state = FullRed.State.Shoot;
                }
                break;

            case BackUpC:
                robot.intake.setPower(-1);
                method.Reverse(.5, 2);
                if (method.DriveDone(2)){
                    Reset();
                    state = FullRed.State.FaceRing2C;
                }
                break;

            case FaceRing2C:
                method.TurnAbsolute(20, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(20) && CurrentTime >= 1){
                    Reset();
                    state = FullRed.State.IntakeRing3C;
                }
                break;

            case IntakeRing3C:
                robot.intake.setPower(1);
                method.Forward(.5,10);
                if (method.DriveDone(10) || CurrentTime >= 4){
                    Reset();
                    state = FullRed.State.Aim2C;
                }
                break;

            case Aim2C:
                method.TurnAbsolute(2, sensors.getZAngle(), .75, .75);
                i = 0;
                if (method.TurnDone(2) && CurrentTime >= 1.5){
                    shoot2C = false;
                    shoot3C = true;
                    Reset();
                    state = FullRed.State.Shoot;
                }
                break;

            case Straighten2C:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state = FullRed.State.ForwardToDeliverC;
                }
                break;

            case ForwardToDeliverC:
                method.Forward(.5, 75);
                if(method.DriveDone(75)){
                    Reset();
                    state = FullRed.State.FaceTargetC;
                }
                break;

            case FaceTargetC:
                method.TurnAbsolute(-90, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(-90) && CurrentTime >= 1.5){
                    Reset();
                    state = FullRed.State.DeliverWobC;
                }
                break;

            case DeliverWobC:
                method.Reverse(.5, 12);
                if(method.DriveDone(12)){
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(400);
                    Reset();
                    state = FullRed.State.PullOutC;
                }
                break;

            case PullOutC:
                method.Forward(.5, 10);
                if(method.DriveDone(10)){
                    Reset();
                    state = FullRed.State.TurnToParkC;
                }
                break;

            case TurnToParkC:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 1.5){
                    Reset();
                    state = FullRed.State.ParkC;
                }
                break;

            case ParkC:
                method.Reverse(1, 45);
                if (method.DriveDone(45)){
                    Reset();
                    state = FullRed.State.Stop;
                }
                break;

//-         these cases are for all 3 paths
            case Aim:
                method.TurnAbsolute(3, sensors.getZAngle(), -.65,.65);
                i=0;
                if (method.TurnDone(3) && CurrentTime >= 2){
                    Reset();
                    robot.hopper.setPosition(robot.SHOOT_POSITION);
                    method.sleep(400);
                    state = FullRed.State.Shoot;
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
                    if (i == 4 && pathA) {
                        i = 0;
                        Reset();
                        state = FullRed.State.ForwardA;
                    } else if (i == 5 && pathB){
                        i = 0;
                        Reset();
                        state = FullRed.State.FaceRingB;
                    } else if (i == 3 && shoot2B){
                        i = 0;
                        Reset();
                        state = FullRed.State.TurnToDeliverB;
                    }
                    else if (i == 5 && pathC){
                        i = 0;
                        Reset();
                        state = FullRed.State.FaceRingsC;
                    } else if (i == 4 && shoot2C){
                        i = 0;
                        Reset();
                        robot.hopper.setPosition(robot.INTAKE_POSITION);
                        state = FullRed.State.BackUpC;
                    } else if (i == 5 && shoot3C){
                        i = 0;
                        Reset();
                        robot.hopper.setPosition(robot.INTAKE_POSITION);
                        state = FullRed.State.Straighten2C;
                    }
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