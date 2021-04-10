package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.HardWareMap;
import org.firstinspires.ftc.teamcode.SubSystems.Methods;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@Autonomous(name = "TeamworkBlue")

public class TeamworkBlue extends OpMode {
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

    private RevBlinkinLedDriver.BlinkinPattern celebrate, APath, BPath, CPath;

    ElapsedTime runtime = new ElapsedTime();

    enum State {
        Vision, Aim, Shoot,  Stop, Forward,

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
        DriveToGrabPt1B, Straighten1A, FaceRingAgain, RingStreighten,
        AfterDropAngle, DropOff, dropAngle, ForwardToRings, PullOutC2,
        BackToOtherRings, TurnTOOtherRings, ForwardsIntoOtherRings,
        OverCorrection, AimAfterInteke, MoveToLastRing, ParkTurnC
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
        state =  State.Vision;
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

        method.shooterSpeed(4600, 4400);

        switch (state) {

            case Vision:
                robot.wobbleGrab.setPosition(robot.CLOSED);
                if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.FOUR && CurrentTime >= 1) {
                    pathC = true;
                    Reset();
                    state =  State.Forward;
                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.ONE && CurrentTime >=2) {
                    pathB = true;
                    Reset();
                    state =  State.Forward;
                } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.NONE && CurrentTime >= 2) {
                    pathA = true;
                    Reset();
                    state =  State.Forward;
                }
                break;

            case Forward:
                robot.wobbleGrab.setPosition(robot.CLOSED);
                method.Reverse(.5, 45);
                if (method.DriveDone(45)) {
                    Reset();
                    state = State.Aim;
                }
                break;


//-         Start of PATH A part one(No rings)
            case Straighten1A:
                method.TurnAbsolute(0, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state =  State.ForwardA;
                }
                break;

            case ForwardA:
                method.Forward(.5, 22);
                if (method.DriveDone(22)) {
                    Reset();
                    state =  State.TurnA;
                }
                break;

            case TurnA:
                method.TurnAbsolute(90, sensors.getZAngle(), -.65,.65);
                if (method.TurnDone(90) && CurrentTime >= 2){
                    Reset();
                    state =  State.DeliverWobA;
                }
                break;

            case DeliverWobA:
                method.Reverse(.5, 25);
                if (method.DriveDone(25)) {
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(500);
                    Reset();
                    state =  State.PullOutA;
                }
                break;

            case PullOutA:
                method.Forward(.75,4);
                if (method.DriveDone(4)){
                    Reset();
                    state =  State.Stop;
                }
                break;

// the robot is named Incognito 3/2/21

//-         Start of PATH B part one(one ring)
            case FaceRingB:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state =  State.IntakeRingB;
                }
                break;

            case IntakeRingB:
                method.Forward(.5, 38);
                if (method.DriveDone(38)){
                    pathB = false;
                    shoot2B = true;
                    Reset();
                    state =  State.Aim2B;
                }
                break;

            case Aim2B:
                method.TurnAbsolute(90, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(90) && CurrentTime >= 2){
                    Reset();
                    state =  State.TurnToDeliverB;
                }
                break;

            case TurnToDeliverB:
                method.Reverse(.5, 2);
                if (method.DriveDone(2)) {
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(500);
                    Reset();
                    state =  State.ForwardToDeliverB;
                }
                break;

            case ForwardToDeliverB:
                method.Forward(.5, 3);
                if (method.DriveDone(3)){
                    Reset();
                    state =  State.TurnToDeliver2B;
                }
                break;

            case TurnToDeliver2B:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state =  State.PullOutB;
                }
                break;

            case PullOutB:
                robot.hopper.setPosition(robot.INTAKE_POSITION);
                robot.intakeServo.setPosition(0);
                method.Reverse(.5, 64);
                if (method.DriveDone(64)){
                    Reset();
                    state =  State.TurnToGrabB;
                }
                break;

            case TurnToGrabB:
                robot.intake.setPower(1);
                method.TurnAbsolute(-55, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(-55) && CurrentTime >= 2){
                    Reset();
                    state =  State.DriveToGrabPt1B;
                }
                break;

            case DriveToGrabPt1B:
                method.Forward(.5, 25);
                if (method.DriveDone(25)){
                    method.sleep(1000);
                    pathB = false;
                    shoot2B = true;
                    Reset();
                    state =  State.AimB;
                }
                break;

            case AimB:
                robot.intake.setPower(0);
                robot.hopper.setPosition(robot.SHOOT_POSITION);
                method.TurnAbsolute(-4, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(-4) && CurrentTime >= 2){
                    Reset();
                    state =  State.Shoot;
                }
                break;

            case StraightenB:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 2){
                    Reset();
                    state =  State.ParkB;
                }
                break;

            case ParkB:
                method.Forward(.5, 28);
                if (method.DriveDone(28)){
                    Reset();
                    state =  State.Stop;
                }
                break;


//-         Start of Path C part one(four rings)
            case StraightenC:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(0) && CurrentTime >= 1.5){
                    Reset();
                    state =  State.ForwardToRings;
                }
                break;

            case ForwardToRings:
                method.Forward(.6, 60);
                if (method.DriveDone(60)){
                    Reset();
                    state =  State.dropAngle;
                }
                break;

            case dropAngle:
                method.TurnAbsolute(90, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(90) && CurrentTime >= 1.5){
                    Reset();
                    state =  State.DropOff;
                }
                break;

            case DropOff:
                method.Reverse(.6, 28);
                if (method.DriveDone(28)) {
                    robot.wobbleGrab.setPosition(robot.OPEN);
                    method.sleep(400);
                    Reset();
                    state =  State.AfterDropAngle;
                }
                break;

            case AfterDropAngle:
                method.TurnAbsolute(90, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(90) && CurrentTime >= 1.5){
                    Reset();
                    state =  State.PullOutC2;
                }
                break;

            case PullOutC2:
                method.Forward(.65, 30);
                if (method.DriveDone(30)){
                    Reset();
                    state =  State.RingStreighten;
                }
                break;

            case RingStreighten:
                method.TurnAbsolute(0, sensors.getZAngle(), .75, -.75);
                if (method.TurnDone(0) && CurrentTime >= 1){
                    Reset();
                    state =  State.BackToOtherRings;
                }
                break;

            case BackToOtherRings:
                robot.hopper.setPosition(robot.INTAKE_POSITION);
                robot.intakeServo.setPosition(0);
                method.Reverse(.65,95);
                if (method.DriveDone(95)){
                    Reset();
                    state =  State.TurnTOOtherRings;
                }
                break;

            case TurnTOOtherRings:
                method.TurnAbsolute(-46, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(-46) && CurrentTime >= 1.5){
                    Reset();
                    state =  State.ForwardsIntoOtherRings;
                }
                break;

            case ForwardsIntoOtherRings:
                robot.intake.setPower(1);
                method.Forward(.40, 32);
                if(method.DriveDone(32)){
                    Reset();
                    state =  State.AimAfterInteke;
                }
                break;

//            case OverCorrection:
//                robot.intake.setPower(-1);
//                method.Reverse(.65, 3);
//                if(method.DriveDone(3)){
//                    Reset();
//                    state =  State.AimAfterInteke;
//                }
//                break;

            case AimAfterInteke:
                method.TurnAbsolute(4, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(4) && CurrentTime >= 1.5){
                    robot.intake.setPower(0);
                    pathC = false;
                    shoot2C = true;
                    shoot3C = false;
                    Reset();
                    state =  State.Shoot;
                }
                break;

            case FaceRingAgain:
                method.TurnAbsolute(-48, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(-48) && CurrentTime >= 1){
                    Reset();
                    state =  State.MoveToLastRing;
                }
                break;

            case MoveToLastRing:
                robot.intake.setPower(1);
                method.Forward(.5, 12);
                if(method.DriveDone(12) && CurrentTime >= 1){
                    Reset();
                    state =  State.TurnToParkC;
                }
                break;

            case TurnToParkC:
                method.TurnAbsolute(8, sensors.getZAngle(), .75, .75);
                if (method.TurnDone(8) && CurrentTime >= 2){
                    robot.intake.setPower(0);
                    shoot2C = false;
                    shoot3C = true;
                    Reset();
                    state =  State.Shoot;
                }
                break;

            case ParkC:
                method.Forward(.75, 30);
                if (method.DriveDone(30)){
                    Reset();
                    state =  State.Stop;
                }
                break;

//-         these cases are for all 3 paths
            case Aim:
                method.TurnAbsolute(-22, sensors.getZAngle(), -.65,.65);
                i=0;
                if (method.TurnDone(-22) && CurrentTime >= 2){
                    Reset();
                    robot.hopper.setPosition(robot.SHOOT_POSITION);
                    method.sleep(400);
                    state =  State.Shoot;
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
                        state =  State.Straighten1A;
                    } else if (i == 5 && pathB){
                        i = 0;
                        Reset();
                        state =  State.FaceRingB;
                    } else if (i == 3 && shoot2B){
                        i = 0;
                        Reset();
                        state =  State.StraightenB;
                    }
                    else if (i == 5 && pathC){
                        i = 0;
                        Reset();
                        state =  State.StraightenC;
                    } else if (i == 6 && shoot2C){
                        i = 0;
                        Reset();
                        robot.hopper.setPosition(robot.INTAKE_POSITION);
                        state =  State.FaceRingAgain;
                    } else if (i == 5 && shoot3C){
                        i = 0;
                        Reset();
                        robot.hopper.setPosition(robot.INTAKE_POSITION);
                        state =  State.ParkC;
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