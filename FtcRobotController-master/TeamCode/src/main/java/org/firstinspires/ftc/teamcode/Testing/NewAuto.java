package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ReadWriteFile;


import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.SubSystems.*;

import java.io.File;

import static java.lang.String.valueOf;

//@Disabled
@Autonomous(name = "NewAuto")

public class NewAuto extends OpMode {
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
    private boolean positionA;
    private boolean positionB;
    private boolean positionC;

    File autonomousGyroFile = AppUtil.getInstance().getSettingsFile("autonomousGyroFile.txt");

    enum State {
        Stop, Split, ReverseA, TurnB, ReverseC, PathA, ShootA, StrafeA, Turn, DriveToTarget
    }

    State state;



    @Override
    public void init() {
        robot.initHardware(hardwareMap);
        vision.initVision(hardwareMap);
        sensors.initSensors(hardwareMap);
        method.initMethods(hardwareMap);

        robot.hopper.setPosition(robot.SHOOT_POSITION);

        time.reset();
        state = State.Turn;

        if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            positionC = true;
        } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.ONE) {
            positionB = true;
        } else if (vision.pipeline.position == Vision.SkystoneDeterminationPipeline.RingPosition.NONE) {
            positionA = true;
        }

    }

    @Override
    public void loop() {
        double CurrentTime = time.time();
        telemetry.addData("time", CurrentTime);
        telemetry.addData("ZAngle", sensors.getZAngle());
        telemetry.addData("Current State", state.toString());
        telemetry.addData("Current Position", vision.pipeline.position);
        telemetry.addData("Shooter RPM = ", currentRPM);
        telemetry.addData("Shooter RPM2 = ", currentRPM2);
        telemetry.update();

//            currentTick = robot.shootyMcShootShoot.getCurrentPosition();
//            currentTick2 = robot.shooter2.getCurrentPosition();
//            currentTime = runtime.time();
//            currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
//            currentRPM = method.calcRPM(lastTick, lastTime, currentTick, currentTime);
//            currentRPM2 = method.calcRPM(lastTick2, lastTime, currentTick2, currentTime);
//            lastTick = currentTick;
//            lastTick2 = currentTick2;
//            lastTime = currentTime;
//            shooterPower = method.shooterPowerForAuto(shooterPower, currentRPM, 3500);
//            shooterPower2 = method.shooterPowerForAuto(shooterPower2, currentRPM2, 3250);
//
//            robot.shootyMcShootShoot.setPower(shooterPower);
//            robot.shooter2.setPower(shooterPower2);

        switch (state) {

            case DriveToTarget:
                method.Forward(.5,10 );
                if (method.DriveDone(10)){
                    state = State.Turn;
                }
                break;

            case Turn:
                method.TurnAbsolute(90, sensors.getZAngle(), -.75, .75);
                if (method.TurnDone(90)){
                    state = State.Stop;
                }
                break;

            case Stop:
                //-will write the current ZAngle to a file that will be used in TeleOp
                ReadWriteFile.writeFile(autonomousGyroFile, valueOf(sensors.getZAngle()));
                Reset();
                method.Death();
                break;
        }
    }
    private void Reset () {
        time.reset();
        method.Kill();
    }
}
