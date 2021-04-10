package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/*
 -This class houses all of our methods that we use during our OpModes for ease
 */

public class Methods {

    HardwareMap hwMap = null;
    HardWareMap robot = new HardWareMap();
    Sensors sensors = new Sensors();


    public double currentRPM, currentRPM2;

    private double rightBack, rightFront, leftFront, leftBack;

//- This inits the sensors and hardware classes so that we can use those in this class
    public void initMethods(HardwareMap ahwMap) {
        hwMap = ahwMap;
        sensors.initSensors(ahwMap);
        robot.initHardware(ahwMap);
    }

    //- This will allow you to input the desired rpm for both shooter motors
    public void shooterSpeed(double desiredRPM, double desiredRPM2){
        robot.shootyMcShootShoot.setVelocity(ticksPerSecond(desiredRPM));
        robot.shooter2.setVelocity(ticksPerSecond(desiredRPM2));
    }
    //- This converts rpm to ticks per second because that's what the motors take
    public double ticksPerSecond(double desiredSpeed){
        return (desiredSpeed * robot.COUNTS_PER_ROTATION / 60);
    }

//- sets all motors to zero (aka sentences them to death)
    public void Death(){
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.shootyMcShootShoot.setPower(0);
        robot.shooter2.setPower(0);
        robot.intake.setPower(0);
    }

//- this allows us to input desired speed and desired inches to move forward
    public void Forward(double speed, double distance){

        robot.frontLeft.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));
        robot.frontRight.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));
        robot.backLeft.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));
        robot.backRight.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontRight.setPower(speed);
        robot.backRight.setPower(speed);
        robot.frontLeft.setPower(speed);
        robot.backLeft.setPower(speed);
    }

//- this allows us to input desired speed and desired inches to move in reverse
    public void Reverse(double speed, double distance){

        robot.backLeft.setTargetPosition((int) (-distance *  robot.COUNTS_PER_INCH));
        robot.frontLeft.setTargetPosition((int) (-distance *  robot.COUNTS_PER_INCH));
        robot.frontRight.setTargetPosition((int) (-distance * robot.COUNTS_PER_INCH));
        robot.backRight.setTargetPosition((int) (-distance * robot.COUNTS_PER_INCH));

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontRight.setPower(-speed);
        robot.backRight.setPower(-speed);
        robot.frontLeft.setPower(-speed);
        robot.backLeft.setPower(-speed);


    }

//- this allows us to input desired speed and desired inches to strafe right
    public void StrafeRight(double speed, double distance){
        robot.frontLeft.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));
        robot.frontRight.setTargetPosition((int) (-distance * robot.COUNTS_PER_INCH));
        robot.backLeft.setTargetPosition((int) (-distance * robot.COUNTS_PER_INCH));
        robot.backRight.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(-speed);
        robot.backLeft.setPower(-speed);
        robot.backRight.setPower(speed);
    }

//- this allows us to input desired speed and desired inches to strafe left
    public void StrafeLeft(double speed, double distance){
        robot.frontLeft.setTargetPosition((int) (-distance * robot.COUNTS_PER_INCH));
        robot.frontRight.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));
        robot.backLeft.setTargetPosition((int) (distance * robot.COUNTS_PER_INCH));
        robot.backRight.setTargetPosition((int) (-distance * robot.COUNTS_PER_INCH));

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(-speed);
        robot.frontRight.setPower(speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(-speed);
    }

//- Allows to set power to drive motors directly
    public void setDrivePower (double rightFront, double leftFront, double leftBack, double rightBack){
        robot.frontLeft.setPower(leftFront);
        robot.frontRight.setPower(rightFront);
        robot.backLeft.setPower(leftBack);
        robot.backRight.setPower(rightBack);
    }

    /*
     - Method calcRPM()
     -Calculates the rpm of the shooter motors
     */
    public double calcRPM(double tick0, double time0, double tick1, double time1){
        double rPM = (((Math.abs(tick0-tick1)/robot.COUNTS_PER_ROTATION))/(Math.abs(time1-time0)))*60;
        return (rPM);
    }   // close calcRPM method

    /*
     - Method shooterPower()
     - Old method to put shooter motors to specific power
     */
    public double shooterPower(double shooterPower, double currentRPM, double targetRPM){

        double integral = 0;
        double error = targetRPM - currentRPM;
        double Cp = 0.0000015;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxPower = 1;
        double derivative = 0, deltaError, lastError=0;

        double powerAdjust = ((Cp*error) + (Ci * integral) + (Cd * derivative)) * maxPower;

        shooterPower = shooterPower + powerAdjust;
        return (Range.clip(shooterPower,-maxPower, maxPower));
    }   // close shooterPower method

    /*
     - Method gyroPositive
     - Will see if gyro angle is positive or negative
     */
    public double gyroPositive(double targetAngle){
        double currentZ = sensors.getZAngle();
        double rotationalAngle = 0;

        if ((currentZ >= 0)  || (targetAngle <= 180)) {
            rotationalAngle = currentZ;
        } else {
            rotationalAngle = 180 + (180 + currentZ);
        }

        return rotationalAngle;
    }


    /*
     - Method gyroNegative
     - Will see if gyro angle is positive or negative
     */
    public double gyroNegative(double targetAngle){
        double currentZ = sensors.getZAngle();
        double rotationalAngle = 0;

        if ((currentZ <= 0) || (targetAngle >= -180)) {
            rotationalAngle = currentZ;
        } else {
            rotationalAngle = -180 - (180 - currentZ);
        }

        return rotationalAngle;
    }
    /*
     - Method: PIDRotate
     - Parameters:
     -      targetAngle -> desire ending angle/position of the robot
     -      targetError -> how close should the robot get to the desired angle
     */
    public void PIDRotate(double targetAngle, double targetError){
        double integral = 0;
        int iterations = 0;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timer.time();
        double totalTime;
        double error = 0;
        double Cp = 0.015;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 1;
        double rotationSpeed;
        double derivative = 0, deltaError, lastError=0;

        if (targetAngle > 0 ) {
            error = gyroPositive(targetAngle) - targetAngle;
        } else {
            error = gyroNegative(targetAngle) - targetAngle;
        }

        while ((Math.abs(error) >= targetError)){
            deltaError = lastError - error;
            rotationSpeed = ((Cp*error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

            // Clip motor speed to between -1 and 1
            if (rotationSpeed > maxSpeed) rotationSpeed = maxSpeed;
            else if (rotationSpeed < -maxSpeed) rotationSpeed = -maxSpeed;
            if (rotationSpeed != 0 & Math.abs(rotationSpeed) < 0.05) rotationSpeed = rotationSpeed * 2;

            rightFront = rotationSpeed;
            leftFront = -rotationSpeed;
            leftBack = -rotationSpeed;
            rightBack = rotationSpeed;

            setDrivePower(rightFront, leftFront, leftBack, rightBack);

            lastError = error;
            iterations++;


            derivative = deltaError/timer.time();
            timer.reset();

            if (targetAngle >= 0 ) {
                error = gyroPositive(targetAngle) - targetAngle;
            } else {
                error = gyroNegative(targetAngle) - targetAngle;
            }

            // Overshooting the targetAngle & need to make correction in the code
            // underpowering the wheels on overshoot correction

        }   // end of while Math.abs(error)

        // shut off the drive motors
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);

        totalTime = timeElapsed.time() - startTime;
    }   //end of the PIDRotate Method

//- Calculates the turning, will also correct if the robot turns too far
    public void TurnAbsolute(double target, double heading, double maxSpeed, double minSpeed){

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Error   = heading - target;
        double Kp = 0.017;
        double LFPower;
        double LRPower;
        double RFPower;
        double RRPower;
        double EPower;

        if ((Math.abs(Error)) > 2 ){
            LFPower = -Error * Kp;
            LRPower = -Error * Kp;
            RFPower = Error * Kp;
            RRPower = Error * Kp;
            EPower = Error * Kp;

            Range.clip(LFPower,minSpeed,maxSpeed);
            Range.clip(LRPower,minSpeed,maxSpeed);
            Range.clip(RFPower,minSpeed,maxSpeed);
            Range.clip(RRPower,minSpeed,maxSpeed);
            Range.clip(EPower,minSpeed,maxSpeed);

            robot.frontLeft.setPower(LFPower);
            robot.backLeft.setPower(LRPower);
            robot.frontRight.setPower(RFPower);
            robot.backRight.setPower(RRPower);
        }
        else {
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
    }

//- Stops and resets the encoders of the drive motors
    public void Kill(){
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
    -Checks to see if the motors are running
     -and if they're not it will return true
     */
    public boolean IsBusy(){
        if (!robot.frontLeft.isBusy() || !robot.backLeft.isBusy() || !robot.frontRight.isBusy() || !robot.backRight.isBusy())
        {
            return (true);
        } else return (false);
    }

    //-Reads if the robot drove the right distance
    public boolean DriveDone(double distance){
        if ((Math.abs(robot.frontLeft.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance) &&
                (Math.abs(robot.backLeft.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance) &&
                 (Math.abs(robot.frontRight.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance) &&
                (Math.abs(robot.backRight.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance)
        )
        {
            return (true);
        } else return (false);
    }

//- reads if the robot turned correctly (with +/- 2 degrees)
    public boolean TurnDone(double target){
        if(
            (sensors.getZAngle() >= (target - 2)) && (sensors.getZAngle() <= (target + 2))
        ){
            return(true);
        }
        else {
            return (false);
        }
    }

//- Reads if the robot strafed the right distance
    public boolean StrafeDone(double distance){
        if (Math.abs(robot.frontLeft.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance
                || Math.abs(robot.backRight.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance)
        {
            return (true);
        } else return (false);
    }

//- Will make robot sleep/wait
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}