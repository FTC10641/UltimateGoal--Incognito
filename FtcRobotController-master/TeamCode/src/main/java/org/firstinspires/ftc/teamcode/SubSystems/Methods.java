package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 -This class houses all of our methods that we use during autonomous
 */

public class Methods {

    HardwareMap hwMap = null;
    HardWareMap robot = new HardWareMap();
    Sensors sensors = new Sensors();
    ElapsedTime runtime = new ElapsedTime();
    double CurrentTime = runtime.time();

    private boolean shooter, shooter2;
    private double currentTick, currentTick2;
    public double currentRPM, currentRPM2;
    private double lastTick, lastTick2;
    private double shooterPower, shooterPower2;
    private double currentTime;
    private double lastTime;

    private double rightBack, rightFront, leftFront, leftBack;

    public void shooterSpeed(double desiredRPM, double desiredRPM2){
        robot.shootyMcShootShoot.setVelocity(ticksPerSecond(desiredRPM));
        robot.shooter2.setVelocity(ticksPerSecond(desiredRPM2));
    }

    public double ticksPerSecond(double desiredSpeed){
        return (desiredSpeed * robot.COUNTS_PER_ROTATION / 60);
    }

    public void initMethods(HardwareMap ahwMap) {
        hwMap = ahwMap;
        sensors.initSensors(ahwMap);
        robot.initHardware(ahwMap);
    }

    public double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


//    public void Forward(double speed, double distance){
//        robot.frontRight.setTargetPosition((int) (distance  * robot.OdometryCountsPerInch));
//        robot.backRight.setTargetPosition((int) (distance * robot.OdometryCountsPerInch));
//        robot.frontLeft.setTargetPosition((int) (distance * robot.OdometryCountsPerInch));
//        robot.backLeft.setTargetPosition((int) (distance * robot.OdometryCountsPerInch));
//
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontRight.setPower(speed);
//        robot.backRight.setPower(speed);
//        robot.frontLeft.setPower(speed);
//        robot.backLeft.setPower(speed);
//    }
//
//    public void Reverse(double speed, double distance){
//        robot.frontRight.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//        robot.backRight.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//        robot.frontLeft.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//        robot.backLeft.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontRight.setPower(-speed);
//        robot.backRight.setPower(-speed);
//        robot.frontLeft.setPower(-speed);
//        robot.backLeft.setPower(-speed);
//    }
//
//    public void StrafeLeft(double speed, double distance){
//        robot.frontRight.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//        robot.backRight.setTargetPosition((int) (distance * robot.OdometryCountsPerInch));
//        robot.frontLeft.setTargetPosition((int) (distance * robot.OdometryCountsPerInch));
//        robot.backLeft.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontRight.setPower(-speed);
//        robot.backRight.setPower(speed);
//        robot.frontLeft.setPower(speed);
//        robot.backLeft.setPower(-speed);
//    }
//
//    public void StrafeRight(double speed, double distance){
//        robot.frontRight.setTargetPosition((int) (distance * robot.OdometryCountsPerInch));
//        robot.backRight.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//        robot.frontLeft.setTargetPosition((int) (-distance * robot.OdometryCountsPerInch));
//        robot.backLeft.setTargetPosition((int) (distance * robot.OdometryCountsPerInch));
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontRight.setPower(speed);
//        robot.backRight.setPower(-speed);
//        robot.frontLeft.setPower(-speed);
//        robot.backLeft.setPower(speed);
//    }
//
//
//    public void TurnAbsolute(double target, double heading){
//
//        robot.verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        double Error = heading - target;
//        double Kp = 0.017;
//        double LFPower;
//        double LRPower;
//        double RFPower;
//        double RRPower;
//        double EPower;
//
//        if ((Math.abs(Error)) > 2 ){
//            LFPower = -Error * Kp;
//            LRPower = -Error * Kp;
//            RFPower = Error * Kp;
//            RRPower = Error * Kp;
//            EPower = Error * Kp;
//
//            Range.clip(LFPower,-1,1);
//            Range.clip(LRPower,-1,1);
//            Range.clip(RFPower,-1,1);
//            Range.clip(RRPower,-1,1);
//            Range.clip(EPower,-1,1);
//
//            robot.frontLeft.setPower(LFPower);
//            robot.backLeft.setPower(LRPower);
//            robot.frontRight.setPower(RFPower);
//            robot.backRight.setPower(RRPower);
//        }
//        else {
//            robot.frontLeft.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backRight.setPower(0);
//        }
//    }
//
//
//    public void Kill(){
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
    public void Death(){
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.shootyMcShootShoot.setPower(0);
        robot.shooter2.setPower(0);
        robot.intake.setPower(0);
    }
//
//    //this method is very important, DON'T delete
//    public void StopUsingEncoders(){
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    /**
//     Checks to see if the motors are running
//     and if they're not it will return true
//     */
//    public boolean IsBusy(){
//        if (!robot.frontLeft.isBusy() || !robot.backRight.isBusy())
//        {
//            return (true);
//        } else return (false);
//    }
//
//    public boolean DriveDone(double distance){
//        if (
//                (Math.abs(robot.verticalLeft.getCurrentPosition() /*/ robot.OdometryCountsPerInch*/) >= distance
//                        || Math.abs(robot.verticalRight.getCurrentPosition() /*/ robot.OdometryCountsPerInch*/) >= distance)
//        )
//        {
//            return (true);
//        }
//        else return (false);
//    }

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

    public void KillDrive(){
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public void setDrivePower (double rightFront, double leftFront, double leftBack, double rightBack){
        robot.frontLeft.setPower(leftFront);
        robot.frontRight.setPower(rightFront);
        robot.backLeft.setPower(leftBack);
        robot.backRight.setPower(rightBack);
    }

    /*
     * Method calcRPM()
     */
    public double calcRPM(double tick0, double time0, double tick1, double time1){
        double rPM = (((Math.abs(tick0-tick1)/robot.COUNTS_PER_ROTATION))/(Math.abs(time1-time0)))*60;
        return (rPM);
    }   // close calcRPM method

    /*
     * Method shooterPower()
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

    public double shooterPowerForAuto(double shooterPower, double currentRPM, double targetRPM){

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
     * Method gyroPositive
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
     * Method gyroNegative
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
     * Method: PIDRotate
     * Parameters:
     *      targetAngle -> desire ending angle/position of the robot
     *      targetError -> how close should the robot get to the desired angle
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

//            updateValues("PIDRotate", targetAngle, lastError, error, derivative);
//            opMode.telemetry.addData("Current First Angle", getZAngle());
//            opMode.telemetry.addData("Positive Angle = ", gyroPositive(targetAngle));
//            opMode.telemetry.addData("Negative Angle = ", gyroNegative(targetAngle));
//            opMode.telemetry.update();

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

    public void Kill(){
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     Checks to see if the motors are running
     and if they're not it will return true
     */
    public boolean IsBusy(){
        if (!robot.frontLeft.isBusy() || !robot.backLeft.isBusy() || !robot.frontRight.isBusy() || !robot.backRight.isBusy())
        {
            return (true);
        } else return (false);
    }

    public boolean isStopped(){
        if (robot.frontLeft.getPower() == 0 || robot.backLeft.getPower() == 0 || robot.frontRight.getPower() == 0 || robot.backRight.getPower() == 0) {
            return (true);
        } else return (false);
    }

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

    public boolean StrafeDone(double distance){
        if (Math.abs(robot.frontLeft.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance
                || Math.abs(robot.backRight.getCurrentPosition() / robot.COUNTS_PER_INCH) >= distance)
        {
            return (true);
        } else return (false);
    }

    public boolean StrafeBetter(double distance){
        if (Math.abs(robot.horizontal.getCurrentPosition() / robot.COUNTS_PER_INCH2) >= distance)
        {
            return (true);
        } else return (false);
    }

    public String formatAngle (AngleUnit angleUnit, double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees (double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
