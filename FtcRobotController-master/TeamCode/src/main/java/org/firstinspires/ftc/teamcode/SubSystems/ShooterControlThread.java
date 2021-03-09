package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ShooterControlThread implements Runnable {

    private boolean isRunning = true;

    public DcMotor shooter1 = null;
    public DcMotor shooter2 = null;
    public ElapsedTime runtime = new ElapsedTime();
    public double currentTick = 0, currentTick2 = 0;
    public double lastTick = 0, lastTick2 = 0;
    public double currentShooterPower = 0, currentShooterPower2 = 0;
    public double currentTime;
    public double lastTime = 0;
    public double targetShooterRPM = 0, targetShooterRPM2 = 0;

    public final double SHOOTER_TICKS_PER_ROTATION = 28;

    public ShooterControlThread(DcMotor motorShooter1, DcMotor motorShooter2){
        this.shooter1 = motorShooter1;
        this.shooter2 = motorShooter2;
    }//end of constructor

    /**
-      Method controlShooter
-      controls speed of shooter motors while programming is running
     */
    public void controlShooter(){
        this.currentShooterPower = shooterPower();
        setShooterPower(this.currentShooterPower);
    }

    public void controlShooter2(){
        this.currentShooterPower2 = shooterPower();
        setShooterPower2(this.currentShooterPower2);
    }

    private void setShooterPower(double powerLevel){
        this.shooter1.setPower(powerLevel);
    }

    private void setShooterPower2(double powerLevel2){
        this.shooter1.setPower(powerLevel2);
    }

    public void setRPMMeasurements(){
        double tempValue = 0;
        this.lastTime = this.currentTime;
        this.lastTick = this.currentTick;
        this.lastTick2 = this.currentTick2;

        for (int i = 0; i< 200000; i++){
            tempValue = tempValue + i;
        }

        this.currentTick = (shooter1.getCurrentPosition());
        this.currentTick2 = (shooter2.getCurrentPosition());
        this.currentTime = runtime.time();

    }

    public void setTargetShooterRPM(double targetRPM, double targetRPM2){
        this.targetShooterRPM = targetRPM;
        this.targetShooterRPM2 = targetRPM2;
    }

    public double measureRPM(){
        setRPMMeasurements();
        double rPM = (((Math.abs(this.currentTick - this.lastTick))/this.SHOOTER_TICKS_PER_ROTATION)/(Math.abs(this.currentTime - this.lastTime)))*60;

        return rPM;
    }

    public double measureRPM2(){
        setRPMMeasurements();
        double rPM = (((Math.abs(this.currentTick2 - this.lastTick2))/this.SHOOTER_TICKS_PER_ROTATION)/(Math.abs(this.currentTime - this.lastTime)))*60;

        return rPM;
    }

    public double shooterPower(){
        double shooterPower = this.currentShooterPower;
        double error = this.targetShooterRPM - measureRPM();
        double Kp = 0.0000015;
        double Ki = 0.0003;
        double Kd = 0.0001;
        double maxPower = 1;
        double derrivative = 0;
        double integral = 0;

        if (this.targetShooterRPM == 0){
            shooterPower = 0;
        } else if (this.targetShooterRPM != 0 && shooterPower == 0){
            shooterPower = .50;
        } else {
            shooterPower =  shooterPower + ((Kp*error) + (Ki * integral)+(Kd*derrivative));
        }

        return (Range.clip(shooterPower, 0,maxPower));
    }

    public double shooterPower2(){
        double shooterPower2 = this.currentShooterPower2;
        double error = this.targetShooterRPM2 - measureRPM2();
        double Kp = 0.0000015;
        double Ki = 0.0003;
        double Kd = 0.0001;
        double maxPower = 1;
        double derrivative = 0;
        double integral = 0;

        if (this.targetShooterRPM == 0){
            shooterPower2 = 0;
        } else if (this.targetShooterRPM != 0 && shooterPower2 == 0){
            shooterPower2 = .50;
        } else {
            shooterPower2 =  shooterPower2 + ((Kp*error) + (Ki * integral)+(Kd*derrivative));
        }

        return (Range.clip(shooterPower2, 0,maxPower));
    }

    public void stopShooterThread(){
        this.isRunning = false;
    }



    @Override
    public void run(){
        while (isRunning){
            controlShooter();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e){
                e.printStackTrace();
            }//end of try - catch

        }//end of while(isRunning)

    }//end of void run()

}
