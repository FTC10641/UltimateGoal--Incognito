package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Sensors extends HardWareMap {

        public BNO055IMU imu;

        public Orientation angles;
        WebcamName webcamName = null;



        HardwareMap hwMap = null;

        public void initSensors(HardwareMap ahwMap) {
            hwMap = ahwMap;

            imu = ahwMap.get(BNO055IMU.class, "imu");
            webcamName = ahwMap.get(WebcamName.class, "Webcam 1");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);

        }

    public double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }


}

