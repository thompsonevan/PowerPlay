
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class DrivetrainHardware  {
    public DcMotorEx driveRR  = null;
    public DcMotorEx  driveLR  = null;
    public DcMotorEx  driveLF  = null;
    public DcMotorEx  driveRF  = null;

    public BNO055IMU imu;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public DrivetrainHardware(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        driveRR = hwMap.get(DcMotorEx.class, "drive_RR");
        driveLR = hwMap.get(DcMotorEx.class, "drive_LR");
        driveLF = hwMap.get(DcMotorEx.class, "drive_LF");
        driveRF = hwMap.get(DcMotorEx.class, "drive_RF");

        driveRR.setDirection(DcMotor.Direction.FORWARD);
        driveLR.setDirection(DcMotor.Direction.REVERSE);
        driveLF.setDirection(DcMotor.Direction.REVERSE);
        driveRF.setDirection(DcMotor.Direction.FORWARD);

        driveRR.setPower(0);
        driveLR.setPower(0);
        driveLF.setPower(0);
        driveRF.setPower(0);

        driveRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}

