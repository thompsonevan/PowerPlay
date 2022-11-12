
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DrivetrainHardware  {
    public DcMotorEx driveRR  = null;
    public DcMotorEx  driveLR  = null;
    public DcMotorEx  driveLF  = null;
    public DcMotorEx  driveRF  = null;

    public BNO055IMU imu;

    public DistanceSensor leftConeCheck;
    public DistanceSensor rightConeCheck;
    public DistanceSensor centerCheck;
    public DistanceSensor junctionDriveDirCheck;

    public DigitalChannel redLed;
    public DigitalChannel greenLed;

    public DigitalChannel redLed2;
    public DigitalChannel greenLed2;

    public DigitalChannel redLed3;
    public DigitalChannel greenLed3;

    public DigitalChannel redLed4;
    public DigitalChannel greenLed4;

    public RevColorSensorV3 revColor;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public DrivetrainHardware(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        leftConeCheck = hwMap.get(DistanceSensor.class, "leftConeCheck");
        rightConeCheck = hwMap.get(DistanceSensor.class, "rightConeCheck");
        junctionDriveDirCheck = hwMap.get(DistanceSensor.class, "junctionDriveDirCheck");
        centerCheck = hwMap.get(DistanceSensor.class, "centerCheck");


        redLed = hwMap.get(DigitalChannel.class, "red");
        greenLed = hwMap.get(DigitalChannel.class, "green");

        redLed2 = hwMap.get(DigitalChannel.class, "red2");
        greenLed2 = hwMap.get(DigitalChannel.class, "green2");

        redLed3 = hwMap.get(DigitalChannel.class, "red3");
        greenLed3 = hwMap.get(DigitalChannel.class, "green3");

        redLed4 = hwMap.get(DigitalChannel.class, "red4");
        greenLed4 = hwMap.get(DigitalChannel.class, "green4");

        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);

        redLed2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed2.setMode(DigitalChannel.Mode.OUTPUT);

        redLed3.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed3.setMode(DigitalChannel.Mode.OUTPUT);

        redLed4.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed4.setMode(DigitalChannel.Mode.OUTPUT);

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

        driveRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

