
package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.Robot.hwMap;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public final class DrivetrainHardware  {
    public static DcMotorEx driveRR  = null;
    public static DcMotorEx  driveLR  = null;
    public static DcMotorEx  driveLF  = null;
    public static DcMotorEx  driveRF  = null;

    public static BNO055IMU imu;

    public static DistanceSensor leftConeCheck;
    public static DistanceSensor rightConeCheck;
    public static DistanceSensor centerCheck;
    public static DistanceSensor junctionDriveDirCheck;

    public static DigitalChannel redLed;
    public static DigitalChannel greenLed;

    public static DigitalChannel redLed2;
    public static DigitalChannel greenLed2;

    public static DigitalChannel redLedC1;
    public static DigitalChannel greenLedC1;

    public static DigitalChannel redLedC2;
    public static DigitalChannel greenLedC2;

    public static DigitalChannel redLedC3;
    public static DigitalChannel greenLedC3;

    public static DigitalChannel redLedC4;
    public static DigitalChannel greenLedC4;

    public static DigitalChannel redLedC5;
    public static DigitalChannel greenLedC5;

    public RevColorSensorV3 revColor;


    private static ElapsedTime period  = new ElapsedTime();

    public static BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public static void initDrivetrainHardware(){
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftConeCheck = hwMap.get(DistanceSensor.class, "leftConeCheck");
        rightConeCheck = hwMap.get(DistanceSensor.class, "rightConeCheck");
        junctionDriveDirCheck = hwMap.get(DistanceSensor.class, "junctionDriveDirCheck");
        centerCheck = hwMap.get(DistanceSensor.class, "centerCheck");

        redLed = hwMap.get(DigitalChannel.class, "red");
        greenLed = hwMap.get(DigitalChannel.class, "green");

        redLed2 = hwMap.get(DigitalChannel.class, "red2");
        greenLed2 = hwMap.get(DigitalChannel.class, "green2");

        redLedC1 =hwMap.get(DigitalChannel.class, "C1red");
        greenLedC1 = hwMap.get(DigitalChannel.class, "C1green");

        redLedC2 = hwMap.get(DigitalChannel.class, "C2red");
        greenLedC2 = hwMap.get(DigitalChannel.class, "C2green");

        redLedC3 = hwMap.get(DigitalChannel.class, "C3red");
        greenLedC3 = hwMap.get(DigitalChannel.class, "C3green");

        redLedC4 = hwMap.get(DigitalChannel.class, "C4red");
        greenLedC4 = hwMap.get(DigitalChannel.class, "C4green");

        redLedC5 = hwMap.get(DigitalChannel.class, "C5red");
        greenLedC5 = hwMap.get(DigitalChannel.class, "C5green");




        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);

        redLed2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed2.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC1.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC2.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC3.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC3.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC4.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC4.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC5.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC5.setMode(DigitalChannel.Mode.OUTPUT);


        redLedC1.setState(false);
        redLedC2.setState(false);
        redLedC3.setState(false);
        redLedC4.setState(false);
        redLedC5.setState(false);

        greenLedC1.setState(true);
        greenLedC2.setState(true);
        greenLedC3.setState(true);
        greenLedC4.setState(true);
        greenLedC5.setState(true);

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

