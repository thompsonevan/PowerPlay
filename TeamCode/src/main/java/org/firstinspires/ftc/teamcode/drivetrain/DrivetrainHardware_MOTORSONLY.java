
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DrivetrainHardware_MOTORSONLY {
    public DcMotor  driveRR  = null;
    public DcMotor  driveLR  = null;
    public DcMotor  driveLF  = null;
    public DcMotor  driveRF  = null;

    public BNO055IMU imu;



    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public DrivetrainHardware_MOTORSONLY(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);




        driveRR = hwMap.get(DcMotor.class, "drive_RR");
        driveLR = hwMap.get(DcMotor.class, "drive_LR");
        driveLF = hwMap.get(DcMotor.class, "drive_LF");
        driveRF = hwMap.get(DcMotor.class, "drive_RF");

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

