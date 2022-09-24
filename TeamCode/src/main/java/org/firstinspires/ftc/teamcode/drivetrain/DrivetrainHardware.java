
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class DrivetrainHardware
{
    /* Public OpMode members. */

    public DcMotor  driveRR  = null;
    public DcMotor  driveLR  = null;
    public DcMotor  driveLF  = null;
    public DcMotor  driveRF  = null;

    public BNO055IMU imu;

    public static final double MID_SERVO       =  0.3 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public DrivetrainHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        //Define and initialize imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and Initialize Motors
        driveRR = hwMap.get(DcMotor.class, "drive_RR");
        driveLR = hwMap.get(DcMotor.class, "drive_LR");
        driveLF = hwMap.get(DcMotor.class, "drive_LF");
        driveRF = hwMap.get(DcMotor.class, "drive_RF");

        driveRR.setDirection(DcMotor.Direction.FORWARD);
        driveLR.setDirection(DcMotor.Direction.REVERSE);
        driveLF.setDirection(DcMotor.Direction.REVERSE);
        driveRF.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        driveRR.setPower(0);
        driveLR.setPower(0);
        driveLF.setPower(0);
        driveRF.setPower(0);

        // Set motor to run with encoder
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

