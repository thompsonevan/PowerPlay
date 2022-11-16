
package org.firstinspires.ftc.teamcode.lift;

import static org.firstinspires.ftc.teamcode.Robot.hwMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


public final class LiftClawHardware
{
    /* Public OpMode members. */

    public static DcMotorEx  lift  = null;
    public static Servo    claw   = null;
    public static CRServo leftIntake;
    public static CRServo rightIntake;

    public static final double MID_SERVO       =  0.3 ;

    /* local OpMode members. */
    //HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    static void initLiftClawHardware() {
        // Save reference to Hardware map


        // Define and Initialize Motors

        lift = (DcMotorEx) hwMap.get(DcMotor.class, "lift");


        lift.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        lift.setPower(0);

        // Set motor to run with encoder
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        claw  = hwMap.get(Servo.class, "claw");

    }
}
