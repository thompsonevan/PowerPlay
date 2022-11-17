package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autons.AutoCommon;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;
import org.firstinspires.ftc.teamcode.lift.LiftClawCommon;
import org.firstinspires.ftc.teamcode.sensors.SensorsCommon;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Robot {

    public static LinearOpMode curOpMode;
    public static AutoCommon auton;
    public static Gamepad driver;
    public static Gamepad operator;
    public static HardwareMap hwMap;
    public static boolean hasLift;
    public static boolean hasDrivetrain;
    public static boolean hasVision;
    public static boolean hasSensors;
    public enum LoopStates {Running, Completed};

    public static LoopStates DrivetrainLoopState;
    public static LoopStates LiftClawLoopState;


    public static void init(boolean drivetrainAdded, boolean liftAdded,
                            boolean visionAdded, boolean autonAdded,
                            boolean sensorsAdded,
                            LinearOpMode opMode)
    {
        curOpMode=opMode;
        hwMap=opMode.hardwareMap;

        driver=curOpMode.gamepad1;
        operator=curOpMode.gamepad2;

        if(drivetrainAdded) {
            DrivetrainCommon_ALT1.initDrivetrainCommon_ALT1();
            hasDrivetrain = drivetrainAdded;
        }

        if(liftAdded) {
            LiftClawCommon.initLiftClawCommon();
            hasLift = liftAdded;
        }

        if(sensorsAdded)
        {
            SensorsCommon.initSensorsCommon();
        }

        if(autonAdded)
        {

        }
        hasVision=visionAdded;


    }
}
