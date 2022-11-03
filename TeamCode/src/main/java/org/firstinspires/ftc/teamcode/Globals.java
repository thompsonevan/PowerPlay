package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;
import org.firstinspires.ftc.teamcode.lift.LiftClawCommon;

public class Globals {

    public static LinearOpMode curOpMode;
    public static DrivetrainCommon_ALT1 drivetrain;
    public static LiftClawCommon liftClaw;

    public Globals(boolean hasDrivetrain, boolean hasLift, boolean hasVision, LinearOpMode opMode)
    {
        curOpMode=opMode;

        if(hasDrivetrain) {
            drivetrain = new DrivetrainCommon_ALT1(curOpMode);
        }

        if(hasLift) {
            liftClaw = new LiftClawCommon(curOpMode);
        }

        if(hasVision)
        {

        }

    }
}
