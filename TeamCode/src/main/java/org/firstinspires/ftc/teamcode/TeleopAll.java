package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;
import org.firstinspires.ftc.teamcode.lift.LiftClawCommon;

@TeleOp(name="Blue Teleop All", group="Pushbot")
//@Disabled

public class TeleopAll extends LinearOpMode {

    DrivetrainCommon_ALT1 drivetrain;
    LiftClawCommon liftClaw;

    @Override
    public void runOpMode() {
        drivetrain = new DrivetrainCommon_ALT1(this);
       liftClaw = new LiftClawCommon(this);

       liftClaw.chassis=drivetrain;
       drivetrain.liftClaw = liftClaw;

        waitForStart();

        while (opModeIsActive()) {
//                oldDrivetrain.executeTeleop();
            drivetrain.executeTeleop();
           liftClaw.executeTeleop();
            telemetry.update();
        }
    }

}