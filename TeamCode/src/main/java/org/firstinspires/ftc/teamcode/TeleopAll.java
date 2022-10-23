package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon;

@TeleOp(name="Blue Teleop All", group="Pushbot")
//@Disabled

public class TeleopAll extends LinearOpMode {

    DrivetrainCommon drivetrain;
//    LiftClawCommon liftClaw;

    @Override
    public void runOpMode() {
        drivetrain = new DrivetrainCommon(this);
//        liftClaw = new LiftClawCommon(this);

//        liftClaw.chassis=drivetrain;
        waitForStart();

        while (opModeIsActive()) {
//                oldDrivetrain.executeTeleop();
            drivetrain.executeTeleop();
//            liftClaw.executeTeleop();
            telemetry.update();
        }
    }

}