package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon;

@TeleOp(name="Blue Teleop All", group="Pushbot")
//@Disabled

public class TeleopAll extends LinearOpMode {

    DrivetrainCommon drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new DrivetrainCommon(this);

        waitForStart();

        while (opModeIsActive()) {
//                oldDrivetrain.executeTeleop();
            drivetrain.executeTeleop();

            telemetry.update();
        }
    }

}
