
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon;

@TeleOp(name="Drivetrain Teleop")
@Disabled
public class DrivetrainTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        DrivetrainCommon drivetrain = new DrivetrainCommon(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drivetrain.executeTeleop();

            telemetry.update();
        }
    }

}
