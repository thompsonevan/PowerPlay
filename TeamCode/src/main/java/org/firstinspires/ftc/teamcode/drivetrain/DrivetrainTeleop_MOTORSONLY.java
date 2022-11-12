
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drivetrain Teleop (MOTORS ONLY)")
//@Disabled
public class DrivetrainTeleop_MOTORSONLY extends LinearOpMode {

    @Override
    public void runOpMode() {

        DrivetrainCommon_ALT2_MOTORS_ONLY drivetrain = new DrivetrainCommon_ALT2_MOTORS_ONLY(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drivetrain.executeTeleop();

            telemetry.update();
        }
    }

}

