
package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1.executeDrivetrainTeleop;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.executeLiftClawTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Drivetrain Teleop")
@Disabled
public class DrivetrainTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot.init(true,false, false,this);
        waitForStart();

        while (opModeIsActive()) {

            executeDrivetrainTeleop();

            executeLiftClawTeleop();

        }
    }

}

