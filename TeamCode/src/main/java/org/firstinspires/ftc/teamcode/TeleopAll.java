package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1.executeDrivetrainTeleop;
import static org.firstinspires.ftc.teamcode.lift.LiftClawCommon.executeLiftClawTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;
import org.firstinspires.ftc.teamcode.lift.LiftClawCommon;

@TeleOp(name="TeleOp", group="Pushbot")
//@Disabled

public class TeleopAll extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot.DrivetrainLoopState = Robot.LoopStates.Completed;
        Robot.init(true,true,false,this);
        waitForStart();

        while (opModeIsActive()) {

//            if(Robot.DrivetrainLoopState == Robot.LoopStates.Completed) {
                executeDrivetrainTeleop();
//            }


            executeLiftClawTeleop();

            Robot.curOpMode.telemetry.update();
        }
    }

}