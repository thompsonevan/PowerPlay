package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision;

/*
    Pos List:
    0 - Ground, picking up cone
    1 - Lifting cone off ground and before picking
    2 - Low
    3 - Medium
    4 - High
 */

@Autonomous(name="Right High", group="OnBot")

public class RightHigh extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;
    double liftSpeed = 1;

    int pos;

    @Override
    public void runOpMode() {
        auto = new AutoCommon(this, red);

        Vision camera = new Vision();
        camera.start(hardwareMap);

        while(!isStarted()){
            pos = camera.getPos();
        }

        auto.resetEncoders();

        auto.lift.closeClaw();

        auto.lift.goToPos(liftSpeed, 1, 10);

        auto.encoderStrafe(strafeSpeed, 10, 1300, true, false, false);

        auto.encoderDrive(driveSpeed,1250, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 590, true, false, false);

        auto.lift.goToPos(liftSpeed, 4, 10);

        auto.encoderDrive(driveSpeed, 300, 10, false);

        sleep(100);

        auto.lift.openClaw();

        sleep(100);

        auto.encoderDrive(driveSpeed, -300, 10, false);

        auto.lift.goToPos(liftSpeed, 1, 10);

        auto.encoderStrafe(strafeSpeed, 10, 600, false, false, false);

        auto.encoderDrive(driveSpeed, 1100, 10, false);

        auto.encoderTurn(.5, -760, 10);

        auto.driveToEnd(driveSpeed, pos);
    }
}