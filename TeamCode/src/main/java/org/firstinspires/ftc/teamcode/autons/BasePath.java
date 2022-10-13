package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Base", group="OnBot")

public class BasePath extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;

    @Override
    public void runOpMode() {
        auto = new AutoCommon(this, red);

        auto.resetEncoders();

        auto.encoderDrive(driveSpeed, 1200, 10, false);

        auto.encoderTurn(.5, -00, 10);

        auto.encoderStrafe(strafeSpeed, 10, 1200, true, false, false);

        auto.encoderDrive(driveSpeed, 1000, 10, false);

        auto.encoderTurn(.5, 775, 10);

        auto.encoderDrive(driveSpeed, 1100, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 2500, true, false, false);

    }
}