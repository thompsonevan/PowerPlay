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

        auto.encoderDrive(driveSpeed, 750, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 1200, false, false, false);

        auto.encoderTurn(.5, 600, 30);

        auto.encoderStrafe(strafeSpeed, 10, 1200, false, false, false);

        auto.encoderDrive(driveSpeed, -750, 10, false);

        auto.encoderTurn(.5, -600, 30);

        auto.encoderDrive(driveSpeed, -550, 10, false);

        auto.encoderTurn(.5, -650, 30);

        auto.encoderDrive(driveSpeed, -1750, 10, false);

        auto.encoderTurn(.5, 500, 30);

        auto.encoderDrive(driveSpeed, -6550, 10, false);

        auto.encoderTurn(.5, -700, 30);

        auto.encoderDrive(driveSpeed, -3050, 10, false);
    }
}