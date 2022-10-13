package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robotboogie", group="OnBot")

public class Robotboogie extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .3;
    double strafeSpeed = .5;
    double driveSpeedslow = .05;
    int drivedistance1block = 1000;
    int distancestrafe1block = 1200;

    @Override
    public void runOpMode() {
        auto = new AutoCommon(this, red);

        auto.resetEncoders();

        auto.encoderDrive(driveSpeed, 100, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 1300, true, false, false);

        auto.encoderDrive(driveSpeed, 2700, 10, false);

        auto.encoderTurn(.25, 730, 10);

        auto.encoderDrive(driveSpeed, 700, 10, false);

        auto.encoderTurn(.5, 5000, 10);

        auto.encoderDrive(driveSpeed, 500, 10, false);

        auto.encoderDrive(driveSpeed, -500, 10, false);

        auto.encoderTurn(.5, 2000, 10);




    }
}