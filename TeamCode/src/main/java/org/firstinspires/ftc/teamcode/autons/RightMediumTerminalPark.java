package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Right Medium Terminal Park", group="OnBot")

public class RightMediumTerminalPark extends LinearOpMode {
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

        auto.encoderStrafe(strafeSpeed, 10, 1150, true, false, false);

        auto.encoderDrive(driveSpeed, 1000, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 600, false, false, false);

        auto.encoderDrive(driveSpeed, 200, 10, false);

        auto.encoderDrive(driveSpeed, -200, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 500, true, false, false);

        auto.encoderDrive(driveSpeed, -900, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 2200, false, false, false);



    }
}