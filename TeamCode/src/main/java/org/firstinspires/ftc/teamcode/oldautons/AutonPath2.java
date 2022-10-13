package org.firstinspires.ftc.teamcode.oldautons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autons.AutoCommon;

@Autonomous(name="AutonPath2", group="OnBot")
@Disabled
public class AutonPath2 extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .3;
    double strafeSpeed = .5;
    double driveSpeedslow = .05;
    int drivedistance1block = 2000;


    @Override
    public void runOpMode() {
        auto = new AutoCommon(this, red);

        auto.resetEncoders();

        auto.encoderDrive(driveSpeed, 1200, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 1800, true, false, false);

        auto.encoderDrive(driveSpeedslow, 200, 10, false);

        auto.encoderDrive(driveSpeedslow, -200, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 600, true, false, false);

        auto.encoderDrive(driveSpeed, drivedistance1block, 10, false);

        auto.encoderTurn(.25, -730, 10);

        auto.encoderDrive(driveSpeed, drivedistance1block, 10, false);

        auto.encoderTurn(.25, -730, 10);

        auto.encoderDrive(driveSpeed, 2500, 10, false);
    }
}