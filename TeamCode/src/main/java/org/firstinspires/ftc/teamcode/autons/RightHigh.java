package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision;

@Autonomous(name="Right High", group="OnBot")

public class RightHigh extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;
    double driveSpeedslow = .05;
    int drivedistance1block = 1000;
    int distancestrafe1block = 1200;

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

//        auto.encoderDrive(driveSpeed, 200, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 1300, true, false, false);

        auto.encoderDrive(driveSpeed,1250, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 600, true, false, false);

        auto.encoderDrive(driveSpeed, 150, 10, false);

        auto.encoderDrive(driveSpeed, -150, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 600, false, false, false);

        auto.encoderDrive(driveSpeed, 1100, 10, false);

        auto.encoderTurn(.25, -730, 10);

        auto.driveToEnd(driveSpeed, pos);
    }
}