package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision;

@Autonomous(name="Right High Terminal Park", group="OnBot")

public class RightHighTerminalPark extends LinearOpMode {
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

        Vision camera = new Vision();
        camera.start(hardwareMap);

        int pos;
        while(!isStarted()){
            pos = camera.getPos();
        }

        auto.resetEncoders();

        auto.encoderDrive(driveSpeed, 100, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 1150, true, false, false);

        auto.encoderDrive(driveSpeed, 1000, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 700, true, false, false);

        auto.encoderDrive(driveSpeed, 200, 10, false);

        auto.encoderDrive(driveSpeed, -200, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 700, false, false, false);

        auto.encoderDrive(driveSpeed, -900, 10, false);

        auto.encoderStrafe(strafeSpeed, 10, 2200, false, false, false);



    }
}