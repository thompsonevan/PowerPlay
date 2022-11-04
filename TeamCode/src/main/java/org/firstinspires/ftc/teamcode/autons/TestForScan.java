package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Scan Test", group="OnBot")

public class TestForScan extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;

    @Override
    public void runOpMode() {

        auto = new AutoCommon(this, red);

        auto.resetEncoders();

        waitForStart();

//        auto.isDone = false;
//
//        sleep(1000);
//
//        auto.encoderStrafe(.2, 10, 750, false, false, false);
//
        while(!auto.scanForPole(true));
    }
}