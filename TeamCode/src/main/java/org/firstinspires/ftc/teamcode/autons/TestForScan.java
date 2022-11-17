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

     
        AutoCommon.resetEncoders();

        waitForStart();

//        AutoCommon.isDone = false;
//
//        sleep(1000);
//
//        AutoCommon.encoderStrafe(.2, 10, 750, false, false, false);
//
        while(!AutoCommon.scanForPole(true));
    }
}