package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagAutonomousInitTest;

@Autonomous(name="Scan Test", group="OnBot")

public class TestForScan extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    AutoCommon auto=null;

    boolean red=false;

    double driveSpeed = .5;
    double strafeSpeed = .5;

    AprilTagAutonomousInitTest april = new AprilTagAutonomousInitTest();

    int pos;

    @Override
    public void runOpMode() {
//        AutoCommon.resetEncoders();

        april.startVision(hardwareMap);

        while(!isStarted() && !isStopRequested()){
            april.runVision();
            pos = april.getPos();
            telemetry.addData("ID", pos);
            telemetry.update();
        }

        waitForStart();
    }
}