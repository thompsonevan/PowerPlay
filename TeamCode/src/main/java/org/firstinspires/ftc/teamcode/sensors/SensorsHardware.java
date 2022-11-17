package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.Robot.hwMap;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public final class SensorsHardware {

    public static DistanceSensor leftConeCheck;
    public static DistanceSensor rightConeCheck;
    public static DistanceSensor centerCheck;
    public static DistanceSensor junctionDriveDirCheck;

    public static DigitalChannel redLed;
    public static DigitalChannel greenLed;

    public static DigitalChannel redLed2;
    public static DigitalChannel greenLed2;

    public static DigitalChannel redLedC1;
    public static DigitalChannel greenLedC1;

    public static DigitalChannel redLedC2;
    public static DigitalChannel greenLedC2;

    public static DigitalChannel redLedC3;
    public static DigitalChannel greenLedC3;

    public static DigitalChannel redLedC4;
    public static DigitalChannel greenLedC4;

    public static DigitalChannel redLedC5;
    public static DigitalChannel greenLedC5;
    public static void initSensorsHardware()
    {
        leftConeCheck = hwMap.get(DistanceSensor.class, "leftConeCheck");
        rightConeCheck = hwMap.get(DistanceSensor.class, "rightConeCheck");
        junctionDriveDirCheck = hwMap.get(DistanceSensor.class, "junctionDriveDirCheck");
        centerCheck = hwMap.get(DistanceSensor.class, "centerCheck");

        redLed = hwMap.get(DigitalChannel.class, "red");
        greenLed = hwMap.get(DigitalChannel.class, "green");

        redLed2 = hwMap.get(DigitalChannel.class, "red2");
        greenLed2 = hwMap.get(DigitalChannel.class, "green2");

        redLedC1 =hwMap.get(DigitalChannel.class, "C1red");
        greenLedC1 = hwMap.get(DigitalChannel.class, "C1green");

        redLedC2 = hwMap.get(DigitalChannel.class, "C2red");
        greenLedC2 = hwMap.get(DigitalChannel.class, "C2green");

        redLedC3 = hwMap.get(DigitalChannel.class, "C3red");
        greenLedC3 = hwMap.get(DigitalChannel.class, "C3green");

        redLedC4 = hwMap.get(DigitalChannel.class, "C4red");
        greenLedC4 = hwMap.get(DigitalChannel.class, "C4green");

        redLedC5 = hwMap.get(DigitalChannel.class, "C5red");
        greenLedC5 = hwMap.get(DigitalChannel.class, "C5green");




        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);

        redLed2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed2.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC1.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC2.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC3.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC3.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC4.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC4.setMode(DigitalChannel.Mode.OUTPUT);

        redLedC5.setMode(DigitalChannel.Mode.OUTPUT);
        greenLedC5.setMode(DigitalChannel.Mode.OUTPUT);


        redLedC1.setState(true);
        redLedC2.setState(true);
        redLedC3.setState(true);
        redLedC4.setState(true);
        redLedC5.setState(true);

        greenLedC1.setState(true);
        greenLedC2.setState(true);
        greenLedC3.setState(true);
        greenLedC4.setState(true);
        greenLedC5.setState(true);

    }
}
