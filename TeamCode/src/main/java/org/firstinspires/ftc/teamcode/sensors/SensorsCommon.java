package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class SensorsCommon {

    public static double leftVal;
    public static double rightVal;
    public static double centerVal;

    public static void initSensorsCommon()
    {
        SensorsHardware.initSensorsHardware();


    }

    public static void updateDistanceValues()
    {
        leftVal = SensorsHardware.leftConeCheck.getDistance(DistanceUnit.INCH);
        rightVal = SensorsHardware.rightConeCheck.getDistance(DistanceUnit.INCH);
        centerVal = SensorsHardware.centerCheck.getDistance(DistanceUnit.INCH);

        Robot.curOpMode.telemetry.addData("leftCheck:",leftVal);
        Robot.curOpMode.telemetry.addData("rightCheck:",rightVal);
        Robot.curOpMode.telemetry.addData("centerCheck:",centerVal);

        Robot.curOpMode.telemetry.update();

    }
}
