package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class SensorsCommon {

    public static double leftVal;
    public static double leftValBU;
    public static double rightVal;
    public static double rightValBU;
    public static double centerVal;
    public static double centerValBU;

    public static void initSensorsCommon()
    {
        SensorsHardware.initSensorsHardware();


    }

    public static void updateDistanceValues()
    {

        leftValBU=300;
        rightValBU=300;
        centerValBU=300;

        leftVal = SensorsHardware.leftConeCheck.getDistance(DistanceUnit.INCH);
        if(leftVal>300) {
            leftVal= SensorsHardware.leftConeCheckBU.getDistance(DistanceUnit.INCH);
        }
        rightVal = SensorsHardware.rightConeCheck.getDistance(DistanceUnit.INCH);
        if(rightVal>300) {
            rightVal = SensorsHardware.rightConeCheckBU.getDistance(DistanceUnit.INCH);
        }
        centerVal = SensorsHardware.centerCheck.getDistance(DistanceUnit.INCH);
        if(centerVal>300) {
            centerVal = SensorsHardware.centerCheckBU.getDistance(DistanceUnit.INCH);
            centerValBU = SensorsHardware.centerCheckBU.getDistance(DistanceUnit.INCH);
        }

        Robot.curOpMode.telemetry.addData("leftCheck:",leftVal);
        Robot.curOpMode.telemetry.addData("leftCheckBU:",leftValBU);
        Robot.curOpMode.telemetry.addData("rightCheck:",rightVal);
        Robot.curOpMode.telemetry.addData("rightCheckBU:",rightValBU);
        Robot.curOpMode.telemetry.addData("centerCheck:",centerVal);
        Robot.curOpMode.telemetry.addData("centerCheckBU:",centerValBU);


        Robot.curOpMode.telemetry.update();

    }
}
