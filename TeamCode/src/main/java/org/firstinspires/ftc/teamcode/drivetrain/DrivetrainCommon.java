
package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.PIDController;

public class DrivetrainCommon {

    public DrivetrainHardware robot = new DrivetrainHardware();

    // LiftClawHardware(); /* HT 15358 */

    // LiftClawCommon(); /* HT 15358 */

    private double servoValue = 0;

    private LinearOpMode curOpMode = null;

    Orientation angles;

    Orientation lastAngles = new Orientation();
    public double globalAngle, currentAngle, power = .30, correction, rotation;
    public PIDController pidRotate, pidDrive;

    double powerRightRear;
    double powerLeftRear;
    double powerLeftFront;
    double powerRightFront;


    double yVal = 0;
    double xVal = 0;

    double xVal_rs=0;

    double turnVal = 0;
    double diagVal = 0;


    double min =0 ;
    double setPointAngle = 0;

    double slowPower = .25;
    double slideSlowPower = .25;
    double slowTurn =.2;

    double max = .8;
    double turnMax = .8;

    Gamepad driverControl;


    // Important Stuff:
    double maxSpeed = 1;
    double controllerCap = .95;
    double speed = .75;

    double powerShift = .15;
    // Important Stuff

    public DrivetrainCommon(LinearOpMode owningOpMode) {
        curOpMode = owningOpMode;
        initHardware();

        pidRotate = new PIDController(0, 0, 0);
        pidDrive = new PIDController(.05, 0, 0);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(setPointAngle);
        pidDrive.setOutputRange(0, .9);
        pidDrive.setInputRange(-180, 180);
        pidDrive.enable();

        resetAngle();

        curOpMode.telemetry.addData("Mode", "calibrating...");
        curOpMode.telemetry.update();


        //    while (!curOpMode.isStopRequested() && !robot.imu.isGyroCalibrated()) {
        //        curOpMode.sleep(50);
        //       curOpMode.idle();
        //   }

        composeTelemetry();
        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        curOpMode.telemetry.update();
    }

    public void executeTeleop() {


        xVal = curOpMode.gamepad1.left_stick_x;
        yVal = -curOpMode.gamepad1.left_stick_y;

        xVal_rs = curOpMode.gamepad1.right_stick_x;

        correction = 0;

        boolean yDir = false;

        if (Math.abs(yVal) > Math.abs(xVal)) {

            yDir = true;
        }

        if (curOpMode.gamepad1.x) {
            robot.driveLF.setPower(-.35);
            robot.driveRF.setPower(.35);
            robot.driveLR.setPower(.35);
            robot.driveRR.setPower(-.35);
        } else {

        //Maintain minimum power value whenever stick is above zero
        if (Math.abs(yVal) > 0 && Math.abs(yVal) < min) {

            yVal = Math.signum(yVal) * min;
        }

        if (Math.abs(xVal) > 0 && Math.abs(xVal) < min) {

            xVal = Math.signum(xVal) * min;
        }

        if (Math.abs(xVal_rs) > 0 && Math.abs(xVal_rs) < min) {

            xVal_rs = Math.signum(xVal_rs) * min;
        }

        //Scale power based on exponential curve

        if (Math.abs(yVal) > 0) {
            yVal = Math.signum(yVal) * (Math.pow(Math.abs(yVal), 4) + powerShift);
        }

        if (Math.abs(xVal) > 0) {
            xVal = Math.signum(xVal) * (Math.pow(Math.abs(xVal), 4) + powerShift);
        }

        if (Math.abs(xVal_rs) > 0) {
            xVal_rs = Math.signum(xVal_rs) * (Math.pow(Math.abs(xVal_rs), 4) + powerShift);
        }


        if (Math.abs(yVal) > 1) {
            yVal = Math.signum(yVal) * 1;
        }

        if (Math.abs(xVal) > 1) {
            xVal = Math.signum(xVal) * 1;
        }

        if (Math.abs(xVal_rs) > 1) {
            xVal_rs = Math.signum(xVal_rs) * 1;
        }

        //Scale resulting curve values to defined maximum
        yVal = yVal * max;
        xVal = xVal * max;
        xVal_rs = xVal_rs * turnMax;

        double leftTurnCorrection = 0;
        double rightTurnCorrection = 0;


        if (Math.abs(xVal_rs) > 0) {

            //This case is for a pure turn with no forward/backward or slide L/R motion
            if (Math.abs(yVal) == 0 && Math.abs(xVal) == 0) {

                if (Math.abs(xVal_rs) > 0) {

                    turnVal = xVal_rs;

                    powerRightRear = -turnVal;
                    powerLeftRear = turnVal;
                    powerLeftFront = turnVal;
                    powerRightFront = -turnVal;

                    robot.driveRR.setPower(powerRightRear);
                    robot.driveLR.setPower(powerLeftRear);
                    robot.driveLF.setPower(powerLeftFront);
                    robot.driveRF.setPower(powerRightFront);
                } else if (Math.abs(curOpMode.gamepad1.right_stick_x) == 0) {

                    robot.driveRR.setPower(0);
                    robot.driveLR.setPower(0);
                    robot.driveLF.setPower(0);
                    robot.driveRF.setPower(0);
                }

            }

            //Otherwise capture a turn value to cause simultaneous turning while moving forward/backward or sliding left/right
            else {

                turnVal = xVal_rs;

                if (turnVal > 0) {
                    rightTurnCorrection = turnVal;
                } else {
                    leftTurnCorrection = turnVal;
                }
            }

            rotation = getAngle();        // reset angle tracking on new heading.
            resetAngle();
        }


        //Drive forward
        if (yVal > 0 && yDir) {


            //Left motors
            powerLeftRear = yVal + leftTurnCorrection;
            powerLeftFront = yVal + leftTurnCorrection;

            //Right Motors
            powerRightRear = yVal - rightTurnCorrection;
            powerRightFront = yVal - rightTurnCorrection;


        }
        //Drive backward
        else if (yVal < 0 && yDir) {
            //Left motors
            powerLeftRear = yVal + rightTurnCorrection;
            powerLeftFront = yVal + rightTurnCorrection;

            //Right Motors
            powerRightRear = yVal - leftTurnCorrection;
            powerRightFront = yVal - leftTurnCorrection;


        }

        //Slide Left/Right with no correction curve
        else if (Math.abs(xVal) > 0 && Math.abs(curOpMode.gamepad1.right_stick_x) == 0) {

            correction = 0;//pidDrive.performPID(getAngle());//*Math.max(Math.abs(yVal),Math.abs(xVal));
            //Front Motors
            powerLeftFront = xVal - correction;
            powerRightFront = -xVal + correction;

            //Rear Motors
            powerRightRear = xVal + correction;
            powerLeftRear = -xVal - correction;


        }
        //Slide Left/Right with correction causing curved motion
        else if (Math.abs(xVal) > 0 && Math.abs(xVal_rs) > 0) {

            correction = xVal_rs;

            //Slide Left with forward curve
            if (xVal < 0 && xVal_rs > 0) {
                //Front Motors
                powerLeftFront = xVal + correction;
                powerRightFront = -xVal - correction;

                //Rear Motors
                powerRightRear = xVal;
                powerLeftRear = -xVal;
            }
            //Slide Left with backward curve
            else if (xVal < 0 && xVal_rs < 0) {
                //Front Motors
                powerLeftFront = xVal;
                powerRightFront = -xVal;

                //Rear Motors
                powerRightRear = xVal - correction;
                powerLeftRear = -xVal + correction;
            }
            //Slide Right with backward curve
            else if (xVal > 0 && xVal_rs > 0) {
                //Front Motors
                powerLeftFront = xVal;
                powerRightFront = -xVal;

                //Rear Motors
                powerRightRear = xVal - correction;
                powerLeftRear = -xVal + correction;
            }
            //Slide right with forward curve
            else if (xVal > 0 && xVal_rs < 0) {
                //Front Motors
                powerLeftFront = xVal + correction;
                powerRightFront = -xVal - correction;

                //Rear Motors
                powerRightRear = xVal;
                powerLeftRear = -xVal;
            }
        } else if (Math.abs(xVal_rs) == 0) {
            powerRightRear = 0;
            powerLeftRear = 0;
            powerLeftFront = 0;
            powerRightFront = 0;
        }


        //Slow controls for forward/backward and slide left/right

        //Slow forward
        if (curOpMode.gamepad1.dpad_up) {

            if (curOpMode.gamepad1.dpad_left) {
                //Left motors
                powerLeftRear = slowPower;
                powerLeftFront = 0;

                //Right Motors
                powerRightRear = 0;
                powerRightFront = slowPower;
            } else if (curOpMode.gamepad1.dpad_right) {
                //Left motors
                powerLeftRear = 0;
                powerLeftFront = slowPower;

                //Right Motors
                powerRightRear = slowPower;
                powerRightFront = 0;
            } else {
                //Left motors
                powerLeftRear = slowPower;
                powerLeftFront = slowPower;

                //Right Motors
                powerRightRear = slowPower;
                powerRightFront = slowPower;
            }
        }
        //Slow backward
        else if (curOpMode.gamepad1.dpad_down) {

            if (curOpMode.gamepad1.dpad_right) {
                //Left motors
                powerLeftRear = -slowPower;
                powerLeftFront = 0;

                //Right Motors
                powerRightRear = 0;
                powerRightFront = -slowPower;
            } else if (curOpMode.gamepad1.dpad_left) {
                //Left motors
                powerLeftRear = 0;
                powerLeftFront = -slowPower;

                //Right Motors
                powerRightRear = -slowPower;
                powerRightFront = 0;
            } else {


                //Left motors
                powerLeftRear = -slowPower;
                powerLeftFront = -slowPower;

                //Right Motors
                powerRightRear = -slowPower;
                powerRightFront = -slowPower;
            }
        }
        //Slow slide left
        else if (curOpMode.gamepad1.dpad_left) {

            //Front Motors
            powerLeftFront = -slideSlowPower;
            powerRightFront = slideSlowPower;

            //Rear Motors
            powerRightRear = -slideSlowPower;
            powerLeftRear = slideSlowPower;

        }
        //Slow slide right
        else if (curOpMode.gamepad1.dpad_right) {

            //Front Motors
            powerLeftFront = slideSlowPower;
            powerRightFront = -slideSlowPower;

            //Rear Motors
            powerRightRear = slideSlowPower;
            powerLeftRear = -slideSlowPower;
        }
        else if(curOpMode.gamepad1.right_bumper)
        {
            powerRightRear = -slowTurn;
            powerLeftRear = slowTurn;
            powerLeftFront = slowTurn;
            powerRightFront = -slowTurn;
        }
        else if(curOpMode.gamepad1.left_bumper)
        {
            powerRightRear = slowTurn;
            powerLeftRear = -slowTurn;
            powerLeftFront = -slowTurn;
            powerRightFront = slowTurn;
        }

        robot.driveRR.setPower(powerRightRear);
        robot.driveLR.setPower(powerLeftRear);
        robot.driveLF.setPower(powerLeftFront);
        robot.driveRF.setPower(powerRightFront);
    }

        printData();
    }

    private void initHardware() {

        robot.init(curOpMode.hardwareMap);
    }

    public void ReturnToZero() {

        pidRotate.setTolerance(.01);
        rotate(-angles.firstAngle,.5);

    }


    public void turnToAngle(double speed, double targetAngle)
    {
        double curAngle = angles.firstAngle;

        double error = curAngle-targetAngle;


        while (curOpMode.opModeIsActive())
        {
            powerRightRear = -speed*Math.signum(error);
            powerLeftRear = speed*Math.signum(error);
            powerLeftFront = speed*Math.signum(error);
            powerRightFront = -speed*Math.signum(error);

            robot.driveRR.setPower(powerRightRear);
            robot.driveLR.setPower(powerLeftRear);
            robot.driveLF.setPower(powerLeftFront);
            robot.driveRF.setPower(powerRightFront);

            error =  angles.firstAngle -targetAngle;

            if(Math.abs(error)<=2)
            {
                break;
            }

            curOpMode.telemetry.addData("error", error);
            curOpMode.telemetry.update();
        }

        robot.driveRR.setPower(0);
        robot.driveLR.setPower(0);
        robot.driveLF.setPower(0);
        robot.driveRF.setPower(0);

    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

        setPointAngle=lastAngles.firstAngle;


    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // If input degrees > 359, we cap at 359 with same sign as input.
        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. We compute the p and I
        // values based on the input degrees and starting power level. We compute the tolerance %
        // to yield a tolerance value of about 1 degree.
        // Overshoot is dependant on the motor and gearing configuration, starting power, weight
        // of the robot and the on target tolerance.

        pidRotate.reset();

        double p = Math.abs((float)power*((float)degrees/(float)180));
        double i = p / 100.0;
        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(-power, power);
        //pidRotate.setTolerance(1.0 / Math.abs(degrees) * 100.0);
        pidRotate.setTolerance((1.0 / Math.abs(degrees)) * 100.0);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (curOpMode.opModeIsActive() && getAngle() == 0)
            {
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);
                //curOpMode.sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.driveLF.setPower(-power);
                robot.driveLR.setPower(-power);

                robot.driveRF.setPower(power);
                robot.driveRR.setPower(power);

                curOpMode.telemetry.update();

            } while (curOpMode.opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.driveLF.setPower(0);
        robot.driveLR.setPower(0);

        robot.driveRF.setPower(0);
        robot.driveRR.setPower(0);




        // wait for rotation to stop.
        curOpMode.sleep(500);
    rotation = getAngle();        // reset angle tracking on new heading.
        resetAngle();
    }



    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        curOpMode.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
        });

        curOpMode.telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }

    /* HT 15358 some of this code belongs higher up, but these are all endgame related */

        public void endgamespin () /*code by HT 15358 */
        {


            while (curOpMode.gamepad1.a) { /* HT 15358 This is the endgamespin actual code, only active
                     while the gamepad button is depressed */
                    double bumpVal; /* HT 15358 first drive into the build platform to be sure squared up */
                double revbumpAdj;  /* HT 15358 adj for backing away from platform */
                bumpVal = 0.2;
                powerRightRear = bumpVal;
                powerLeftRear = bumpVal;
                powerLeftFront = bumpVal;
                powerRightFront = bumpVal;

                robot.driveRR.setPower(powerRightRear);
                robot.driveLR.setPower(powerLeftRear);
                robot.driveLF.setPower(powerLeftFront);
                robot.driveRF.setPower(powerRightFront);


                /* HT 15358 Fifth back away from the platform */
                revbumpAdj = 0.4;
                powerRightRear  = -revbumpAdj;
                powerLeftRear   = -revbumpAdj;
                powerLeftFront  = -revbumpAdj;
                powerRightFront = -revbumpAdj;

                robot.driveRR.setPower(powerRightRear);
                robot.driveLR.setPower(powerLeftRear);
                robot.driveLF.setPower(powerLeftFront);
                robot.driveRF.setPower(powerRightFront);

                /* leftover HT 15358 that we might copy later
                            lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                            globalAngle = 0;

                            setPointAngle=lastAngles.firstAngle;
                */

            }

        } /*end of endgamespin */



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void printData(){
        curOpMode.telemetry.addData("Left Front: ", robot.driveLF.getCurrentPosition());
        curOpMode.telemetry.addData("Left Rear: ", robot.driveLR.getCurrentPosition());
        curOpMode.telemetry.addData("Right Front: ", robot.driveRF.getCurrentPosition());
        curOpMode.telemetry.addData("Right Rear: ", robot.driveRR.getCurrentPosition());
    }

}
