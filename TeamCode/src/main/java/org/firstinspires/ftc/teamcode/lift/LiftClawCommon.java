package org.firstinspires.ftc.teamcode.lift;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.HashMap;
import java.util.Map;
/**
 *      LiftClawCommon  This class contains methods for operating the
 *
 *   Assumptions:
 *   .  The lowest point of the lift just clears the floor when the lift is at its lowest
 *      point and is just low enough to go under the skybridge.
 *   .  When open, the claw is about 5.5" above the floor and clears a stone resting on
 *      the floor.
 *   .  The stone must be raised about 3" above the floor to clear the foundation when
 *      placing the first stone.
 *   .  The first stone must be lowered about 0.75" when placing it on the foundation.
 *   .  After placing the stone, when claw is opened it clears the top tabs.
 *   .  The lift needs to be lowered to its lowest level to go under the skybridge.
 *   .  All of the above apply to the first stone.  For the subsequent stones, the height
 *      when placing the stone needs to be adjusted by 4" per stone already in place.
 *   Trajectory to a stone.
 *   .  Claw does not need to be opened until in proximity with stone.  If the robot
 *      starts on the building side of the skybridge, the lift MUST NOT be raised.
 *   .  At the appropriate time, the claw must be opened to clear the stone.
 *   Picking up a stone (assuming the claw has been positioned over the stone).
 *   .  With the lift at its lowest point, the claw only needs to be opened.
 *   .  To pick up the stone, the claw must be closed.
 *   .  The claw should be raised 0.25" to a carrying level which will allow moving under
 *      the skybridge.
 *   Trajectory to the foundation.
 *   .  On approching the foundation the claw should be raised 3.0" to clear the height
 *      of the foundation plus the height of the tabs. It then must be raised in 4"
 *      increments to clear previously deposited stones in the tower being built.
 *   Dropping the stone.
 *   .  The claw should be lowered 0.75" to place the stone on the foundation or stones
 *      in a tower.
 *   .  The claw should release the stone.
 *   .  The claw should be raised 3" to clear the tabs on top of the stone.  The robot
 *      should now be moved away from the foundation.
 *   Trajectory after releasing a stone.
 *   .  The claw should be lowered to its lowest point for moving under the skybridge
 *      to the next stone.
 *   Servo to English conversion:
 *   .  180 counts on the servo = 1".  All computation uses int values, with the desired
 *      value rounded to the nearest integral count value.
 *
 *
 *    Game controller:
 *    .  left bumper close claw
 *    .  right bumper open claw
 *    .  left trigger pick up stone
 *    .  right trigger deposit stone
 *    .  left joystick up raise lift
 *    .  left joystick down lower lift
 *    .  down arrow engage foundation grabber
 *    .  up arrow disengage foundation grabber
 *    .  gamepad x lower lift one stone height
 *    .  gamepad b lower lift to transit to new stone (base level)
 *    .  gamepad y raise lift to deposit stone
 *       .  press once to raise lift to foundation level
 *       .  press again as many times as needed to clear blocks on an existing tower
 *
 *
 */

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autons.AutoCommon;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;
//import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;

public class LiftClawCommon {

    public LiftClawHardware robot = new LiftClawHardware();

    private int lift_position;
    private static final double DEFAULT_LIFT_SPEED = 0.5;

    private static final double FAST_LIFT_SPEED = .8;

    private LinearOpMode curOpMode=null;

    public int pos = 4;
    int lengthOfPos;

    int rightPos =0;
    int leftPos=0;
    double maxCurrent=0;
    private ElapsedTime     runtime = new ElapsedTime();

    public int autoLiftLoopCount = 0;

    Map<Integer,Integer> LIFT_POSITIONS = new HashMap<>();
    Map<Integer,Integer> STACK_POSITIONS = new HashMap<>();

    public DrivetrainCommon_ALT1 chassis;

    public LiftClawCommon(LinearOpMode owningOpMode){

        curOpMode=owningOpMode;
        initLiftClawHardware();

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_position = 0;

        lengthOfPos = 4;

        LIFT_POSITIONS.put(0,-30);
        LIFT_POSITIONS.put(1, 300);
        LIFT_POSITIONS.put(2, 1400);
        LIFT_POSITIONS.put(3, 2400);
        LIFT_POSITIONS.put(4, 3350);

        STACK_POSITIONS.put(0,0);
        STACK_POSITIONS.put(1, 105);
        STACK_POSITIONS.put(2, 270);
        STACK_POSITIONS.put(3, 380);
        STACK_POSITIONS.put(4, 540);
    }

    public void executeTeleop(){

        if(curOpMode.gamepad2.left_bumper )//&& claw_servoValue>0)
        {
            openClaw();
        }
        else if(curOpMode.gamepad2.right_bumper )//&& claw_servoValue!=.6)
        {
            closeClaw();
        }

        if(curOpMode.gamepad2.dpad_up)
        {
            encoderDrive(1, LIFT_POSITIONS.get(3), 2);
        }

        if(curOpMode.gamepad2.dpad_left)
        {
            encoderDrive(1, LIFT_POSITIONS.get(2),2 );
        }

        if(curOpMode.gamepad2.dpad_down)
        {
            encoderDrive(1, LIFT_POSITIONS.get(1), 3);

        }

        if(curOpMode.gamepad2.dpad_right)
        {
            encoderDrive(1, LIFT_POSITIONS.get(4), 3);

        }

        curOpMode.telemetry.addData("yVal",curOpMode.gamepad2.left_stick_y);
        curOpMode.telemetry.addData("curLiftVal",robot.lift.getCurrentPosition());
        curOpMode.telemetry.addData("current:",maxCurrent);
        if(curOpMode.gamepad2.left_stick_y>0) //Stick values flipped???
        {
           // if (robot.lift.getCurrentPosition() >70 ){  // test for move down request

                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.lift.setPower(-.3);

                curOpMode.telemetry.addData("Current:",robot.lift.getCurrent(CurrentUnit.AMPS));
                //robot.lift.setTargetPosition(robot.lift.getCurrentPosition());

                // Turn On RUN_TO_POSITION
                //robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           // }

        }
        else if(curOpMode.gamepad2.left_stick_y<0 && robot.lift.getCurrentPosition()<3800)
        {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(.5);

            curOpMode.telemetry.addData("Current:",robot.lift.getCurrent(CurrentUnit.AMPS));

        }
        else
        {
            robot.lift.setPower(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(curOpMode.gamepad2.y){
            //clearConeStack(true);
            pos++;
        }

        if(curOpMode.gamepad2.x){

          //nextConeInStack(true, pos);

            if(pos == 0){
                pos = pos;
            } else {
                pos -= 1;
            }

        }

        if(curOpMode.gamepad2.b)
        {
            pos=4;
        }

        if(curOpMode.gamepad2.a)
        {
            openClaw();
            if(robot.lift.getCurrentPosition()<LIFT_POSITIONS.get(0))
            {
                LIFT_POSITIONS.put(0,robot.lift.getCurrentPosition());
            }

            encoderDrive(1, LIFT_POSITIONS.get(0), 4);
        }

        if(curOpMode.gamepad2.left_stick_button)
        {
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        curOpMode.telemetry.addLine().addData("encoder1:", robot.lift.getCurrentPosition());
    }

    public void clearConeStack(boolean checkControls)
    {
        encoderDrive(.4, STACK_POSITIONS.get(pos)+700, 2,checkControls);
    }

    public void nextConeInStack(boolean checkControls, int curPos)
    {
        openClaw();
        encoderDrive(.4, STACK_POSITIONS.get(curPos), 2,checkControls);

    }

    public void goToPos(double speed, int pos, int timeout){
        encoderDrive(speed, LIFT_POSITIONS.get(pos), timeout);
    }

    private void initLiftClawHardware(){

        robot.init(curOpMode.hardwareMap);
    }



    /**
     *
     *    close Claw
     *
     *    Activate the claw servo so that a stone may be grasped.
     *
     */
    public void closeClaw(){
        //robot.claw.setPosition(0.16);//Value for Standard Metal Parts
        robot.claw.setPosition(0.30);//Value for 3D Printed parts
        curOpMode.sleep(500);
    }

    /**
     *
     *    open Claw
     *
     *    Deactivate the claw servo so that a stone may be released.
     *
     */
    public void openClaw(){
        robot.claw.setPosition(0);
        curOpMode.sleep(50);
    }

    /**
     *    pickUpStone()
     *
     *    Compound action:
     *      1.  Move the lift to the bottom location (assumes the stone is on the
     *          floor).
     *      2.  Engage the claw to grasp the stone.
     *      3.  Move the lift up slightly so that the stone will clear the floor.
     */
    public void pickUpStone()
    {
        //  move the lift to 0 to position the claw, engage the claw, move lift to 0.5"
        lift_position = 0;  //  move from wherever to 0
        encoderDrive(DEFAULT_LIFT_SPEED,lift_position,15);
        closeClaw();
        curOpMode.sleep(200);

        lift_position = 200;  //  0.5" clearance under the stone
        encoderDrive(1,lift_position,15);
    }



    /**
     *    returnToBottom()
     *
     *    Move the lift to its lowest position.
     *
     */
    public void returnToBottom(){
        lift_position = 0;
        encoderDrive(FAST_LIFT_SPEED,lift_position,15);
    }


    public void checkDriverControls()
    {
        if(chassis !=null)
        {
            chassis.executeTeleop();
        }
    }

    /**
     *   encoderDrive()
     *
     *   Drive the lift to the level requested by encoderValue.
     *
     * @param speed
     * @param encoderValue
     * @param timeoutS
     */
    public void encoderDrive(double speed,
                             int encoderValue,
                             double timeoutS) {

        //(robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            robot.lift.setTargetPosition(encoderValue);
            int currentPosition = robot.lift.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.lift.setPower(speed);

            curOpMode.telemetry.clear();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            if (encoderValue > currentPosition) {  // if going up, no need to check
                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        robot.lift.isBusy()) {

                    // Display it for the driver.
                    checkDriverControls();
                    curOpMode.telemetry.addData("Path2", "Running at %7d",
                            robot.lift.getCurrentPosition()
                    );
                    curOpMode.telemetry.addData("Current:",robot.lift.getCurrent(CurrentUnit.AMPS));
                    curOpMode.telemetry.update();
                }
            }
            else
            {  // moving down, don't pass zero!

                boolean currentExceeded = false;

                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        robot.lift.isBusy()
                        && !currentExceeded) {

                    // Display it for the driver.
                    checkDriverControls();
                    curOpMode.telemetry.addData("Path2", "Running at %7d",
                            robot.lift.getCurrentPosition()
                    );
                    curOpMode.telemetry.addData("Current:",robot.lift.getCurrent(CurrentUnit.AMPS));
                    curOpMode.telemetry.update();

                    if(robot.lift.getCurrent(CurrentUnit.AMPS)>maxCurrent)
                    {
                        maxCurrent=robot.lift.getCurrent(CurrentUnit.AMPS);

                    }

                    if(robot.lift.getCurrent(CurrentUnit.AMPS)>3)
                    {
                        currentExceeded=true;
                    }
                }
                robot.lift.setPower(0);
                if(currentExceeded && robot.lift.getCurrentPosition()>50)
                {
                    encoderDrive(1,robot.lift.getCurrentPosition()+300,3);
                }

            }
            // Stop all motion;
            robot.lift.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void encoderDrive(double speed,
                             int encoderValue,
                             double timeoutS, boolean checkControls) {

        //(robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            robot.lift.setTargetPosition(encoderValue);
            int currentPosition = robot.lift.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.lift.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            if (encoderValue > currentPosition) {  // if going up, no need to check
                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        robot.lift.isBusy()) {

                    if(checkControls) {
                        // Display it for the driver.
                        checkDriverControls();
                    }
                    curOpMode.telemetry.addData("Path2", "Running at %7d",
                            robot.lift.getCurrentPosition()
                    );
                    curOpMode.telemetry.update();
                }
            }
            else
            {  // moving down, don't pass zero!

                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        robot.lift.isBusy()) {

                    // Display it for the driver.
                    checkDriverControls();
                    curOpMode.telemetry.addData("Path2", "Running at %7d",
                            robot.lift.getCurrentPosition()
                    );
                    curOpMode.telemetry.update();
                }


            }
            // Stop all motion;
            robot.lift.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
    /**
     *   encoderDrive()
     *
     *   Drive the lift to the level requested by encoderValue.
     *
     * @param speed
     * @param encoderValue
     * @param timeoutS
     */
    public void encoderDriveHoldPosition(double speed,
                             int encoderValue,
                             double timeoutS) {

        //(robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            robot.lift.setTargetPosition(encoderValue);
            int currentPosition = robot.lift.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.lift.setPower(speed);

            if (encoderValue > currentPosition) {  // if going up, no need to check
                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        robot.lift.isBusy()) {

                    // Display it for the driver.
                    robot.lift.setPower(speed);
                    curOpMode.telemetry.addData("Path2", "Running at %7d",
                            robot.lift.getCurrentPosition()
                    );
                    curOpMode.telemetry.update();

                    checkDriverControls();
                }
            }

            if(lift_position==0) {
                robot.lift.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



    }

}
