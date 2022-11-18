package org.firstinspires.ftc.teamcode.lift;
import static org.firstinspires.ftc.teamcode.Robot.DrivetrainLoopState;
import static org.firstinspires.ftc.teamcode.Robot.curOpMode;
import static org.firstinspires.ftc.teamcode.Robot.operator;
import static org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1.executeDrivetrainTeleop;
import static org.firstinspires.ftc.teamcode.lift.LiftClawHardware.claw;
import static org.firstinspires.ftc.teamcode.lift.LiftClawHardware.lift;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.Robot;
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

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainHardware;
import org.firstinspires.ftc.teamcode.sensors.SensorsCommon;
import org.firstinspires.ftc.teamcode.sensors.SensorsHardware;
//import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainCommon_ALT1;

public final class LiftClawCommon {


    private static int lift_position;
    private static final double DEFAULT_LIFT_SPEED = 0.5;

    private static final double FAST_LIFT_SPEED = .8;

    public static int pos = 5;
    static int lengthOfPos;

    static int buttonPressCount=0;

    int rightPos =0;
    int leftPos=0;
    static double maxCurrent=0;
    private static ElapsedTime     runtime = new ElapsedTime();

    static ElapsedTime yButtonElapsed = new ElapsedTime();
    static ElapsedTime xButtonElapsed = new ElapsedTime();
    static Map<Integer,Integer> LIFT_POSITIONS = new HashMap<>();
    static Map<Integer,Integer> STACK_POSITIONS = new HashMap<>();

    public DrivetrainCommon_ALT1 chassis;

    public static void initLiftClawCommon(){

        LiftClawHardware.initLiftClawHardware();

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_position = 0;

        lengthOfPos = 4;

//        40:1 Gear Ratio Encoder Values
         LIFT_POSITIONS.put(0,-30);
        LIFT_POSITIONS.put(1, 350);
        LIFT_POSITIONS.put(2, 1400);
        LIFT_POSITIONS.put(3, 2400);
        LIFT_POSITIONS.put(4, 3400);

        STACK_POSITIONS.put(0,0);
        STACK_POSITIONS.put(1, 105);
        STACK_POSITIONS.put(2, 270);
        STACK_POSITIONS.put(3, 380);
        STACK_POSITIONS.put(4, 540);

        //20:1 Gear Ratio Encoder Values
//        LIFT_POSITIONS.put(0,0);
//        LIFT_POSITIONS.put(1, 175);
//        LIFT_POSITIONS.put(2, 710);
//        LIFT_POSITIONS.put(3, 1210);
//        LIFT_POSITIONS.put(4, 1710);
//
//        STACK_POSITIONS.put(0,0);
//        STACK_POSITIONS.put(1, 74);
//        STACK_POSITIONS.put(2, 139);
//        STACK_POSITIONS.put(3, 208);
//        STACK_POSITIONS.put(4, 281);
    }

    public static void executeLiftClawTeleop(){

        if(operator.left_bumper )//&& claw_servoValue>0)
        {
            openClaw();
        }
        else if(operator.right_bumper )//&& claw_servoValue!=.6)
        {
            closeClaw();
        }

        if(operator.dpad_up)
        {
            encoderDrive(1, LIFT_POSITIONS.get(3), 2);

        }

        if(operator.dpad_left)
        {
            encoderDrive(1, LIFT_POSITIONS.get(2),2 );
        }

        if(operator.dpad_down)
        {
            encoderDrive(1, LIFT_POSITIONS.get(1), 3);

        }

        if(operator.dpad_right)
        {
            encoderDrive(1, LIFT_POSITIONS.get(4), 3);

        }

        if(operator.left_stick_y>0) //Stick values flipped???
        {
           // if (robot.lift.getCurrentPosition() >70 ){  // test for move down request

                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                lift.setPower(-.3);

        }
        else if(operator.left_stick_y<0 && lift.getCurrentPosition()<3800)
        {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setPower(1);
        }
        else
        {
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(operator.y && yButtonElapsed.milliseconds()>250){

           increaseConeStackCount();
           yButtonElapsed.reset();
        }

        if(operator.x && xButtonElapsed.milliseconds()>250){

            reduceConeStackCount();
            xButtonElapsed.reset();
        }

        if(operator.b)
        {

            pos=5;

            SensorsHardware.redLedC1.setState(false);
            SensorsHardware.redLedC2.setState(false);
            SensorsHardware.redLedC3.setState(false);
            SensorsHardware.redLedC4.setState(false);
            SensorsHardware.redLedC5.setState(false);

            SensorsHardware.greenLedC1.setState(true);
            SensorsHardware.greenLedC2.setState(true);
            SensorsHardware.greenLedC3.setState(true);
            SensorsHardware.greenLedC4.setState(true);
            SensorsHardware.greenLedC5.setState(true);

        }

        if(operator.a)
        {
            openClaw();

            encoderDrive(1, LIFT_POSITIONS.get(0), 4);
            //if(lift.getCurrentPosition()<LIFT_POSITIONS.get(0))
            {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


        }

        if(operator.left_stick_button)
        {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        curOpMode.telemetry.addData("lift:",lift.getCurrentPosition());
    }

    public static void clearConeStack(boolean checkControls)
    {
        encoderDrive(.4, STACK_POSITIONS.get(pos)+700, 2,checkControls);
    }

    public static void nextConeInStack(boolean checkControls)
    {
        openClaw();
        encoderDrive(.4, STACK_POSITIONS.get(pos), 3,checkControls);

    }

    public static void goToPos(double speed, int pos, int timeout){
        encoderDrive(speed, LIFT_POSITIONS.get(pos), timeout);
    }

    public static void reduceConeStackCount()
    {
        if(pos > 0){
            pos--;
        }

        switch(pos)
        {
            case 4:
            {
                SensorsHardware.greenLedC5.setState(false);
                SensorsHardware.redLedC5.setState(true);
                break;
            }
            case 3:
            {
                SensorsHardware.greenLedC4.setState(false);
                SensorsHardware.redLedC4.setState(true);
                break;
            }

            case 2:
            {
                SensorsHardware.greenLedC3.setState(false);
                SensorsHardware.redLedC3.setState(true);
                break;
            }

            case 1:
            {
                SensorsHardware.greenLedC2.setState(false);
                SensorsHardware.redLedC2.setState(true);
                break;
            }
            case 0:
            {
                SensorsHardware.greenLedC1.setState(false);
                SensorsHardware.redLedC1.setState(true);
                break;
            }
        }

    }
    public static void increaseConeStackCount()
    {
        if(pos<5) {
            pos++;
        }
        switch(pos)
        {
            case 5:
            {
                SensorsHardware.greenLedC5.setState(true);
                SensorsHardware.redLedC5.setState(false);
                break;
            }
            case 4:
            {
                SensorsHardware.greenLedC4.setState(true);
                SensorsHardware.redLedC4.setState(false);
                break;
            }

            case 3:
            {
                SensorsHardware.greenLedC3.setState(true);
                SensorsHardware.redLedC3.setState(false);
                break;
            }

            case 2:
            {
                SensorsHardware.greenLedC2.setState(true);
                SensorsHardware.redLedC2.setState(false);
                break;
            }
            case 1:
            {
                SensorsHardware.greenLedC1.setState(true);
                SensorsHardware.redLedC1.setState(false);
                break;
            }
        }
    }


    /**
     *
     *    close Claw
     *
     *    Activate the claw servo so that a stone may be grasped.
     *
     */
    public static void closeClaw(){
        //robot.claw.setPosition(0.16);//Value for Standard Metal Parts
        claw.setPosition(0.30);//Value for 3D Printed parts
        curOpMode.sleep(500);
    }

    /**
     *
     *    open Claw
     *
     *    Deactivate the claw servo so that a stone may be released.
     *
     */
    public static void openClaw(){
        claw.setPosition(0);
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


    public static void checkDriverControls()
    {

            executeDrivetrainTeleop();


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
    public static void encoderDrive(double speed,
                                    int encoderValue,
                                    double timeoutS) {

        //(robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            lift.setTargetPosition(encoderValue);
            int currentPosition = lift.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(speed);

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
                        lift.isBusy()) {

                    // Display it for the driver.
                    checkDriverControls();

                }
            }
            else
            {  // moving down, don't pass zero!

                boolean currentExceeded = false;

                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        lift.isBusy()
                        && !currentExceeded) {

                    // Display it for the driver.
                    checkDriverControls();

                    if(lift.getCurrent(CurrentUnit.AMPS)>maxCurrent)
                    {
                        maxCurrent= lift.getCurrent(CurrentUnit.AMPS);

                    }

                    if(maxCurrent>3)
                    {
                        currentExceeded=true;
                    }
                }
                lift.setPower(0);
                if(currentExceeded && lift.getCurrentPosition()>50)
                {
                    //encoderDrive(1, lift.getCurrentPosition()+300,3);
                }

            }
            // Stop all motion;
            lift.setPower(0);

            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Reset Maximum current value
            maxCurrent=0;
        }

    }

    public static void encoderDrive(double speed,
                                    int encoderValue,
                                    double timeoutS, boolean checkControls) {

        //(robot.lift_check.getDistance(DistanceUnit.INCH)>= 0)

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            lift.setTargetPosition(encoderValue);
            int currentPosition = lift.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            if (encoderValue > currentPosition) {  // if going up, no need to check
                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        lift.isBusy()) {

                    if(checkControls) {
                        // Display it for the driver.
                        checkDriverControls();
                    }



                }
            }
            else
            {  // moving down, don't pass zero!

                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        lift.isBusy()) {

                    // Display it for the driver.
                    checkDriverControls();

                }


            }
            // Stop all motion;
            lift.setPower(0);

            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            lift.setTargetPosition(encoderValue);
            int currentPosition = lift.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(speed);

            if (encoderValue > currentPosition) {  // if going up, no need to check
                while (curOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        lift.isBusy()) {

                    // Display it for the driver.
                    lift.setPower(speed);


                    checkDriverControls();
                }
            }

            if(lift_position==0) {
                lift.setPower(0);

                // Turn off RUN_TO_POSITION
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



    }

}
