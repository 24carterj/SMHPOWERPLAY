/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.old;

import android.view.View;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot with mecanum wheels, color sensor on servo, and using vuMark.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_front" and "left_rear"
 * Motor channel:  Right drive motor:        "right_front" and "right_rear"
 * Motor channel:  Manipulator drive motor:  "elevator"
 * Servo channel:  Servo to roll:  "hand"

 */
public class Hardware_StarParty
{
    /* Public OpMode members. */
    // declare DcMotors
    public static DcMotor  leftFront   = null;
    public static DcMotor  rightFront  = null;
    public static DcMotor  leftBack = null;
    public static DcMotor  rightBack = null;
    public DcMotor lift = null;


    // constants relating to robot measurements, motor hardware, etc.
    static final double COUNTS_PER_MOTOR_REV = 312;    // Gobilda 5202 19.2:1 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = 45;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    Telemetry telemetry;
    private ElapsedTime period = new ElapsedTime();

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    /* Constructor */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry = telemetry;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack  = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");

        lift = hwMap.get(DcMotor.class, "lift");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        /*leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public static void noEncoder(){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void runEncoder(){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void rest() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public static void resetEncoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void toPosition(){
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void reset(){
        resetEncoder();
        runEncoder();
        //toPosition();
    }

    public boolean allBusy(){
        return (leftFront.isBusy()&&leftBack.isBusy()&&rightFront.isBusy()&&rightBack.isBusy());
    }

    public boolean anyBusy(){
        return (leftFront.isBusy()||leftBack.isBusy()||rightFront.isBusy()||rightBack.isBusy());
    }

    public void goRight(double howfast){
        leftFront.setPower(howfast);
        rightFront.setPower(-howfast);
        leftBack.setPower(-howfast);
        rightBack.setPower(howfast);
    }

    public void goLeft(double howfast){
        leftFront.setPower(-howfast);
        rightFront.setPower(howfast);
        leftBack.setPower(howfast);
        rightBack.setPower(-howfast);
    }

    public void forwardInches(double speed, double inches, double timeoutS) {
        int newTarget;
        reset();
        // Determine new target position, and pass to motor controller
        newTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTarget);
        rightFront.setTargetPosition(newTarget);
        leftBack.setTargetPosition(newTarget);
        rightBack.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        toPosition();

        // reset the timeout time and start motion.
        period.reset();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((period.seconds() < timeoutS) && anyBusy()) {
            // Display it for the driver.
            telemetry.update();
        }

        // Stop all motion;
        rest();

        // Turn off RUN_TO_POSITION
        runEncoder();
    }

    public void rightInches(double speed, double inches, double timeoutS) {
        int newTarget;
        reset();

        // Determine new target position, and pass to motor controller
        newTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH*1.2);

        leftFront.setTargetPosition(newTarget);
        rightFront.setTargetPosition(-1 * newTarget);
        leftBack.setTargetPosition(-1 * newTarget);
        rightBack.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        toPosition();

        // reset the timeout time and start motion.
        period.reset();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((period.seconds() < timeoutS) && anyBusy()) {
            // Display it for the driver.
            telemetry.update();
        }

        // Stop all motion;
        rest();

        // Turn off RUN_TO_POSITION
        runEncoder();
    }
    public void liftUp(double speed, double height, double timeoutS){
        int newTarget;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        int current = lift.getCurrentPosition();
        newTarget = current + (int) (height * 330); // went up 70mm for 10 rotations
        lift.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        period.reset();
        lift.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((period.seconds() < timeoutS) && lift.isBusy()) {
            // Display it for the driver
            telemetry.update();
        }

        // Stop lift;
        lift.setPower(0);

        // Turn off RUN_TO_POSITION
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnDegrees(double speed, int degrees, double timeoutS) {
        int newTarget;
        reset();

        // Determine new target position, and pass to motor controller
        newTarget = leftFront.getCurrentPosition() + (int) (degrees/5.2 * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTarget);
        rightFront.setTargetPosition(-1*newTarget);
        leftBack.setTargetPosition(newTarget);
        rightBack.setTargetPosition(-1*newTarget);

        // Turn On RUN_TO_POSITION
        toPosition();

        // reset the timeout time and start motion.
        period.reset();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((period.seconds() < timeoutS) && anyBusy()) {
            // Display it for the driver.

            telemetry.update();
        }

        // Stop all motion;
        rest();

        // Turn off RUN_TO_POSITION
        runEncoder();
    }
}
