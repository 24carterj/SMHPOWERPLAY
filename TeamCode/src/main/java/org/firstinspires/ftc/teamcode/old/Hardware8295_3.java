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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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

public class Hardware8295_3
{
    /* Public OpMode members. */
    // declare DcMotors
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack = null;
    public DcMotor  rightBack = null;
    public DcMotor lift = null;
    //public DcMotor arm = null;
    // declare Servos
    public CRServo spinnerL = null;
    public CRServo spinnerR = null;
    public CRServo roller = null;
    public Servo arm = null;
    public CRServo tape = null;
    // declare Sensors ??/
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;

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
    public Hardware8295_3(){

    }

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
        arm = hwMap.get(Servo.class, "arm");
        spinnerL = hwMap.get(CRServo.class, "spinnerL");
        spinnerR = hwMap.get(CRServo.class, "spinnerR");
        roller = hwMap.get(CRServo.class, "roller");
        tape = hwMap.get(CRServo.class, "tape");
        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

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
        lift.setPower(0);
    }

    public void noEncoder(){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runEncoder(){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rest() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void resetEncoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void toPosition(){
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void reset(){
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
            telemetry.addData("Path1", "Running to %7d", newTarget);
            telemetry.addData("Path2", "Running at %7d", leftFront.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        rest();

        // Turn off RUN_TO_POSITION
        runEncoder();
    }

    public void rollerSpeed(int rollSpeed, double timeoutS) {
        //reset period
        period.reset();
        //roller speed = rollSpeed
        roller.setPower(rollSpeed);
        //run for timeoutS seconds
        while ((period.seconds() < timeoutS)) {
            // Display it for the driver.
            telemetry.addData("Rolling ", period.seconds());
            //telemetry.addData("Roller active", "Running to %7d", period.seconds());
            telemetry.update();
        }
        //roller speed 0
        roller.setPower(0);
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
            telemetry.addData("Path1", "Running to %7d", newTarget);
            telemetry.addData("Path2", "Running at %7d", leftFront.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        rest();

        // Turn off RUN_TO_POSITION
        runEncoder();
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
            telemetry.addData("Path1", "Running to %7d", newTarget);
            telemetry.addData("Path2", "Running at %7d", leftFront.getCurrentPosition());
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
            // Display it for the driver.
            telemetry.addData("starting from: ", current);
            telemetry.addData("Path1", "Running to %7d", newTarget);
            telemetry.addData("Path2", "Running at %7d", lift.getCurrentPosition());
            telemetry.update();
        }

        // Stop lift;
        lift.setPower(0);

        // Turn off RUN_TO_POSITION
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean findObject(){
        boolean result = false;
        final double SCALE_FACTOR = 255;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        int alpha = sensorColor.alpha();
        int hue = (int)hsvValues[0];
        //int sum = sensorColor.alpha()+sensorColor.red()+sensorColor.green()+sensorColor.blue() - (int)hsvValues[0];
        telemetry.addData("Alpha",alpha );
        telemetry.addData("Hue", hue);
        telemetry.update();
        if (alpha > 800 || hue < 115)
        {
            result = true;
        }
        return result;
    }
}
