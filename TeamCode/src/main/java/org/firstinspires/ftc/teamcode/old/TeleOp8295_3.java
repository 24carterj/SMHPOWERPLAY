
package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Disabled

@TeleOp(name = "TeleOp8295_3", group = "Zhou")
public class TeleOp8295_3 extends LinearOpMode {

    Hardware8295_3 robot = new Hardware8295_3();
    private ElapsedTime runtime = new ElapsedTime();
    static final double FORWARD_SPEED = 0.6;
    double          armOffset = 0;                // Servo mid position
    final double    ARM_SPEED  = 0.001 ;
    // sets rate to move servo
    //double flipOffset = 0.5;
    
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        robot.noEncoder();

        waitForStart();
        while (opModeIsActive()) {

            // use gamepad1 left stick to go forward, backward, and turn
            if (gamepad1.left_stick_button) {
                robot.leftFront.setPower(-1* (gamepad1.left_stick_y) + 1 * (gamepad1.left_stick_x));
                robot.rightFront.setPower(-1 * (gamepad1.left_stick_y) - 1 * (gamepad1.left_stick_x));
                robot.leftBack.setPower(-1 * (gamepad1.left_stick_y) + 1 * (gamepad1.left_stick_x));
                robot.rightBack.setPower(-1 * (gamepad1.left_stick_y) - 1 * (gamepad1.left_stick_x));
            }
            else {
                robot.leftFront.setPower(-0.7 * (gamepad1.left_stick_y) + 0.7 * (gamepad1.left_stick_x));
                robot.rightFront.setPower(-0.7 * (gamepad1.left_stick_y) - 0.7 * gamepad1.left_stick_x);
                robot.leftBack.setPower(-0.7 * (gamepad1.left_stick_y) + 0.7 * gamepad1.left_stick_x);
                robot.rightBack.setPower(-0.7 * (gamepad1.left_stick_y) - 0.7 * (gamepad1.left_stick_x));
            }

            telemetry.addData("leftFront position",robot.leftFront.getCurrentPosition());
            telemetry.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors",gamepad1.left_stick_y +", " + gamepad1.left_stick_x);
            telemetry.update();

            //use gamepad1 right stick to move left and right
            if(gamepad1.right_stick_x> 0){
                robot.goRight(gamepad1.right_stick_x);
            }
            if(gamepad1.right_stick_x< 0){
                robot.goLeft(-gamepad1.right_stick_x);
            }

            // Use gamepad dpad to spin
            if (gamepad1.dpad_left) {
                robot.spinnerL.setPower(-1);
                robot.spinnerR.setPower(-1);
            }
            else if(gamepad1.dpad_right) {
                robot.spinnerL.setPower(1);
                robot.spinnerR.setPower(1);
            }
            else {
                robot.spinnerL.setPower(0);
                robot.spinnerR.setPower(0);
            }

            // Use gamepad triggers to lift
            if (gamepad1.left_trigger > 0 ) {
                robot.lift.setPower(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0) {
                robot.lift.setPower(-gamepad1.right_trigger);
            }
            else {
                robot.lift.setPower(0);
            }

            // Use bumpers to roll in and out
            if (gamepad1.left_bumper) {
                robot.roller.setPower(-1);
                if(robot.findObject()) {
                    gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
                    //gamepad1.stopRumble();
                }
            }
            else if (gamepad1.right_bumper) {
                robot.roller.setPower(1);
            }
            else {
                robot.roller.setPower(0);
            }

            // Use Y and A to pick up shipping element
            if(gamepad1.y) {
                armOffset += ARM_SPEED;
            }
            else if (gamepad1.a) {
                armOffset -= ARM_SPEED;
            }
            armOffset= Range.clip(armOffset, -0.45, 0.65);
            robot.arm.setPosition(0.05 + armOffset);

            //Use X and B to extend tape
            if(gamepad1.x) {
                robot.tape.setPower(-1);
            }
            else if (gamepad1.b) {
                robot.tape.setPower(1);
            }
            else {
                robot.tape.setPower(0);
            }

            //rumble
        }

    }
}

