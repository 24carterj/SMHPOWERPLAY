
package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "TeleOp8295_PPTrig", group = "Zhou")
public class TeleOp8295_PP2 extends LinearOpMode {

    Hardware8295_4 robot = new Hardware8295_4();
    private ElapsedTime runtime = new ElapsedTime();
    static final double FORWARD_SPEED = 0.6;
    double          clawOffset = 0;                // Servo mid position
    final double    CLAW_SPEED  = 0.001 ;
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
                double lsx = gamepad1.left_stick_x;
                double lsy = gamepad1.left_stick_y;
                double rs = gamepad1.right_stick_x;
                // Calculate variables used for wheel powers
                double ang = Math.atan( lsy / lsx );
                double mag = Math.sqrt( Math.pow(lsx, 2) + Math.pow(lsy, 2));
                double turn = rs;
                // Calculate wheel powers
                double frblPow = Math.sin(ang-Math.PI/4) * mag + turn;
                double flbrPow = Math.sin(ang+Math.PI/4) * mag + turn;
                // Set wheel powers
                robot.rightFront.setPower(frblPow);
                robot.leftFront.setPower(flbrPow);
                robot.rightBack.setPower(flbrPow);
                robot.leftBack.setPower(flbrPow);
            }
            else {
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightBack.setPower(0);
            }

            //use gamepad1 right stick to move left and right
            if(gamepad1.right_stick_x> 0){
                robot.goRight(gamepad1.right_stick_x);
            }
            if(gamepad1.right_stick_x< 0) {
                robot.goLeft(-gamepad1.right_stick_x);
            }
            /*
            // use right & left trigger to move the lift up & down
            if (gamepad1.left_trigger > 0 ) {
                robot.lift.setPower(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0) {
                robot.lift.setPower(-gamepad1.right_trigger);
            }
            else {
                robot.lift.setPower(0);
            }
            // use y & a bumper to open/close claw
            if (gamepad1.y){
                robot.claw.setPower(0.7);
            }
            else if (gamepad1.a){
                robot.claw.setPower(-0.7);
            }
            else {
                robot.claw.setPower(0);
            }
        */
        }


    }
}

