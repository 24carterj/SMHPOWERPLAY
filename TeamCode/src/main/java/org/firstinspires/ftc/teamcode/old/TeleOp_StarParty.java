
package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "TeleOp_StarParty", group = "Zhou")
public class TeleOp_StarParty extends LinearOpMode {

    Hardware_StarParty robot = new Hardware_StarParty();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        robot.noEncoder();

        String direction = "none";


        double dist = 0.0;
        double dist_incr = 0.001;
        int degrees = 0;

        telemetry.addLine("Direction");
        telemetry.addLine("Distance");
        telemetry.addLine("Distance increment");
        telemetry.addLine("Turn degrees");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Direction",direction);
            telemetry.addData("Distance",dist);
            telemetry.addData("Distance increment",dist_incr);
            telemetry.addData("Turn degrees",degrees);
            // direction :
            if (gamepad1.dpad_down) direction="back";
            if (gamepad1.dpad_up) direction="forward";
            if (gamepad1.dpad_left) direction="left";
            if (gamepad1.dpad_right) direction="right";
            telemetry.update();
            // amount increase distance
            if (gamepad1.right_trigger>0.2) dist_incr+=0.001;
            if (gamepad1.left_trigger>0.2) dist_incr-=0.001;
            if (dist_incr<0.0) dist_incr = 0.0;
            telemetry.update();
            // increase distance interval
            if (gamepad1.y) dist+=dist_incr;
            if (gamepad1.a) dist-=dist_incr;
            if (dist<0) dist=0.0;

            telemetry.update();
            // turning
            if (gamepad1.b) {
                direction="turn";
                degrees+=1;
            }
            if (gamepad1.x) {
                direction = "turn";
                degrees+=1;
            }
            telemetry.update();
            if (degrees<0) degrees=360-Math.abs(degrees);
            if (degrees>360) degrees%=360;

            if (gamepad1.left_bumper){
                sleep(3000);
                if (direction=="turn") robot.turnDegrees(0.7,degrees,7);
                if (direction=="forward") robot.forwardInches(0.7,dist,7);
                if (direction=="back") robot.forwardInches(0.7,-dist,7);
                if (direction=="right") robot.rightInches(0.7,dist,7);
                if (direction=="left") robot.rightInches(0.7,-dist,7);
                direction="none";
                dist=0.0;
                degrees=0;
            }
            if (gamepad1.right_bumper){
                direction="none";
                degrees=0;
                dist=0.0;
                dist_incr=0.001;
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper) stop();
            
        }

    }
}

