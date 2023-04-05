
package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "TeleOp8295_Test", group = "Zhou")
public class TeleOpTest extends LinearOpMode {

    Hardware robot = new Hardware(hardwareMap);
    private ElapsedTime runtime = new ElapsedTime();
    static final double FORWARD_SPEED = 0.6;
    double          clawOffset = 0;                // Servo mid position
    final double    CLAW_SPEED  = 0.001 ;
    // sets rate to move servo
    //double flipOffset = 0.5;

    @Override
    public void runOpMode() {
        while (opModeIsActive()) {
            robot.drive(gamepad1);
        }

    }
}

