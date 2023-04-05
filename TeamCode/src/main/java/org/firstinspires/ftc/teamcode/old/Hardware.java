package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Hardware {

    private final DcMotor fL, bL, fR, bR;
    private final MotorEx lift;
    private final ServoEx claw;
    // initialize clawOpen in
    private boolean clawOpen = false;

    HardwareMap hwMap;
    Telemetry telemetry;
    public Hardware(HardwareMap hardwareMap){
        hwMap = hardwareMap;
        fL = hwMap.get(DcMotor.class, "frontLeft");
        bL = hwMap.get(DcMotor.class, "backLeft");
        fR = hwMap.get(DcMotor.class, "frontRight");
        bR = hwMap.get(DcMotor.class, "backRight");

        // lift motor
        lift = new MotorEx(
                hwMap,"lift"
        );

        // claw servo
        claw = new SimpleServo(
                hwMap,"claw",0.0,1.0
        );

        clawOpen = !(claw.getPosition() < 0.55);
    }

    /* Methods for TeleOp
     */

    public void drive(Gamepad gamepad) {
        // Variables for gamepad positions
        double lsx = gamepad.left_stick_x;
        double lsy = gamepad.left_stick_y;
        double rs = gamepad.right_stick_x;
        // Calculate variables used for wheel powers
        double ang = Math.atan( lsy / lsx );
        double mag = Math.sqrt( Math.pow(lsx, 2) + Math.pow(lsy, 2));
        double turn = rs;
        // Calculate wheel powers
        double frblPow = Math.sin(ang-Math.PI/4) * mag + turn;
        double flbrPow = Math.sin(ang+Math.PI/4) * mag + turn;
        // Set wheel powers
        this.fR.setPower(frblPow);
        this.fL.setPower(flbrPow);
        this.bR.setPower(flbrPow);
        this.bL.setPower(flbrPow);
    }

    public void lift(Gamepad gamepad) {
        double liftPow = gamepad.right_trigger - gamepad.left_trigger;
        this.lift.setVelocity(liftPow);
    }

    /* Methods for Autonomous
    */

    public void goForward()
    {

    }
    // wait until odometry setup for this part
}
