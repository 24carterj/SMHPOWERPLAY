package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ClawGrab;
import org.firstinspires.ftc.teamcode.commands.ClawRelease;
import org.firstinspires.ftc.teamcode.commands.DefDrive;
import org.firstinspires.ftc.teamcode.commands.LiftDown;
import org.firstinspires.ftc.teamcode.commands.LiftUp;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp(name="Push NO CLAW NO LIFT")
public class APUSHBOT extends LinearOpMode {

    public static DcMotor leftFront   = null;
    public static DcMotor  rightFront  = null;
    public static DcMotor  leftBack = null;
    public static DcMotor  rightBack = null;
    // public DcMotor lift = null;
    // public CRServo claw = null;


    // constants relating to robot measurements, motor hardware, etc.
    static final double COUNTS_PER_MOTOR_REV = 312;    // Gobilda 5202 19.2:1 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = 45;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    Telemetry telemetry;
    private ElapsedTime period = new ElapsedTime();

    private boolean autoDetect;

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

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

        waitForStart();
        while (opModeIsActive()) {
            /*
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            double r = Math.hypot(leftX, leftY);
            double theta = Math.atan(leftY/leftX);

            // double lfrbMove = r * Math.sin(theta + Math.PI/4) + rightX;
            // double lbrfMove = r * Math.sin(theta - Math.PI/4) + rightX;

            leftFront.setPower(r * Math.cos(theta) + rightX);
            rightBack.setPower(r * Math.cos(theta) - rightX);
            leftBack.setPower(r * Math.sin(theta) + rightX);
            rightFront.setPower(r * Math.sin(theta) - rightX);
            */
            // use gamepad1 left stick to go forward, backward, and turn
            if (gamepad1.left_stick_button) {
                leftFront.setPower(-0.3* (gamepad1.left_stick_y) + 0.3 * (gamepad1.left_stick_x));
                rightFront.setPower(-0.3 * (gamepad1.left_stick_y) - 0.3 * (gamepad1.left_stick_x));
                leftBack.setPower(-0.3 * (gamepad1.left_stick_y) + 0.3 * (gamepad1.left_stick_x));
                rightBack.setPower(-0.3 * (gamepad1.left_stick_y) - 0.3 * (gamepad1.left_stick_x));
            }
            else {
                leftFront.setPower(-0.5 * (gamepad1.left_stick_y) + 0.5 * (gamepad1.left_stick_x));
                rightFront.setPower(-0.5 * (gamepad1.left_stick_y) - 0.5* gamepad1.left_stick_x);
                leftBack.setPower(-0.5 * (gamepad1.left_stick_y) + 0.5 * gamepad1.left_stick_x);
                rightBack.setPower(-0.5 * (gamepad1.left_stick_y) - 0.5 * (gamepad1.left_stick_x));
            }


            //use gamepad1 right stick to move left and right
            if(gamepad1.right_stick_x> 0){
                leftFront.setPower(gamepad1.right_stick_x);
                rightFront.setPower(-gamepad1.right_stick_x);
                leftBack.setPower(-gamepad1.right_stick_x);
                rightBack.setPower(gamepad1.right_stick_x);
            }
            if(gamepad1.right_stick_x< 0){
                leftFront.setPower(-gamepad1.right_stick_x);
                rightFront.setPower(gamepad1.right_stick_x);
                leftBack.setPower(-gamepad1.right_stick_x);
                rightBack.setPower(gamepad1.right_stick_x);
            }


        }
    }
}
