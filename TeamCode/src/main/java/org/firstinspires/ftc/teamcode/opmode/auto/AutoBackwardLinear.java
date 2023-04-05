package org.firstinspires.ftc.teamcode.opmode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Autonomous(name="Go Backwards Linear")
public class AutoBackwardLinear extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 312;    // Gobilda 5202 19.2:1 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = 45;

    private ElapsedTime period = new ElapsedTime();
    Telemetry telemetry;

    DcMotor lf;
    DcMotor rf;
    DcMotor rb;
    DcMotor lb;

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotor.class, "leftFront");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        rb = hardwareMap.get(DcMotor.class, "rightBack");
        lb = hardwareMap.get(DcMotor.class, "leftBack");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            fauxwardInches(34);
            sleep(500000000);
            stop();
            terminateOpModeNow();
        }
    }

    public void fauxwardInches(double inches) {
        int newTarget;
        // Determine new target position, and pass to motor controller
        newTarget = lf.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        // newTarget = (int) (inches * 4.0 * Math.PI);

        lf.setTargetPosition(-newTarget);
        rf.setTargetPosition(-newTarget);
        lb.setTargetPosition(-newTarget);
        rb.setTargetPosition(-newTarget);

        // Turn On RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        period.reset();

        lf.setPower(0.5);
        rf.setPower(0.5);
        lb.setPower(0.5);
        rb.setPower(0.5);

        while (lf.getCurrentPosition() == newTarget/*(period.seconds() < timeoutS)*/ || lf.isBusy() || rf.isBusy() || lb.isBusy() || rb.isBusy()) {
            lf.getCurrentPosition();
        }

        // Stop all motion;
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        // Turn off RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
