package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.acmerobotics.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.acmerobotics.drive.DriveConstants.rpmToVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class DriveSubsystemAutoBasic extends SubsystemBase {

    private final MecanumDrive mecDrive;

    private final MotorEx frontLeft, backLeft, frontRight, backRight;
    private final MotorEx[] motors;

    private double WHEEL_DIAM = 2.0;
    private double CPI = 45;

    public DriveSubsystemAutoBasic(HardwareMap hwMap) {
        frontLeft = new MotorEx(hwMap, "leftFront");
        backLeft = new MotorEx(hwMap, "leftBack");
        backRight = new MotorEx(hwMap, "rightBack");
        frontRight = new MotorEx(hwMap, "rightFront");

        motors = new MotorEx[]{frontLeft, frontRight, backLeft, backRight};

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    public enum AutoSpeed {
        SLOW,
        MEDIUM,
        FAST
    }

    public void basicGoLR(AutoSpeed speed, long inches) {
        int newTarget = frontLeft.getCurrentPosition() + (int) (inches * CPI);
        reset();

        frontLeft.setTargetPosition(newTarget);
        backLeft.setTargetPosition(-newTarget);
        frontRight.setTargetPosition(-newTarget);
        backRight.setTargetPosition(newTarget);

        positionMode();
        double d_speed = 0.0;

        switch (speed) {
            case SLOW:
                d_speed = 0.3;
                break;
            case MEDIUM:
                d_speed = 0.5;
                break;
            case FAST:
                d_speed = 0.7;
                break;
            default: break;
        }

        frontLeft.set(d_speed);
        frontRight.set(d_speed);
        backLeft.set(d_speed);
        backRight.set(d_speed);

        frontLeft.stopMotor();
        frontRight.stopMotor();
        backLeft.stopMotor();
        backRight.stopMotor();

    }

    public void basicGoFB(AutoSpeed speed, long inches, long millis) {
        int newTarget = frontLeft.getCurrentPosition() + (int) (inches * CPI);
        reset();

        frontLeft.setTargetPosition(newTarget);
        backLeft.setTargetPosition(newTarget);
        frontRight.setTargetPosition(newTarget);
        backLeft.setTargetPosition(newTarget);

        positionMode();
        double d_speed = 0.0;

        switch (speed) {
            case SLOW:
                d_speed = 0.3;
                break;
            case MEDIUM:
                d_speed = 0.5;
                break;
            case FAST:
                d_speed = 0.7;
                break;
            default: break;
        }

        while (anyBusy()) {

        }

        frontLeft.stopMotor();
        frontRight.stopMotor();
        backLeft.stopMotor();
        backRight.stopMotor();

    }

    public void semiBasicFB(AutoSpeed speed, long inches) {

    }

    public void basicTurn(AutoSpeed speed, long inches) {

    }

    public void positionMode() {
        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);
    }

    public void veloMode() {
        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);
    }

    public void reset() {

    }

    public boolean anyBusy() {
        return  frontLeft.getVelocity()  != 0 ||
                frontRight.getVelocity() != 0 ||
                backLeft.getVelocity()   != 0 ||
                backRight.getVelocity()  != 0;
    }
}
