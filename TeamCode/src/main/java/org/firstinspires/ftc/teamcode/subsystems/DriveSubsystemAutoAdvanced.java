package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class DriveSubsystemAutoAdvanced extends SubsystemBase {

    private final MecanumDrive mecDrive;

    private final MecanumDriveKinematics mecKine;
    private final MecanumDriveOdometry mecOdometry;

    private final MotorEx frontLeft, backLeft, frontRight, backRight;
    private final MotorEx[] motors;
    // private final Encoder flEnc, blEnc, brEnc, frEnc;
    // private final Encoder leftEnc, rightEnc, centEnc; // for odometry
    private double WHEEL_DIAM = 2.0;
    private double CPI = 45;
    private RevIMU imu;

    public DriveSubsystemAutoAdvanced(HardwareMap hwMap) {
        frontLeft = new MotorEx(hwMap, "leftFront");
        backLeft = new MotorEx(hwMap, "leftBack");
        backRight = new MotorEx(hwMap, "rightBack");
        frontRight = new MotorEx(hwMap, "rightFront");

        motors = new MotorEx[]{frontLeft, frontRight, backLeft, backRight};

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);


        imu = new RevIMU(hwMap);
        // note: no dead wheels (yet), only drive encoders == tune PID
        // todo get these measurements!!!
        Translation2d fl_Loc = new Translation2d(0.381, 0.381);
        Translation2d fr_Loc = new Translation2d(0.381, -0.381);
        Translation2d bl_Loc = new Translation2d(-0.381, 0.381);
        Translation2d br_Loc = new Translation2d(-0.381, -0.381);

        // Define kine using the wheel locations.
        mecKine = new MecanumDriveKinematics(fl_Loc, fr_Loc, bl_Loc, br_Loc);
        mecOdometry = new MecanumDriveOdometry(mecKine, imu.getRotation2d());

    }

    public enum AutoSpeed {
        SLOW,
        MEDIUM,
        FAST
    }

    public void advBasicGoLR(AutoSpeed speed, long inches) {
        int newTarget = frontLeft.getCurrentPosition() + (int) (inches * CPI);
        reset();

        frontLeft.setTargetPosition(newTarget);
        backLeft.setTargetPosition(-newTarget);
        frontRight.setTargetPosition(-newTarget);
        backLeft.setTargetPosition(newTarget);

        positionMode();

        while (anyBusy()) {

        }

        frontLeft.stopMotor();
        frontRight.stopMotor();
        backLeft.stopMotor();
        backRight.stopMotor();

    }

    public void advBasicGoFB(AutoSpeed speed, long inches, long millis) {
        int newTarget = frontLeft.getCurrentPosition() + (int) (inches * CPI);
        reset();

        frontLeft.setTargetPosition(newTarget);
        backLeft.setTargetPosition(newTarget);
        frontRight.setTargetPosition(newTarget);
        backLeft.setTargetPosition(newTarget);

        positionMode();

        while (anyBusy()) {

        }

        frontLeft.stopMotor();
        frontRight.stopMotor();
        backLeft.stopMotor();
        backRight.stopMotor();
    }

    public void advBasicTurn(AutoSpeed speed, long inches) {

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
