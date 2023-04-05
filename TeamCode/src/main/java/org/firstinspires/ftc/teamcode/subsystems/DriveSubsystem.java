package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.util.Direction;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.PWMOutputControllerEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.apache.commons.math3.analysis.function.Max;

import java.util.concurrent.TimeUnit;

public class DriveSubsystem extends SubsystemBase {

    // drivetrain
    private final MecanumDrive mecDrive;
    // motors
    private final MotorEx frontLeft, backLeft, frontRight, backRight;

    // drive speed
    private double speed = 0.75;
    private boolean reverse = false;

    // odometry stuff
    // private final MecanumDriveKinematics mecKine;
    // private final MecanumDriveOdometry mecOdom;

    // encoders
    private final Motor.Encoder fL_enc, bL_enc, fR_enc, bR_enc;
    // gyro
    // private RevIMU imu;

    // pose
    // private Pose2d pose;

    // tuning / constants
    private final double MAX_SPEED = 4;
    private final double WHEEL_DIAM = 2.0;


    public DriveSubsystem(HardwareMap hwMap) {
        frontLeft = new MotorEx(hwMap, "leftFront", Motor.GoBILDA.RPM_435);
        backLeft = new MotorEx(hwMap, "leftBack", Motor.GoBILDA.RPM_435);
        backRight = new MotorEx(hwMap, "rightBack", Motor.GoBILDA.RPM_435);
        frontRight = new MotorEx(hwMap, "rightFront", Motor.GoBILDA.RPM_435);

        fL_enc = frontLeft.encoder;
        bL_enc = backLeft.encoder;
        bR_enc = backRight.encoder;
        fR_enc = frontRight.encoder;

        // mecanum drive
        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // imu for gyroscope

        // imu = new RevIMU(hwMap);
        // imu.init();

        /*
        // wheel locations
        Translation2d fL_loc =
                new Translation2d(0.381, 0.381);
        Translation2d fR_loc =
                new Translation2d(0.381, -0.381);
        Translation2d bL_loc =
                new Translation2d(-0.381, 0.381);
        Translation2d bR_loc =
                new Translation2d(-0.381, -0.381);

        // mecanum kinematics
        mecKine = new MecanumDriveKinematics(fL_loc, fR_loc, bL_loc, bR_loc);

        // mecanum odometry
        mecOdom = new MecanumDriveOdometry(mecKine,
                imu.getRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d())
        );
        */

    }

    /*
    @Override
    public void periodic() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
            fL_enc.getRate(), fR_enc.getRate(), bL_enc.getRate(), bR_enc.getRate()
        );

        Rotation2d rot = imu.getRotation2d();

        pose = mecOdom.updateWithTime(0.5, rot, wheelSpeeds);
    }
    */

    /**
     * Toggle robot direction
     */
    public void switchDirection() {
        reverse = !reverse;
        speed *= -1;
    }
    /**
     *
     * @param x strafeSpeed
     * @param y forwardSpeed
     * @param turn turnSpeed
     * @param head heading
     */

    /*
    public void driveFC(double x, double y, double turn, double head) {
        mecDrive.driveFieldCentric(-x  * speed, -y  * speed, turn  * speed, imu.getHeading());
    }
    */
    /** Drive robotic centric
     *
     * @param x strafeSpeed
     * @param y forwardSpeed
     * @param turn turnSpeed
     */
    public void driveRC(double x, double y, double turn) {
        // int dirCoeff = reverse ? -1 : 1;
        // mecDrive.driveRobotCentric(dirCoeff * -x * speed, dirCoeff * -y  * speed, dirCoeff * -turn  * speed);
        mecDrive.driveRobotCentric(-x * speed, -y  * speed, -turn  * speed);
    }
    /** Driving, but restricted to cardinal directions
     *
     * @param x strafeSpeed
     * @param y forwardSpeed
     * @param turn turnSpeed
     */
    public void driveCardinal(double x, double y, double turn) {
        if (turn > 0) {
            mecDrive.driveRobotCentric(0, 0, turn);
        } else {
            double angle = Math.atan2(y,x);
            double hypot = Math.hypot(x,y);
            if ( angle > 7*Math.PI/4 || angle <= Math.PI/4 ) //todo is this right?!
                mecDrive.driveRobotCentric(hypot, 0, 0);
            else if ( angle > Math.PI/4 && angle <= 3*Math.PI/4 )
                mecDrive.driveRobotCentric(0, hypot, 0);
            else if ( angle > 3*Math.PI/4 && angle <= 5*Math.PI/4 )
                mecDrive.driveRobotCentric(-hypot, 0, 0);
            else if ( angle > 5*Math.PI/4 && angle <= 7*Math.PI/4 )
                mecDrive.driveRobotCentric(0, -hypot, 0);
            else
                mecDrive.driveRobotCentric(0, 0, 0);
        }
    }

    /**
     * Drive dist inches at s speed in dir direction
     *
     * @param dist Distance
     * @param s Speed
     * @param dir Direction
     * @return Finished
     */
    public boolean driveDist(double dist, double s, Direction dir) {

        int target = (int)(frontLeft.getCurrentPosition() + dist * 4.0 * Math.PI);
        positionMode();

        switch (dir) {
            case FORWARD:
                mecDrive.driveWithMotorPowers(-1, -1, -1, -1);
                break;
            case BACKWARDS:
                mecDrive.driveWithMotorPowers(1,1,1,1);
                break;
            case LEFT:
                mecDrive.driveWithMotorPowers(-1,1,1,-1);
                break;
            case RIGHT:
                mecDrive.driveWithMotorPowers(1,-1,-1,1);
                break;
            default: break;
        }

        while (getRevolutions() * 4.0 * Math.PI < dist) {
            frontLeft.getCurrentPosition();
        }

        stop();
        velocityMode();
        return true;
    }

    public void setSpeed(double s) {
        if (0 >= s) speed = 0.1;
        else if (s > MAX_SPEED) speed = MAX_SPEED;
        else speed = Math.abs(s);
    }

    /**
     * Increase the robot speed
     */
    public void incrSpeed() {
        speed += 0.1;
        speed = Math.min(1.0, Math.max(speed, 0.1));
    }
    /**
     * Decrease the robot speed
     */
    public void decrSpeed() {
        speed -= 0.1;
        speed = Math.min(1.0, Math.max(speed, 0.1));
    }
    /**
     * Return the robot speed
     */
    public double getSpeed() {
        return speed;
    }
    /**
     * Return the average encoder revolutions
     */
    public double getRevolutions() {
        return
        (fL_enc.getRevolutions() + fR_enc.getRevolutions() + bL_enc.getRevolutions() + bR_enc.getRevolutions()) / 4.;
    }
    /**
     * Return the left front encoder distance
     */
    public double getLeftEncoderDistance(){
        return frontLeft.getDistance();
    }
    /**
     * Return the left front encoder revolutions
     */
    public double getLeftEncoderRevs(){
        return fL_enc.getRevolutions();
    }

    /**
     * Stops all motors
     */
    public void stop() {
        frontLeft.stopMotor();
        frontRight.stopMotor();
        backLeft.stopMotor();
        backRight.stopMotor();
    }

    /**
     * Resets all encoders
     */
    public void resetEncoders() {
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    /**
     * Switches robot to position mode
     */
    public void positionMode() {
        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);
    }

    /**
     * Switches robot to velocity mode
     */
    public void velocityMode() {
        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);
    }
}
