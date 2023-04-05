package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoLiftSubsystem extends SubsystemBase {

    // lift motor
    private final MotorEx leftLift;
    private final MotorEx rightLift;
    // private final MotorGroup dual;
    // speed
    private double speed = 1;
    // positions
    private double lowered;
    private double liftPos;
    private double lastLiftPos;
    // constants
    private final double MAX_SPEED = 4;
    private final double MAX_HEIGHT = 36/383.6 * Math.PI;

    public TwoLiftSubsystem(final HardwareMap hwMap, double s) {

        leftLift = new MotorEx(hwMap, "leftLift", Motor.GoBILDA.RPM_435);
        leftLift.setInverted(true);


        rightLift = new MotorEx(hwMap, "rightLift", Motor.GoBILDA.RPM_435);

        lowered = leftLift.getDistance();

        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftLift.encoder.reset();
        rightLift.encoder.reset();

        speed = s;


        /*
            // dual = new MotorGroup(leftLift, rightLift);

            // dual.setDistancePerPulse(90 / 384.5);
            // dual.setRunMode(Motor.RunMode.PositionControl);
            // dual.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            leftLift.setRunMode(Motor.RunMode.PositionControl);
            rightLift.setRunMode(Motor.RunMode.PositionControl);


            // position things
            lowered = leftLift.getCurrentPosition();
            liftPos = leftLift.getCurrentPosition();
            lastLiftPos = liftPos;

            // 9.0 cm / rev = 90 mm / rev
            // 90 mm / rev * (384.5 pulses / rev)^-1 = 90 / 384.5

            leftLift.setDistancePerPulse(90 / 384.5);
            rightLift.setDistancePerPulse(90 / 384.5);


        */

    }
    @Override
    public void periodic() {
        liftPos = leftLift.getDistance();
    }

    public void up() {
        // completely made up value plz help
        leftLift.set(-2.5 * speed);
        rightLift.set(-2.5 * speed);
        // dual.set(-4 * speed);
        /*
        leftLift.set(-4 * speed);
        rightLift.set(-4 * speed);
         */
    }

    public void down() {
        // completely made up value plz help
        leftLift.set(2.5 * speed);
        rightLift.set(2.5 * speed);
        /*
        leftLift.set(0.6 * speed);
        rightLift.set(0.6 * speed);
         */
    }

    public void incrSpeed() {
        // completely made up value plz help
        speed = Math.max(0,Math.min(speed+0.2,3));
    }

    public void decrSpeed() {
        // completely made up value plz help
        speed=Math.max(0,Math.min(speed-0.2,3));
    }

    public void setSpeed(double s) {
        speed *= s;
    }

    public void moveToPos(double s, double d) {
        // 7.5 -> 12.2 per rot == 4.7 in / rev
        int target = (int)(Math.PI * d / 4.7); // number of inches
    }
    /*
    public void moveToPos2(double d) {
        while ()
    }
     leftLift.setDistancePerPulse(0.2340702210663199);
        rightLift.setDistancePerPulse(0.2340702210663199);
    */
    public double getPos() {
        return leftLift.getDistance();

    }
    public double getRevs() {
        return leftLift.encoder.getRevolutions();
    }

    public void stop() {
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftLift.stopMotor();
        rightLift.stopMotor();
    }

    /**
     * Stop conditions for the lift:
     * - Height is approx. 38 inches or greater
     * - Height is less than or equal to the default fully lowered
     * - Height is less than or equal to 0
     * - Velocity is equal to 0 (lift is stopped)
     * - Acceleration is equal to 0 (lift is stopped)
     * @return
     */

    /*public boolean stopConditions() {
        // explaining `lift.encoder.getRevolutions() * 9 / 2.54 >= 38`
        // 9 cm per revolution / 2.54 == 38 in
        return dual.encoder.getRevolutions() * 9 / 2.54 >= 38 && dual.encoder.getAcceleration() <= 0;
    }

     */
}
