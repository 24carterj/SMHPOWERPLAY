package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {

    // lift motor
    private final MotorEx lift;
    // speed
    private double speed = 1;
    // positions
    private int lowered;
    private double liftPos;
    private double lastLiftPos;
    // constants
    private final double MAX_SPEED = 4;
    private final double MAX_HEIGHT = 36/383.6 * Math.PI;

    public LiftSubsystem(final HardwareMap hwMap, final String name) {
        lift = new MotorEx(hwMap, "lift", Motor.GoBILDA.RPM_435);
        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.encoder.reset();

        // position things
        lowered = lift.getCurrentPosition();
        liftPos = lift.getCurrentPosition();
        lastLiftPos = liftPos;

        // 9.0 cm / rev = 90 mm / rev
        // 90 mm / rev * (384.5 pulses / rev)^-1 = 90 / 384.5
        lift.setDistancePerPulse(90 / 384.5);
        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void up() {
        // completely made up value plz help
        lift.set(-4 * speed);
    }

    public void down() {
        // completely made up value plz help
        lift.set(2 * speed);
    }

    public void incrSpeed() {
        // completely made up value plz help
        speed+=0.2;
    }

    public void decrSpeed() {
        // completely made up value plz help
        speed-=0.2;
    }

    public void stop() {
        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lift.stopMotor();
    }
    /*
    public void setSpeed(double s) {
        if (0 >= s) speed = 0.1;
        else if (s > MAX_SPEED) speed = MAX_SPEED;
        else speed = Math.abs(s);
    }

    public double getSpeed() {
        return speed;
    }
    */

    /*
    @Override
    public void periodic() {
        lastLiftPos = liftPos;
        liftPos = lift.getCurrentPosition();
    }
    */

    // might delete
    /**
     * Stop conditions for the lift:
     * - Height is approx. 38 inches or greater
     * - Height is less than or equal to the default fully lowered
     * - Height is less than or equal to 0
     * - Velocity is equal to 0 (lift is stopped)
     * - Acceleration is equal to 0 (lift is stopped)
     * @return
     */
    public boolean stopConditions() {
        // explaining `lift.encoder.getRevolutions() * 9 / 2.54 >= 38`
        // 9 cm per revolution / 2.54 == 38 in

        return lift.encoder.getRevolutions() * 9 / 2.54 >= 38 && lift.getAcceleration() <= 0;
    }

    public MotorEx getMotor() {
        return lift;
    }
    public Motor.Encoder getEncoder() {
        return lift.encoder;
    }


    // public void hold() { lift.set(0.29); }
}
