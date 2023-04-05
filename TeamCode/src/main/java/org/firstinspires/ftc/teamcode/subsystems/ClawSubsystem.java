package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private final ServoEx claw;

    public ClawSubsystem(final HardwareMap hwMap, final String name) {
        claw = new SimpleServo(hwMap, name, 0, 360);
        // claw.setRange(0.1, 0.8);

        // opens claw - default
        // release();
        // isOpen = true;
    }

    // Servo turning
    public void grab() {
        claw.setPosition(0.58);
    }
    public void release() {
        claw.setPosition(0.38);
    }
}
