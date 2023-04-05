package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSubsystem extends SubsystemBase {

    private final SensorColor sensor;
    private boolean autoDetect;

    // open by default to prevent refs from yelling

    public ColorSubsystem(final HardwareMap hwMap, final String name) {
        sensor = new SensorColor(hwMap, "color");
    }
    // Servo turning
    public double getRed() {
        if (autoDetect) return sensor.red();
        else return -100;
    }

    public double getBlue() {
        if (autoDetect) return sensor.blue();
        else return -100;
    }

    public double getGreen() {
        if (autoDetect) return sensor.green();
        else return -100;
    }

    public boolean isAutoDetect() {
        return autoDetect;
    }
    public void switchAutoDetect() {
        autoDetect = !autoDetect;
    }
}
