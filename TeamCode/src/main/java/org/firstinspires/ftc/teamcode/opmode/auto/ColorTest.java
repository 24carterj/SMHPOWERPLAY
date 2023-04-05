package org.firstinspires.ftc.teamcode.opmode.auto;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="ColorTest")
public class ColorTest extends LinearOpMode {

    private ColorRangeSensor color;
    HardwareMap hwMap = null;

    @Override
    public void runOpMode() {

        color = hardwareMap.get(ColorRangeSensor.class, "color");
        waitForStart();

        while (opModeIsActive()) {;



            telemetry.addLine().addData("Distance: ", color.getDistance(DistanceUnit.INCH));
            NormalizedRGBA colors = color.getNormalizedColors();
            telemetry.addLine().addData("Alpha: ", colors.alpha*100_000);
            telemetry.addLine().addData("Red: ", colors.red*100_000);
            telemetry.addLine().addData("Green: ", colors.green*100_000);
            telemetry.addLine().addData("Blue: ", colors.blue*100_000);
            telemetry.update();
        }
    }
}
