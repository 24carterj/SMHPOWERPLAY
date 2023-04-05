package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.MoveFB;
import org.firstinspires.ftc.teamcode.commands.MoveLR;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetection;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;

@Disabled
@Autonomous(name="BlueRightVis")
public class VisionSelectNAH extends LinearOpMode {

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    static final double COUNTS_PER_MOTOR_REV = 312;    // Gobilda 5202 19.2:1 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = 45;
    static final double DIST_PER_REV = Math.PI * 4.0;

    DriveSubsystem drive;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        drive = new DriveSubsystem(hardwareMap);

        CommandScheduler.getInstance().reset();

        // CommandScheduler.getInstance().schedule(new InstantCommand(this::requestOpModeStop));

        while (!isStarted()) {
            telemetry.addData("POSITION: ", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();

        CommandScheduler.getInstance().schedule(
            new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(SleeveDetection.ParkingPosition.LEFT, new MoveLR(drive,-30, 0.8));
                    put(SleeveDetection.ParkingPosition.RIGHT, new MoveLR(drive,30, 0.8));
                    put(SleeveDetection.ParkingPosition.CENTER, new WaitCommand(0));
                }},
                sleeveDetection::getPosition
            )
        );
        CommandScheduler.getInstance().schedule(new MoveFB(drive, 55, 0.8));
        CommandScheduler.getInstance().schedule(new InstantCommand(this::requestOpModeStop));

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

        }
    }
}
