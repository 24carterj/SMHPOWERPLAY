package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.ClawGrab;
import org.firstinspires.ftc.teamcode.commands.ClawRelease;
import org.firstinspires.ftc.teamcode.commands.LiftDownDist;
import org.firstinspires.ftc.teamcode.commands.LiftUpDist;
import org.firstinspires.ftc.teamcode.commands.MoveFB;
import org.firstinspires.ftc.teamcode.commands.MoveLR;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoLiftSubsystem;

@Autonomous(name="BlueConeMedium")
public class BlueConeMedium extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 312;    // Gobilda 5202 19.2:1 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = 45;
    static final double DIST_PER_REV = Math.PI * 4.0;

    DriveSubsystem drive;
    ClawSubsystem claw;
    TwoLiftSubsystem lift;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DriveSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap, "claw");
        lift = new TwoLiftSubsystem(hardwareMap, 0.5);
        CommandScheduler.getInstance().reset();

        // CommandScheduler.getInstance().schedule(new InstantCommand(this::requestOpModeStop));
        waitForStart();
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new ClawGrab(claw),
                new WaitCommand(1000),
                new MoveFB(drive,210,0.7),
                new WaitCommand(3000),
                new MoveLR(drive, -147, 1),
                new WaitCommand(200),
                new LiftUpDist(lift, 40),
                new WaitCommand(200),
                new MoveFB(drive, 23, 0.6),//.withTimeout(2000),
                new LiftDownDist(lift, 40),
                new ClawRelease(claw),
                new WaitCommand(200),
                /*
                new MoveFB(drive, -18, 0.6),
                new WaitCommand(200),
                new MoveLR(drive, 630, 0.6),
                new WaitCommand(500),
                */

                //new WaitCommand(200)
                /*
                new InstantCommand(claw::release).withTimeout(2000),
                new WaitCommand(500),
                new MoveFB(drive, -15, 1).withTimeout(2000),
                new InstantCommand(claw::grab).withTimeout(2000),
                new LiftDownDist(lift, 30).withTimeout(2000),
                new WaitCommand(500),
                new MoveLR(drive, 100, 1),
                */
                new WaitCommand(20000)

                /*
                new WaitCommand(500),
                new MoveFB(drive,40,1),
                new MoveLR(drive,-10,1),
                new LiftUpDist(lift, 15),
                new MoveFB(drive, 10, 0.2),
                new InstantCommand(claw::release),
                new MoveFB(drive, -10, 0.2),
                new LiftDownDist(lift, 20),
                 */
            )//,
            // new InstantCommand(this::requestOpModeStop)
        );


        // CommandScheduler.getInstance().schedule(new WaitCommand(500));
        // CommandScheduler.getInstance().schedule(new InstantCommand(this::requestOpModeStop));

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

        }

        drive.positionMode();
    }
}
