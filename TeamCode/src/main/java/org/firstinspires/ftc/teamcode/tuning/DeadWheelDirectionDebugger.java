package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.drivetrain.MecanumDrive;

/**
 * Tuning class to ensure your odometry is properly configured and reversed.
 * Originally from the road-runner-ftc backend Roadrunner library under the MIT License
 * Ported to Java by j5155
 */
@TeleOp
public class DeadWheelDirectionDebugger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addLine("Move each dead wheel individually and make sure the direction is correct");
            telemetry.addLine();

            telemetry.addLine("Parallel Dead Wheel (should increase forward)");
            telemetry.addLine("  Wheel $i Position: " + drive.localizer.getPose().position.x);
            telemetry.addLine();

            telemetry.addLine("Perpendicular Dead Wheel (should increase leftward)");
            telemetry.addLine("  Wheel $i Position: " + drive.localizer.getPose().position.x);


            telemetry.update();
        }
    }
}
