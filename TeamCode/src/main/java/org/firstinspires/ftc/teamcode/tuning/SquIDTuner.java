package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.PIDFConstants;
import org.firstinspires.ftc.teamcode.constants.SquIDConstants;
import org.firstinspires.ftc.teamcode.lib.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.lib.controllers.SquIDController;
import org.firstinspires.ftc.teamcode.lib.dash.Drawing;
import org.firstinspires.ftc.teamcode.lib.drivetrain.MecanumDrive;

@TeleOp
public class SquIDTuner extends LinearOpMode {

    private SquIDController xController, yController, headingController;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        xController = new SquIDController(SquIDConstants.TRANSLATION_P);
        yController = new SquIDController(SquIDConstants.TRANSLATION_P);
        headingController = new SquIDController(SquIDConstants.HEADING_P);

        waitForStart();

        while (opModeIsActive()) {

            xController.setP(SquIDConstants.TRANSLATION_P);
            yController.setP(SquIDConstants.TRANSLATION_P);
            headingController.setP(SquIDConstants.HEADING_P);

            drive.setDrivePowersFieldCentric(
                    getPIDFOutput(drive.localizer.getPose(), new Pose2d(SquIDConstants.TARGET_X,
                            SquIDConstants.TARGET_Y, Math.toRadians(SquIDConstants.TARGET_HEADING)))
            );

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.localizer.getPose().position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    public PoseVelocity2d getPIDFOutput(Pose2d robotPose, Pose2d targetPose) {

        // compute heading error
        double hError = targetPose.heading.toDouble() - robotPose.heading.toDouble();
        hError = AngleUnit.normalizeRadians(hError);

        // compute outputs
        double xOutput = xController.calculate(robotPose.position.x, targetPose.position.x);
        double yOutput = yController.calculate(robotPose.position.y, targetPose.position.y);
        double hOutput = headingController.calculate(hError, 0);

        return new PoseVelocity2d(
                new Vector2d(
                        xOutput,
                        yOutput
                ),
                hOutput
        );
    }
}
