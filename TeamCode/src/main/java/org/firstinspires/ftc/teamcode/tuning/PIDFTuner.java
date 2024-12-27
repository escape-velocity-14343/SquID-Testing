package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.PIDFConstants;
import org.firstinspires.ftc.teamcode.lib.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.lib.dash.Drawing;
import org.firstinspires.ftc.teamcode.lib.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.localizer.PinpointLocalizer;

@TeleOp
public class PIDFTuner extends LinearOpMode {

    private PIDFController xController, yController, headingController;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        xController = new PIDFController(PIDFConstants.TRANSLATION_P,
                0, PIDFConstants.TRANSLATION_D, PIDFConstants.TRANSLATION_F);
        yController = new PIDFController(PIDFConstants.TRANSLATION_P,
                0, PIDFConstants.TRANSLATION_D, PIDFConstants.TRANSLATION_F);
        headingController = new PIDFController(PIDFConstants.HEADING_P,
                0, PIDFConstants.HEADING_D, PIDFConstants.HEADING_F);

        waitForStart();

        while (opModeIsActive()) {

            xController.setPIDF(PIDFConstants.TRANSLATION_P,
                    0, PIDFConstants.TRANSLATION_D, PIDFConstants.TRANSLATION_F);
            yController.setPIDF(PIDFConstants.TRANSLATION_P,
                    0, PIDFConstants.TRANSLATION_D, PIDFConstants.TRANSLATION_F);
            headingController.setPIDF(PIDFConstants.HEADING_P,
                    0, PIDFConstants.HEADING_D, PIDFConstants.HEADING_F);

            drive.setDrivePowersFieldCentric(
                    getPIDFOutput(drive.localizer.getPose(), new Pose2d(PIDFConstants.TARGET_X,
                            PIDFConstants.TARGET_Y, Math.toRadians(PIDFConstants.TARGET_HEADING)))
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
