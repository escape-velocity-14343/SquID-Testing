package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivetrain.MecanumDrive;

/**
 * Tuning class to ensure your motors are properly configured and reversed.
 * Originally from the road-runner-ftc backend Roadrunner library under the MIT License
 * Ported to Java by j5155
 */
@TeleOp
public class MecanumMotorDirectionDebugger extends LinearOpMode {
    static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Press play to begin the debugging op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (opModeIsActive()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetry.addLine();

            if (gamepad1.x) {
                drive.leftFront.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Left");
            } else if (gamepad1.y) {
                drive.rightFront.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
            } else if (gamepad1.b) {
                drive.rightBack.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Right");
            } else if (gamepad1.a) {
                drive.leftBack.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Left");
            } else {
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightBack.setPower(0);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
