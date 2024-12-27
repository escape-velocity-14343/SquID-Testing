package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer implements Localizer {
    GoBildaPinpointDriver pinpoint;
    Pose2d currentPose;

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d startPose){
        this.currentPose = startPose;
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }



    @Override
    public void setPose(Pose2d pose) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose.position.x,
                pose.position.y,
                AngleUnit.RADIANS,
                pose.heading.toDouble()));

    }

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     *
     * @return the Localizer's current pose
     */
    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * Updates the Localizer's pose estimate.
     *
     * @return the Localizer's current velocity estimate
     */
    @Override
    public PoseVelocity2d update() {
        pinpoint.update();
        currentPose = new Pose2d(
                pinpoint.getPosition().getX(DistanceUnit.INCH),
                pinpoint.getPosition().getY(DistanceUnit.INCH),
                pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        return new PoseVelocity2d(
                new Vector2d (
                pinpoint.getVelocity().getX(DistanceUnit.INCH),
                pinpoint.getVelocity().getY(DistanceUnit.INCH)
                ),
                pinpoint.getHeadingVelocity()
        );
    }
}
