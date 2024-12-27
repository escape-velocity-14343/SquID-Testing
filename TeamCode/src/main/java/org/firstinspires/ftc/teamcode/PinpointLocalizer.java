package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Experimental localizer that uses the Gobilda Pinpoint sensor
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
public class PinpointLocalizer implements Localizer {
    public static class Params {
        /*
        Set this to the name that your Pinpoint is configured as in your hardware config.
        */
        public String pinpointDeviceName = "pinpoint";
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of the center is a negative number. The Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is: forward of the center is a positive number,
        backwards is a negative number.
         */
        //These are tuned for 3110-0002-0001 Product Insight #1
        // RR localizer note: These units are inches, presets are converted from mm (which is why they are inexact)
        public double xOffset = -3.3071;
        public double yOffset = -6.6142;

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, input the number of ticks per millimeter for that pod.

        RR LOCALIZER NOTE: this is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; the value for the Gobilda 4 Bar Pod is approximately 20.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)

        goBILDA swingarm pod was tuned by GoBILDA at 13.26291192 ticks per mm
        goBILDA 4 bar pod was tuned by GoBILDA at 19.89436789 ticks per mm
         */
        public double encoderResolution = 19.89436789;

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    public static Params PARAMS = new Params();
    GoBildaPinpointDriver pinpoint;
    Pose2d currentPose;

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d startPose){
        FlightRecorder.write("PINPOINT_PARAMS", PARAMS);
        this.currentPose = startPose;
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PARAMS.pinpointDeviceName);



        // RR localizer note: don't love this conversion (change driver?)
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));


        pinpoint.setEncoderResolution(PARAMS.encoderResolution);

        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        setPose(startPose);
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
