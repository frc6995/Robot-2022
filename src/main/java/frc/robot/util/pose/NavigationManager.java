// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.interpolation.ShooterInterpolatingTable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.util.NomadMathUtil.*;
import static frc.robot.Constants.*;

/** Add your docs here. */

public class NavigationManager implements Loggable{
    private Supplier<Pose2d> m_poseSupplier;
    private Supplier<Rotation2d> m_drivebaseHeadingSupplier;
    private Supplier<ChassisSpeeds> m_drivebaseVelocitySupplier;
    private Supplier<Rotation2d> m_robotToTurretSupplier;
    private Consumer<Double> m_turretTransformVelocityConsumer;

    // Previous values for numerical differentiation
    private Transform2d lastTOFAdjustedRobotToHubTransform;

    private HubTransformEstimator hubTransformEstimator;
    private VisionHubTransformAdjuster visionHubTransformAdjuster;
    private TimeOfFlightAdjuster timeOfFlightAdjuster;


    public NavigationManager(Supplier<Pose2d> poseSupplier,
        Supplier<Rotation2d> drivebaseHeadingSupplier,
        Supplier<ChassisSpeeds> drivebaseVelocitySupplier,
        Supplier<Rotation2d> robotToTurretSupplier,
        Consumer<Double> turretTransformVelocityConsumer
        ) {
        m_poseSupplier = poseSupplier;
        m_drivebaseHeadingSupplier = drivebaseHeadingSupplier;
        m_drivebaseVelocitySupplier = drivebaseVelocitySupplier;
        m_robotToTurretSupplier = robotToTurretSupplier;
        m_turretTransformVelocityConsumer = turretTransformVelocityConsumer;

        hubTransformEstimator = new HubTransformEstimator(m_poseSupplier);
        visionHubTransformAdjuster = new VisionHubTransformAdjuster(m_robotToTurretSupplier, (transform)->{}/* hubTransformEstimator::setCorrectedRobotToHubTransform */);
        timeOfFlightAdjuster = new TimeOfFlightAdjuster(m_drivebaseVelocitySupplier, this::getRobotToHubTransform);

        lastTOFAdjustedRobotToHubTransform = getTOFAdjustedRobotToHubTransform();
    }

    public void update() {
        //lastTOFAdjustedRobotToHubTransform = getTOFAdjustedRobotToHubTransform();
        hubTransformEstimator.update();
        lastTOFAdjustedRobotToHubTransform = getTOFAdjustedRobotToHubTransform();
        visionHubTransformAdjuster.update();
        //timeOfFlightAdjuster.update();

        m_turretTransformVelocityConsumer.accept(0.0);
           /* (getDirection(getTOFAdjustedRobotToHubTransform()).getRadians()
                - getDirection(lastTOFAdjustedRobotToHubTransform).getRadians()) / 0.02);*/
    }

    //Setters
    @Config
    public void setVisionEnabled(boolean enabled) {
        visionHubTransformAdjuster.setVisionEnabled(enabled);
    }

    public void addVisionMeasurement(PhotonTrackedTarget target) {
        visionHubTransformAdjuster.addVisionMeasurement(target);
    }
    //Transform and pose getters
    
    
    public static boolean getDistanceInRange(double distance) {
        return (
            (ShooterInterpolatingTable.MIN_DISTANCE < distance)
            && (distance < ShooterInterpolatingTable.MAX_DISTANCE)
        );
    }

    @Log
    public boolean getTargetDistanceInRange() {
        return getDistanceInRange(getDistance(getTOFAdjustedRobotToHubTransform()));
    } 

    public Pose2d getCurrentRobotPose() {
        return m_poseSupplier.get();
    }

    public Transform2d getRobotToHubTransform() {
        return hubTransformEstimator.getRobotToHubTransform();
    }

    public Transform2d getTOFAdjustedRobotToHubTransform() {
        return getRobotToHubTransform()/*.plus(timeOfFlightAdjuster.getHubTOFAdjustment())*/; 
    }

    public Transform2d getLastTOFAdjustedRobotToHubTransform() {
        return lastTOFAdjustedRobotToHubTransform;
    }

    public Transform2d getRobotTOFAdjustment() {
        return timeOfFlightAdjuster.getHubTOFAdjustment().inverse();
    }

    public Transform2d getHubTOFAdjustment() {
        return timeOfFlightAdjuster.getHubTOFAdjustment();
    }

    public Transform2d getRobotToCameraTransform() {
        return visionHubTransformAdjuster.getRobotToCamera();
    }
    
    public Transform2d getRobotToTurretTransform() {
        return visionHubTransformAdjuster.getRobotToTurret();
    }

    // for logging only
    //@Log(methodName = "getRadians")
    public Rotation2d getRobotToTurretRotation() {
        return getRobotToTurretTransform().getRotation();
    }

    public Pose2d getEstimatedRobotPose() {
        return HUB_CENTER_POSE.plus( // Turn the hub pose to match the robot
            new Transform2d(
                new Translation2d(),
                m_drivebaseHeadingSupplier.get())
            )
            .plus(
                getRobotToHubTransform().inverse());
    }

}
