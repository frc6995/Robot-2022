// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.pose;

import static frc.robot.Constants.CAMERA_HEIGHT_METERS;
import static frc.robot.Constants.CAMERA_PITCH_RADIANS;
import static frc.robot.Constants.HUB_RADIUS_METERS;
import static frc.robot.Constants.TARGET_HEIGHT_METERS;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.NomadMathUtil;

/** Add your docs here. */
public class BasicVisionTransformEstimator {

    private Transform2d m_cameraToHubTranslation;

    public BasicVisionTransformEstimator() {
    }

    public Rotation2d getCameraToHubAngleRadians() {
        return NomadMathUtil.getDirection(m_cameraToHubTranslation);
    }

    void addVisionMeasurement(PhotonTrackedTarget target) {
        double xOffset = target.getYaw();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(target.getPitch()));
        
        Transform2d cameraToHubTrans = new Transform2d(
        PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-1 * xOffset))
        .times((distance + HUB_RADIUS_METERS)/distance),
        new Rotation2d()); //Scale the translation an extra 2 meters to account for the radius of the ring
        m_cameraToHubTranslation = cameraToHubTrans;
    }
}
