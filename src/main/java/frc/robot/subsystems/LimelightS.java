// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.OdometryManager;

import static frc.robot.Constants.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * @author Valentine King
 */

public class LimelightS extends SubsystemBase {
  OdometryManager odometryManager;
  Supplier<Rotation2d> turretAngleSupplier;
  ArrayList<Pose2d> targetPoseList = new ArrayList<>();
  PhotonCamera limelight = new PhotonCamera("gloworm");

  // Sim stuff
  //SimVisionSystem limelightSimVisionSystem;
  Trigger hasSteadyTarget = new Trigger(() -> limelight.getLatestResult().hasTargets()).debounce(0.5);
  // Data filtering
  @Log(methodName = "getFilteredXOffset")
  @Log(methodName = "getFilteredDistance")
  private FilterValues filterValues;

  private LinearFilter xOffsetFilter;
  private LinearFilter distanceFilter;

  /** Creates a new LimelightS. */
  public LimelightS(OdometryManager odometryManager, Supplier<Rotation2d> turretAngleSupplier,
      Consumer<List<Pose2d>> addFieldVisionTargets) {
    this.turretAngleSupplier = turretAngleSupplier;
    this.odometryManager = odometryManager;

    xOffsetFilter = LinearFilter.singlePoleIIR(LIMELIGHT_FILTER_TIME_CONSTANT,
        LIMELIGHT_FILTER_PERIOD_CONSTANT);
    distanceFilter = LinearFilter.singlePoleIIR(LIMELIGHT_FILTER_TIME_CONSTANT,
        LIMELIGHT_FILTER_PERIOD_CONSTANT);
    NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("gloworm").getEntry("version")
        .setString(PhotonVersion.versionString);
    /* if (RobotBase.isReal()) {
      limelightSimVisionSystem = new SimVisionSystem(
          "gloworm",
          CAMERA_DIAG_FOV_DEGREES, Units.radiansToDegrees(CAMERA_PITCH_RADIANS),
          new Transform2d(
              new Translation2d(
                  Units.inchesToMeters(7), Rotation2d.fromDegrees(0)),
              new Rotation2d()),
          CAMERA_HEIGHT_METERS, 9000, 920, 540, 5);
      // Set up the target ring
      for (int i = 0; i > TAPE_STRIP_COUNT; i++) {
        Pose2d targetPose = HUB_CENTER_POSE
            .transformBy(
                new Transform2d(
                    new Translation2d(
                        HUB_RADIUS_METERS,
                        360.0 * i / TAPE_STRIP_COUNT),
                    Rotation2d.fromDegrees(360.0 * i / TAPE_STRIP_COUNT)));
        targetPoseList.add(targetPose);
        limelightSimVisionSystem.addSimVisionTarget(new SimVisionTarget(
            targetPose,
            TARGET_HEIGHT_METERS,
            Units.inchesToMeters(6),
            Units.inchesToMeters(2)));
      } 
      //addFieldVisionTargets.accept(targetPoseList);
    }*/
  }

  /**
   * Sets the driver mode on the camera.
   * 
   * @param driverMode True to enable driver mode, false to disable driver mode.
   */
  public void setDriverMode(boolean driverMode) {
    limelight.setDriverMode(driverMode);
  }

  /**
   * Turns on or off the LEDs.
   * 
   * @param LED True for on, false for off.
   */
  public void setLED(boolean LED) {
    limelight.setLED(LED ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  /**
   * Adds the latest PhotonVision result to the filters.
   */
  @Override
  public void periodic() {
    // if (!RobotBase.isReal()) {
    // limelightSimVisionSystem.moveCamera(
    // new Transform2d(
    // new Translation2d(
    // CAMERA_CENTER_OFFSET, 0).rotateBy(
    // turretAngleSupplier.get()),
    // turretAngleSupplier.get()),
    // CAMERA_HEIGHT_METERS,
    // Units.radiansToDegrees(CAMERA_PITCH_RADIANS));
    // limelightSimVisionSystem.processFrame(odometryManager.getCurrentRobotPose());
    // }
    // This method will be called once per scheduler run

    PhotonTrackedTarget target = limelight.getLatestResult().hasTargets()
        ? limelight.getLatestResult().getBestTarget()
        : new PhotonTrackedTarget(0, 0, 0, 0, new Transform2d(), new ArrayList<TargetCorner>());
    this.filterValues = new FilterValues(
        xOffsetFilter.calculate(target.getYaw()),
        distanceFilter.calculate(
            PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(target.getPitch()))
                + CAMERA_CENTER_OFFSET
                + HUB_RADIUS_METERS));
    SmartDashboard.putNumber("x offset", this.filterValues.getFilteredXOffset()); // TODO Oblog
    SmartDashboard.putNumber("y offset", this.filterValues.getFilteredDistance());
    // if (hasSteadyTarget.get()) {
    // odometryManager.addVisionMeasurement(this.filterValues.getFilteredDistance(),
    // turretAngleSupplier.get().plus(Rotation2d.fromDegrees(this.filterValues.getFilteredXOffset())));
    // }
  }

  /**
   * Returns the current FilterValues.
   * 
   * @return
   */
  public FilterValues getFilterValues() {
    return this.filterValues;
  }

  /**
   * Returns the filtered X offset.
   */
  public double getFilteredXOffset() {
    return this.filterValues.filteredXOffset;
  }

  /**
   * Returns the filtered distance.
   */
  public double getFilteredDistance() {
    return this.filterValues.filteredDistance;
  }

  /**
   * A data class to store filtered X offset and distance.
   */
  public class FilterValues {
    private double filteredXOffset;
    private double filteredDistance;

    /**
     * Constructs a new FilterValues class.
     * 
     * @param filteredXOffset  the X offset
     * @param filteredDistance the distance
     */
    public FilterValues(double filteredXOffset, double filteredDistance) {
      this.filteredXOffset = filteredXOffset;
      this.filteredDistance = filteredDistance;

    }

    /**
     * Returns the filtered X offset.
     */
    public double getFilteredXOffset() {
      return this.filteredXOffset;
    }

    /**
     * Returns the filtered distance.
     */
    public double getFilteredDistance() {
      return this.filteredDistance;
    }

  }
}
