package frc.robot.subsystems;

import static frc.robot.Constants.CAMERA_CENTER_OFFSET;
import static frc.robot.Constants.CAMERA_DIAG_FOV_DEGREES;
import static frc.robot.Constants.CAMERA_HEIGHT_METERS;
import static frc.robot.Constants.CAMERA_HORIZ_RES;
import static frc.robot.Constants.CAMERA_PITCH_RADIANS;
import static frc.robot.Constants.CAMERA_VERT_RES;
import static frc.robot.Constants.HUB_CENTER_POSE;
import static frc.robot.Constants.HUB_RADIUS_METERS;
import static frc.robot.Constants.TAPE_STRIP_COUNT;
import static frc.robot.Constants.TARGET_HEIGHT_METERS;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.OdometryManager;
import frc.robot.util.SimCamera;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * @author Valentine King
 */

public class LimelightS extends SubsystemBase {
  OdometryManager odometryManager;
  Supplier<Rotation2d> turretAngleSupplier;
  PhotonCamera limelight = new PhotonCamera("gloworm");


  // Sim stuff
  SimCamera limelightSimVisionSystem;
  Trigger hasSteadyTarget = new Trigger(() -> limelight.getLatestResult().hasTargets()).debounce(0.5);

  /** Creates a new LimelightS. */
  public LimelightS(OdometryManager odometryManager,
      Consumer<List<Pose2d>> addFieldVisionTargets) {
        NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setString(
          PhotonVersion.versionString
        );
        this.odometryManager = odometryManager;
    this.turretAngleSupplier = ()->odometryManager.getRobotToTurret().getRotation();

    if (!RobotBase.isReal()) {
      limelightSimVisionSystem = new SimCamera(
          "gloworm",
          CAMERA_DIAG_FOV_DEGREES, Units.radiansToDegrees(CAMERA_PITCH_RADIANS),
          new Transform2d(
              new Translation2d(
                  CAMERA_CENTER_OFFSET, Rotation2d.fromDegrees(0)),
              new Rotation2d()),
          CAMERA_HEIGHT_METERS, 9000, CAMERA_HORIZ_RES, CAMERA_VERT_RES, 5);
      // Set up the target ring
      ArrayList<Pose2d> targetPoseList = new ArrayList<>();
      for (int i = 0; i < TAPE_STRIP_COUNT; i++) {

        Pose2d targetPose = HUB_CENTER_POSE
            .transformBy(
                new Transform2d(
                    new Translation2d(
                        HUB_RADIUS_METERS,
                        Rotation2d.fromDegrees(360.0 * i / TAPE_STRIP_COUNT)),
                    Rotation2d.fromDegrees(360.0 * i / TAPE_STRIP_COUNT)));
        targetPoseList.add(targetPose);
      } 
      
      addFieldVisionTargets.accept(targetPoseList);
    }
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
    Transform2d cameraToRobotTrans = odometryManager.getRobotToCamera().inverse();
    if (!RobotBase.isReal()) {
    limelightSimVisionSystem.moveCamera(
      cameraToRobotTrans,
      CAMERA_HEIGHT_METERS,
      Units.radiansToDegrees(CAMERA_PITCH_RADIANS));
      limelightSimVisionSystem.processFrame(odometryManager.getCurrentRobotPose());
    }

    PhotonTrackedTarget target = limelight.getLatestResult().hasTargets()
        ? limelight.getLatestResult().getBestTarget()
        : new PhotonTrackedTarget(0, 0, 0, 0, new Transform2d(), new ArrayList<TargetCorner>());
    double xOffset = target.getYaw();
    double distance = PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS,
      TARGET_HEIGHT_METERS,
      CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(target.getPitch()));
    
    Transform2d cameraToHubTrans = new Transform2d(
      PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-1 * xOffset))
      .times((distance + HUB_RADIUS_METERS)/distance),
      Rotation2d.fromDegrees(0)); //Scale the translation an extra 2 meters to account for the radius of the ring
    if (hasSteadyTarget.get()) {
      odometryManager.addVisionMeasurement(cameraToHubTrans);
    }
  }
}
