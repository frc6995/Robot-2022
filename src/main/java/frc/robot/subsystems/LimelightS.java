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
import org.photonvision.PhotonVersion;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.SimCamera;
import frc.robot.util.pose.NavigationManager;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * @author Valentine King
 */

public class LimelightS extends SubsystemBase implements Loggable {
  Supplier<Rotation2d> turretAngleSupplier;
  PhotonCamera limelight = new PhotonCamera("gloworm");


  // Sim stuff
  SimCamera limelightSimVisionSystem;
  @Log(methodName = "get")
  public final Trigger hasSteadyTarget = new Trigger(() -> limelight.getLatestResult().hasTargets()).debounce(0.5);

  LinearFilter xOffsetFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT, Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);
  LinearFilter distanceFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT, Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);
  @Log
  private double filteredXOffsetRadians = 0;
  @Log
  private double filteredDistanceMeters = 0;
  private double lastValidDistance = 0;
  public final Trigger hasTargetTrigger = new Trigger(this::hasTarget);

  private NavigationManager navigationManager;
  /** Creates a new LimelightS. */
  public LimelightS(
    NavigationManager navigationManager,
    Consumer<List<Pose2d>> addFieldVisionTargets) {
        NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setString(
          PhotonVersion.versionString
        );
        this.navigationManager = navigationManager;

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

  public void ledsOn() {
    setLED(true);
  }

  public void ledsOff() {
    setLED(false);
  }

  public boolean hasTarget(){
    return limelight.getLatestResult().hasTargets();
  }
  @Log
  public double getFilteredXOffset() {
    return filteredXOffsetRadians;
  }
  @Log
  public double getFilteredDistance() {
    return filteredDistanceMeters;
  }

  /**
   * Adds the latest PhotonVision result to the filters.
   */
  @Override
  public void periodic() {
    if ( limelight.getLatestResult().hasTargets()) {
      PhotonTrackedTarget target = limelight.getLatestResult().getBestTarget();
      if(target!=null) {
        double distance = NomadMathUtil.calculateDistanceToTargetMeters(
          CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(target.getPitch()),
          Units.degreesToRadians(
            RobotBase.isReal() ? target.getYaw() : 0 // Sim camera doesn't need a perspective transform.
          )) + Constants.HUB_RADIUS_METERS;
  
        filteredXOffsetRadians = xOffsetFilter.calculate(Units.degreesToRadians(-target.getYaw()));
        filteredDistanceMeters = distanceFilter.calculate(distance);
        lastValidDistance = distance;

        if(hasSteadyTarget.getAsBoolean()) {
          navigationManager.addVisionMeasurement(new Rotation2d(filteredXOffsetRadians), filteredDistanceMeters);
        }

        
      }
       else {

        filteredXOffsetRadians = 0; //xOffsetFilter.calculate(0); // Because this is used in a limited range mechanism (turret), reduce error to zero
        filteredDistanceMeters = distanceFilter.calculate(lastValidDistance);
      }  

    }
    else {
      filteredXOffsetRadians = 0; //xOffsetFilter.calculate(0); // Because this is used in a limited range mechanism (turret), reduce error to zero
      filteredDistanceMeters = distanceFilter.calculate(lastValidDistance);
    }
  }

  @Override
  public void simulationPeriodic() {
      limelightSimVisionSystem.moveCamera(navigationManager.getRobotToCameraTransform().inverse(), Constants.CAMERA_HEIGHT_METERS, Units.radiansToDegrees(Constants.CAMERA_PITCH_RADIANS));
      limelightSimVisionSystem.processFrame(navigationManager.getCurrentRobotPose());
  }
}
