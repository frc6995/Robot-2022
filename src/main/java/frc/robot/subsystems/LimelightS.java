package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * @author Valentine King
 */

public class LimelightS extends SubsystemBase implements Loggable {
  PhotonCamera limelight;

  @Log(methodName = "getFilteredXOffset")
  @Log(methodName = "getFilteredDistance")
  private FilterValues filterValues;

  private LinearFilter xOffsetFilter;
  private LinearFilter distanceFilter;

  /** Creates a new LimelightS. */
  public LimelightS() {
    limelight = new PhotonCamera("gloworm");

    xOffsetFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT,
        Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);

    distanceFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT,
        Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);
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
    // This method will be called once per scheduler run
    PhotonTrackedTarget target = limelight.getLatestResult().hasTargets()
        ? limelight.getLatestResult().getBestTarget()
        : new PhotonTrackedTarget(0, 0, 0, 0, new Transform2d(), new ArrayList<TargetCorner>());
    this.filterValues = new FilterValues(
        xOffsetFilter.calculate(target.getYaw()),
        distanceFilter.calculate(
            PhotonUtils.calculateDistanceToTargetMeters(
                Constants.CAMERA_HEIGHT_METERS,
                Constants.TARGET_HEIGHT_METERS,
                Constants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(target.getPitch()))));
    SmartDashboard.putNumber("x offset", this.filterValues.getFilteredXOffset()); // TODO Oblog
    SmartDashboard.putNumber("y offset", this.filterValues.getFilteredDistance());
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
