// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightS extends SubsystemBase {
  PhotonCamera limelight = new PhotonCamera("photonvision");


  private FilterValues filterValues;

  private LinearFilter xOffsetFilter;
  private LinearFilter yOffsetFilter;

  /** Creates a new LimelightS. */
  public LimelightS() {
    xOffsetFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT,
        Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);

    yOffsetFilter = LinearFilter.singlePoleIIR(Constants.LIMELIGHT_FILTER_TIME_CONSTANT,
        Constants.LIMELIGHT_FILTER_PERIOD_CONSTANT);
  }

  public void enableDriverMode(boolean driverMode) {
    limelight.setDriverMode(driverMode);
  }

  public void enableLED(boolean LED) {
    limelight.setLED(VisionLEDMode.kOn);
    if (LED == true) {
      limelight.setLED(VisionLEDMode.kOn);
    }
    if (LED == false) {
      limelight.setLED(VisionLEDMode.kOff);
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonTrackedTarget target = limelight.getLatestResult().getBestTarget();
    this.filterValues = new FilterValues(xOffsetFilter.calculate(target.getYaw()),
        yOffsetFilter.calculate(target.getPitch()));
  }

  public FilterValues getFilterValues() {
    return this.filterValues;
  }

  public class FilterValues {
    private double filteredXOffset;
    private double filteredYOffset;

    public FilterValues(double filteredXOffset, double filteredYOffset) {
      this.filteredXOffset = filteredXOffset;
      this.filteredYOffset = filteredYOffset;

    }

    public double getFilteredXOffset() {
      return this.filteredXOffset;
    }

    public double getFilteredYOffset() {
      return this.filteredYOffset;
    }

  }
}
