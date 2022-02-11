// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightS extends SubsystemBase {
  PhotonCamera limelight = new PhotonCamera("gloworm");

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
    PhotonTrackedTarget target = (limelight.getLatestResult().hasTargets() == false)
        ? new PhotonTrackedTarget(0, 0, 0, 0, new Transform2d(), new ArrayList<TargetCorner>())
        : limelight.getLatestResult().getBestTarget();
    this.filterValues = new FilterValues(xOffsetFilter.calculate(target.getYaw()),
        yOffsetFilter.calculate(target.getPitch()));
    SmartDashboard.putNumber("x offset", this.filterValues.getFilteredXOffset());
    SmartDashboard.putNumber("y offset", this.filterValues.getFilteredYOffset());
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
