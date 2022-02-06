// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MidtakeS;

public class MidtakeC extends CommandBase {
  /** Creates a new MidtakeC. */
  private MidtakeS midtake;

  public MidtakeC(MidtakeS midtake) {
    this.midtake = midtake;
    addRequirements(midtake);
  }

  @Override
  public void initialize() {
    midtake.frontSparkMaxSpeed(Constants.MIDTAKE_FRONT_MOTOR_SPEED);
    midtake.backSparkMaxSpeed(Constants.MIDTAKE_BACK_MOTOR_SPEED);
  }
/**
 * 
 */

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    midtake.frontSparkMaxSpeed(0);
    midtake.backSparkMaxSpeed(0);
  }

  @Override
  public boolean isFinished() {
    boolean isBottomBeamBroken = false;
    boolean isTopBeamBroken = false;
    if (midtake.bottomBeamBroken() == false) {
      isBottomBeamBroken = true;
    }
    if (midtake.topBeamBroken() == true) {
      isTopBeamBroken = true;
    }

    return isBottomBeamBroken || isTopBeamBroken;
  }
}
/** Stops midtake motors when either bottom beam break sensor is cleared or top beam break sensor is triggered
 * 
 * 
 * @author Jonas An
 * @author Ben Su
 */