// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterS extends SubsystemBase {
  private double speed = 0; //The current speed.
  private double setSpeed = 0; //The current target speed.
  private boolean enableAutoAdjust = false;
  private final int SPEED_MARGIN = 10; //speed units of allowable margin between target and actual speed.
  private final double DEFAULT_TARGET_SPEED = 5000;
  private final Debouncer atSpeedDebouncer = new Debouncer(0.5);

  //private final... create the motor controller here.

  //The current state of the shooter
  private SHOOTER_STATE state = SHOOTER_STATE.STOPPED;

  //The possible states of the shooter.
  public static enum SHOOTER_STATE {
    STOPPED //Stable state, speed is under a certain threshold.
    ,SPINUP //Target speed has just increased, wheel is spinning up
    ,READY //Wheel is at target speed
    ,SPINDOWN //Target speed has just decreased, wheel is slowing down to new target
  }
  /** Creates a new ExampleSubsystem. */
  public ShooterS() {
    //Do motor controller setup
  }


  /**
   * The periodic method will    */
  @Override
  public void periodic() {
    if(enableAutoAdjust) {
      setTargetSpeed(updateTargetSpeed());
    }    
    checkState(); //Change state based on inputs.
    handleState(); //Drive outputs based on new state.
    updateTelemetry(); //update the telemetry with the new state 
  }

  public void handleState() {
    // Actually drive the outputs and update the inputs 
  }

  public void checkState() {
    SHOOTER_STATE newState = state;
    // This method will be called once per scheduler run
   
    switch(state) {
      case READY: //If we say we're ready right now
        if (isUnderSpeed()) { // But we're now too slow for the target speed...
          newState = SHOOTER_STATE.SPINUP; // Go to spinup mode.
          //reportBallFired();
        }
        if (isOverSpeed()) {
          newState = SHOOTER_STATE.SPINDOWN;
        }
        break;
      case SPINUP: // If we're spinning up
        if (isOverSpeed()) { // If we're now over speed
          newState = SHOOTER_STATE.SPINDOWN;
        }
        else if (isAtSpeed()) {
          if(Math.abs(setSpeed) < SPEED_MARGIN) { //check if our target speed is ~0, and if we're there yet.
            newState = SHOOTER_STATE.STOPPED; // if we've reached the target of being stopped, set the state accordingly.
          } else {
            newState = SHOOTER_STATE.READY;
          }
        }


      case SPINDOWN: // or if we're trying to slow down
        if (isUnderSpeed()) { // But we're now too slow...
          newState = SHOOTER_STATE.SPINUP; // Go to spinup mode.
        }
        if (isAtSpeed()) {
          if(Math.abs(setSpeed) < SPEED_MARGIN ) { //Before anything else, check if our target speed is ~0, and if we're there yet.
            newState = SHOOTER_STATE.STOPPED; // if we've reached the target of being stopped, set the state accordingly.
          } else {
            newState = SHOOTER_STATE.READY;
          }
        }
        break;
      case STOPPED: // If we're supposedly stopped, do nothing. Our target speed should be 0 now. 
        if (Math.abs(setSpeed) > SPEED_MARGIN ) { // If we're no longer at 0 target speed,
          newState = SHOOTER_STATE.SPINUP; //then spin up to the new target speed.
        }
        break;
      default:
        break;
    }
    setState(newState);
  }

  public void updateTelemetry(){
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("setSpeed", setSpeed);
    SmartDashboard.putString("state", state.toString());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Add/subtract one to speed in whichever direction we need to go.
    speed += Math.signum(setSpeed - speed); //simple bang-bang control system. Fine for getting values to change in simulation.
  }

  public boolean isAtSpeed() {
    //If the speed is not over, and not under, and has been "just right" for all of the last .5 seconds, we're at speed
    return atSpeedDebouncer.calculate(!isUnderSpeed() && !isOverSpeed()); 
  }

  public boolean isUnderSpeed() {
    return (setSpeed - speed) > SPEED_MARGIN;
  }

  public boolean isOverSpeed() {
    return (speed - setSpeed) > SPEED_MARGIN;
  }

  private void setTargetSpeed(double target) {
    setSpeed = target; //Adjust the setpoint given the parameter to the method.
  }

  private void setState(SHOOTER_STATE state) { //This is private because commands should only request some states, and subsystem will verify.
    this.state = state;
  }

  public void requestAutoSpeed() {
    enableAutoAdjust = true;
  }

  public void requestDefaultSpeed() {
    enableAutoAdjust = false;
    setTargetSpeed(DEFAULT_TARGET_SPEED);
  }

  public double updateTargetSpeed() {
    return 300; // return some calculation off the limelight 
  }

  public void requestStop() {
    setTargetSpeed(0);
    enableAutoAdjust = false;
    if (state != SHOOTER_STATE.STOPPED) {
      setState(SHOOTER_STATE.SPINDOWN);
    }
  }

  public SHOOTER_STATE getState() {
    return state;
  }

  public double getSpeed() {
    return speed;
  }

  public double getTargetSpeed() {
    return setSpeed;
  }

  public void simulateSpeedDrop() {
    speed -= 50;
  }
}
