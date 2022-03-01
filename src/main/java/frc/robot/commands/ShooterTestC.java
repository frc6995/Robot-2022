package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterS;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterTestC extends CommandBase implements Loggable {
  /** Creates a new ShooterC. */
  private ShooterS shooter;
  @Config
  private double frontTargetRPM = 4000;
  @Config
  private double backTargetRPM = 2000;

  public ShooterTestC(ShooterS shooter) {
    this.shooter = shooter;
    this.addRequirements(this.shooter);
    SmartDashboard.putNumber("frontRPM", frontTargetRPM);
    SmartDashboard.putNumber("backRPM", backTargetRPM);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    frontTargetRPM = SmartDashboard.getNumber("frontRPM", 1000);
    backTargetRPM = SmartDashboard.getNumber("backRPM", 1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setBackSpeed(0.5);
    shooter.setFrontSpeed(0.5);
    // shooter.pidFrontSpeed(frontTargetRPM);
    // shooter.pidBackSpeed(backTargetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setFrontSpeed(0);
    this.shooter.setBackSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
