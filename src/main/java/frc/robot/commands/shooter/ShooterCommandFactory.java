package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterS;
import frc.robot.util.interpolation.ShooterInterpolatingTable;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

/** Factory class to create shoot commands. */
public class ShooterCommandFactory {

    public static Command createShooterDistanceSpinupC(DoubleSupplier distance, ShooterS shooterS) {
        return createShooterFollowC(
            ()->{
              return ShooterInterpolatingTable.get(
                 distance.getAsDouble() + Constants.CAMERA_CENTER_OFFSET
              ).frontWheelRpm;
            },
            ()->{
              return ShooterInterpolatingTable.get(
                distance.getAsDouble() + Constants.CAMERA_CENTER_OFFSET
              ).backWheelRpm;
            },
            shooterS);
      }

    /**
     * Creates a ShooterFollowC, which uses PID to run the front and back wheels to the given speeds.
     * @param frontSpeed a DoubleSupplier for the desired front wheel speed
     * @param backSpeed a DoubleSupplier for the desired back wheel speed
     * @param shooterS the shooter subsystem
     * @return the ShooterFollowC
     */
    public static Command createShooterFollowC(DoubleSupplier frontSpeed, DoubleSupplier backSpeed, ShooterS shooterS) {
        return new FunctionalCommand(
            ()->{}, 
            ()->{
                shooterS.pidFrontSpeed(frontSpeed.getAsDouble());
                shooterS.pidBackSpeed(backSpeed.getAsDouble());
            },
            interrupted -> {
                shooterS.setFrontSpeed(0);
                shooterS.setBackSpeed(0);
            },
            () -> false,
            shooterS
        )
        .withName("ShooterFollowC");
    }

    public static Command createShooterIdleC(ShooterS shooterS) {
        return new FunctionalCommand(
            ()->{}, 
            ()->{
                shooterS.pidFrontSpeed(1000);
                shooterS.pidBackSpeed(1000);        
            },
            interrupted -> {},
            () -> false,
            shooterS
        )
        .withName("ShooterIdleC");
    
    }

}
