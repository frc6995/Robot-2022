package frc.robot.util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NomadMathUtil {

    public static Rotation2d getDirection(Transform2d transform) {
        return getDirection(transform.getTranslation());
    }

    public static Rotation2d getDirection(Translation2d transform) {
        return new Rotation2d(Math.atan2(transform.getY(), transform.getX()));
    }

    public static double getDistance(Transform2d transform){
        return getDistance(transform.getTranslation());
    }

    public static double getDistance(Translation2d transform) {
        return transform.getNorm();
    }

    
}
