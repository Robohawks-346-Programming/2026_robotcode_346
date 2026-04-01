package frc.robot.subsystems.shooter;


import java.util.Arrays;
import java.util.Comparator;
import java.util.function.ToDoubleFunction;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


public final class ShooterAutoMap {
    public ShooterAutoMap() {
    }


    private record Marker(double distanceFeet, double threeInchPercent, double twoInchPercent) {
    }


    /**
     * Shooter center is 6.261 inches behind robot center (negative X in robot frame).
     * Distances for auto-shot map are measured from this point.
     */
    // private static final Transform2d SHOOTER_CENTER_FROM_ROBOT_CENTER = new Transform2d(
    //         new Translation2d(-Units.inchesToMeters(14 ), 0.0),
    //         new Rotation2d());


   
    private static final Marker[] MARKERS = new Marker[] {
            new Marker(5.0, 3120.0, 4260.0),
            new Marker(6.0, 3360.0, 4560 ),
            new Marker(7.0, 3566.0, 4848.0),
            new Marker(8.0, 3800, 5260),
            new Marker(9.0, 4120, 5650),
            new Marker(10.0, 4140, 5700),
            new Marker(11.0, 6000, 6000.0),
           
    };


    static {
        Arrays.sort(MARKERS, Comparator.comparingDouble(Marker::distanceFeet));
    }


    public static double getTwoInchRpm(double distanceFeet) {
    return getPercent(distanceFeet, Marker::twoInchPercent);
}


    public static double getThreeInchRpm(double distanceFeet) {
    return getPercent(distanceFeet, Marker::threeInchPercent);
}


    public static double getTwoInchPercent(double distanceFeet) {
        return getPercent(distanceFeet, Marker::twoInchPercent);
    }


    public static double getThreeInchPercent(double distanceFeet) {
        return getPercent(distanceFeet, Marker::threeInchPercent);
    }


    public static double getDistanceFeet(Pose2d robotPose, Translation2d targetTranslation, double offset) {
            Logger.recordOutput("Shooter offset", offset);
            Transform2d SHOOTER_CENTER_FROM_ROBOT_CENTER = new Transform2d(
            new Translation2d(-Units.inchesToMeters(6.261 +offset), 0.0),
            new Rotation2d());
        Translation2d shooterTranslation = robotPose
                .transformBy(SHOOTER_CENTER_FROM_ROBOT_CENTER)
                .getTranslation();
        Logger.recordOutput("Distance hub", Units.metersToFeet(shooterTranslation.getDistance(targetTranslation)));
        return Units.metersToFeet(shooterTranslation.getDistance(targetTranslation));
    }


    private static double getPercent(double distanceFeet, ToDoubleFunction<Marker> selector) {
        if (MARKERS.length == 0) {
            return 0.0;
        }
        if (MARKERS.length == 1) {
            return selector.applyAsDouble(MARKERS[0]);
        }


        if (distanceFeet <= MARKERS[0].distanceFeet()) {
            return interpolate(distanceFeet, MARKERS[0], MARKERS[1], selector);
        }


        for (int i = 0; i < MARKERS.length - 1; i++) {
            Marker a = MARKERS[i];
            Marker b = MARKERS[i + 1];
            if (distanceFeet <= b.distanceFeet()) {
                return interpolate(distanceFeet, a, b, selector);
            }
        }


        int last = MARKERS.length - 1;
        return interpolate(distanceFeet, MARKERS[last - 1], MARKERS[last], selector);
    }


    private static double interpolate(double x, Marker a, Marker b, ToDoubleFunction<Marker> selector) {
        double dx = b.distanceFeet() - a.distanceFeet();
        if (Math.abs(dx) < 1e-9) {
            return selector.applyAsDouble(a);
        }
        double t = (x - a.distanceFeet()) / dx;
        return selector.applyAsDouble(a) + t * (selector.applyAsDouble(b) - selector.applyAsDouble(a));
    }
}



