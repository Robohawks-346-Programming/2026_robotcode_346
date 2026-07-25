package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public final class MovingShotCalculator {
    private static final Transform2d shooterOffset = new Transform2d(
            new Translation2d(-Units.inchesToMeters(6.261), 0.0),
            new Rotation2d());

    private static final double MIN_DISTANCE_METERS = 1e-6;
    private static final double GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665;
    private static final double LAUNCH_ANGLE_RADIANS = Units.degreesToRadians(55.0);
    private static final double LAUNCH_HEIGHT_METERS = Units.inchesToMeters(15.9325);
    private static final double FUEL_RADIUS_METERS = Units.inchesToMeters(5.91 / 2.0);
    private static final double HUB_RIM_HEIGHT_METERS = Units.inchesToMeters(72.0);
    private static final double HUB_OPENING_INSCRIBED_RADIUS_METERS = Units.inchesToMeters(41.7 / 2.0)-Units.inchesToMeters(1);
    private static final double TARGET_CENTER_HEIGHT_METERS = HUB_RIM_HEIGHT_METERS + FUEL_RADIUS_METERS;
    private static final double EMPIRICAL_SHOT_TIME_SECONDS = 0.967;
    private static final int TRAJECTORY_SAMPLE_COUNT = 25;
    private static final Rotation2d SHOOTER_HEADING_OFFSET = new Rotation2d();

    private MovingShotCalculator() {
    }

    public record MovingShotResult(
            Translation2d aimTarget,
            Rotation2d targetHeading,
            double actualDistanceFeet,
            double effectiveDistanceFeet,
            double timeOfFlightSeconds,
            double twoInchRpm,
            double threeInchRpm,
            boolean shouldMake,
            ShotSimulation simulation) {
    }

    public record ShotSimulation(
            boolean shouldMake,
            double ballCenterHeightAtHubFeet,
            double requiredCenterHeightFeet,
            double horizontalMissFeet,
            double allowedHorizontalMissFeet,
            double fallingEntryTimeSeconds,
            Pose3d[] trajectory) {
    }

    private record CompensationResult(
            Translation2d aimTarget,
            double effectiveDistanceFeet,
            double timeOfFlightSeconds) {
    }

    public static MovingShotResult calculate(
            Pose2d robotPose,
            Translation2d hubTarget,
            ChassisSpeeds robotSpeeds) {

        ChassisSpeeds fieldSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());
        Translation2d fieldVelocity = new Translation2d(
                fieldSpeed.vxMetersPerSecond,
                fieldSpeed.vyMetersPerSecond);

        Translation2d shooterPosition = robotPose.transformBy(shooterOffset).getTranslation();
        Translation2d vectorToHub = hubTarget.minus(shooterPosition);

        double distanceMeters = vectorToHub.getNorm();
        double actualDistanceFeet = Units.metersToFeet(distanceMeters);
        if (distanceMeters < MIN_DISTANCE_METERS) {
            double twoInchRpm = ShooterAutoMap.getTwoInchRpm(actualDistanceFeet);
            double threeInchRpm = ShooterAutoMap.getThreeInchRpm(actualDistanceFeet);
            ShotSimulation simulation = simulateShot(
                    shooterPosition,
                    hubTarget,
                    robotPose.getRotation().plus(SHOOTER_HEADING_OFFSET),
                    fieldVelocity,
                    actualDistanceFeet);
            return new MovingShotResult(
                    hubTarget,
                    vectorToHub.getAngle(),
                    actualDistanceFeet,
                    actualDistanceFeet,
                    0.0,
                    twoInchRpm,
                    threeInchRpm,
                    simulation.shouldMake(),
                    simulation);
        }

        CompensationResult compensation = calculateCompensation(
                shooterPosition,
                hubTarget,
                fieldVelocity,
                actualDistanceFeet);

        double twoInchRpm = ShooterAutoMap.getTwoInchRpm(compensation.effectiveDistanceFeet());
        double threeInchRpm = ShooterAutoMap.getThreeInchRpm(compensation.effectiveDistanceFeet());
        Rotation2d targetHeading = compensation.aimTarget().minus(shooterPosition).getAngle();
        ShotSimulation simulation = simulateShot(
                shooterPosition,
                hubTarget,
                robotPose.getRotation().plus(SHOOTER_HEADING_OFFSET),
                fieldVelocity,
                compensation.effectiveDistanceFeet());

        return new MovingShotResult(
                compensation.aimTarget(),
                targetHeading,
                actualDistanceFeet,
                compensation.effectiveDistanceFeet(),
                compensation.timeOfFlightSeconds(),
                twoInchRpm,
                threeInchRpm,
                simulation.shouldMake(),
                simulation);
    }

    private static CompensationResult calculateCompensation(
            Translation2d shooterPosition,
            Translation2d hubTarget,
            Translation2d fieldVelocity,
            double actualDistanceFeet) {

        double timeOfFlightSeconds = EMPIRICAL_SHOT_TIME_SECONDS;
        Translation2d aimTarget = hubTarget.minus(fieldVelocity.times(timeOfFlightSeconds));
        double effectiveDistanceFeet = Units.metersToFeet(shooterPosition.getDistance(aimTarget));

        return new CompensationResult(aimTarget, effectiveDistanceFeet, timeOfFlightSeconds);
    }

    private static double getLaunchSpeedMetersPerSecond(double distanceFeet) {
        double distanceMeters = Math.max(MIN_DISTANCE_METERS, Units.feetToMeters(distanceFeet));
        double cos = Math.cos(LAUNCH_ANGLE_RADIANS);
        double denominator = 2.0 * cos * cos
                * (distanceMeters * Math.tan(LAUNCH_ANGLE_RADIANS)
                        + LAUNCH_HEIGHT_METERS
                        - TARGET_CENTER_HEIGHT_METERS);
        if (denominator <= 0.0) {
            return 0.0;
        }

        double speedSquared = GRAVITY_METERS_PER_SECOND_SQUARED * distanceMeters * distanceMeters / denominator;
        return speedSquared > 0.0 ? Math.sqrt(speedSquared) : 0.0;
    }

    private static ShotSimulation simulateShot(
            Translation2d shooterPosition,
            Translation2d hubTarget,
            Rotation2d launchHeading,
            Translation2d fieldVelocity,
            double effectiveDistanceFeet) {

        Translation2d launchDirection = new Translation2d(
                launchHeading.getCos(),
                launchHeading.getSin());

        double launchSpeed = getLaunchSpeedMetersPerSecond(effectiveDistanceFeet);
        double horizontalLaunchSpeed = launchSpeed * Math.cos(LAUNCH_ANGLE_RADIANS);
        Translation2d ballHorizontalVelocity = launchDirection.times(horizontalLaunchSpeed).plus(fieldVelocity);

        double horizontalSpeedSquared = ballHorizontalVelocity.getX() * ballHorizontalVelocity.getX()
                + ballHorizontalVelocity.getY() * ballHorizontalVelocity.getY();
        double allowedHorizontalMissMeters = Math.max(0.0, HUB_OPENING_INSCRIBED_RADIUS_METERS - FUEL_RADIUS_METERS);
        if (horizontalSpeedSquared < 1e-9) {
            return new ShotSimulation(
                    false,
                    Units.metersToFeet(LAUNCH_HEIGHT_METERS),
                    Units.metersToFeet(TARGET_CENTER_HEIGHT_METERS),
                    Units.metersToFeet(hubTarget.getDistance(shooterPosition)),
                    Units.metersToFeet(allowedHorizontalMissMeters),
                    0.0,
                    sampleTrajectory(shooterPosition, new Translation2d(), 0.0, 0.0));
        }

        double verticalSpeed = launchSpeed * Math.sin(LAUNCH_ANGLE_RADIANS);
        double fallingEntryTimeSeconds = getFallingHeightCrossingTimeSeconds(verticalSpeed, TARGET_CENTER_HEIGHT_METERS);
        if (Double.isNaN(fallingEntryTimeSeconds)) {
            return new ShotSimulation(
                    false,
                    Units.metersToFeet(LAUNCH_HEIGHT_METERS),
                    Units.metersToFeet(TARGET_CENTER_HEIGHT_METERS),
                    Units.metersToFeet(hubTarget.getDistance(shooterPosition)),
                    Units.metersToFeet(allowedHorizontalMissMeters),
                    0.0,
                    sampleTrajectory(shooterPosition, ballHorizontalVelocity, verticalSpeed, 0.0));
        }

        Translation2d fallingEntryPosition = shooterPosition.plus(ballHorizontalVelocity.times(fallingEntryTimeSeconds));
        double fallingEntryMissMeters = fallingEntryPosition.getDistance(hubTarget);
        boolean shouldMake = fallingEntryMissMeters <= allowedHorizontalMissMeters;

        return new ShotSimulation(
                shouldMake,
                Units.metersToFeet(TARGET_CENTER_HEIGHT_METERS),
                Units.metersToFeet(TARGET_CENTER_HEIGHT_METERS),
                Units.metersToFeet(fallingEntryMissMeters),
                Units.metersToFeet(allowedHorizontalMissMeters),
                fallingEntryTimeSeconds,
                sampleTrajectory(shooterPosition, ballHorizontalVelocity, verticalSpeed, fallingEntryTimeSeconds));
    }

    private static double getFallingHeightCrossingTimeSeconds(double verticalSpeed, double targetHeightMeters) {
        double a = -0.5 * GRAVITY_METERS_PER_SECOND_SQUARED;
        double b = verticalSpeed;
        double c = LAUNCH_HEIGHT_METERS - targetHeightMeters;
        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) {
            return Double.NaN;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double rootA = (-b + sqrtDiscriminant) / (2.0 * a);
        double rootB = (-b - sqrtDiscriminant) / (2.0 * a);
        double fallingTime = Math.max(rootA, rootB);
        double verticalVelocityAtCrossing = verticalSpeed
                - GRAVITY_METERS_PER_SECOND_SQUARED * fallingTime;

        return fallingTime > 0.0 && verticalVelocityAtCrossing < 0.0 ? fallingTime : Double.NaN;
    }

    private static Pose3d[] sampleTrajectory(
            Translation2d start,
            Translation2d horizontalVelocity,
            double verticalSpeed,
            double endTimeSeconds) {

        Pose3d[] samples = new Pose3d[TRAJECTORY_SAMPLE_COUNT];
        double endTime = Math.max(0.0, endTimeSeconds);
        for (int i = 0; i < samples.length; i++) {
            double t = samples.length == 1 ? 0.0 : endTime * i / (samples.length - 1);
            Translation2d position = start.plus(horizontalVelocity.times(t));
            double height = LAUNCH_HEIGHT_METERS
                    + verticalSpeed * t
                    - 0.5 * GRAVITY_METERS_PER_SECOND_SQUARED * t * t;
            samples[i] = new Pose3d(position.getX(), position.getY(), Math.max(0.0, height), new Rotation3d());
        }
        return samples;
    }
}
