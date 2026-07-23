package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class MovingShotCalculatorTest {
    private static final Pose2d ROBOT_POSE = new Pose2d(0.0, 0.0, new Rotation2d());
    private static final Translation2d HUB_TARGET = new Translation2d(Units.feetToMeters(7.0), 0.0);

    @Test
    void stationaryShotDoesNotChangeAimOrDistance() {
        var result = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds());

        assertEquals(HUB_TARGET.getX(), result.aimTarget().getX(), 1e-9);
        assertEquals(HUB_TARGET.getY(), result.aimTarget().getY(), 1e-9);
        assertEquals(result.actualDistanceFeet(), result.effectiveDistanceFeet(), 1e-9);
    }

    @Test
    void drivingTowardHubReducesEffectiveDistanceWithRobotSignConvention() {
        var stationary = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds());

        var movingToward = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds(1.0, 0.0, 0.0));

        assertTrue(movingToward.effectiveDistanceFeet() < stationary.effectiveDistanceFeet());
    }

    @Test
    void drivingAwayFromHubIncreasesEffectiveDistanceWithRobotSignConvention() {
        var stationary = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds());

        var movingAway = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds(-1.0, 0.0, 0.0));

        assertTrue(movingAway.effectiveDistanceFeet() > stationary.effectiveDistanceFeet());
    }

    @Test
    void strafingShiftsAimOppositeRobotMotion() {
        var strafingLeft = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds(0.0, 1.0, 0.0));

        assertTrue(strafingLeft.aimTarget().getY() < HUB_TARGET.getY());
        assertTrue(strafingLeft.targetHeading().getDegrees() < 0.0);
    }

    @Test
    void movingCompensationUsesMeasuredShotTime() {
        var strafingLeft = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds(0.0, 1.0, 0.0));

        assertEquals(0.967, strafingLeft.timeOfFlightSeconds(), 1e-9);
        assertEquals(-0.967, strafingLeft.aimTarget().getY(), 1e-9);
    }

    @Test
    void trajectoryShowsCurrentPhysicalLaunchHeading() {
        var result = MovingShotCalculator.calculate(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)),
                HUB_TARGET,
                new ChassisSpeeds());

        var trajectory = result.simulation().trajectory();
        double xDelta = trajectory[trajectory.length - 1].getX() - trajectory[0].getX();
        double yDelta = trajectory[trajectory.length - 1].getY() - trajectory[0].getY();
        assertTrue(yDelta > Math.abs(xDelta));
        assertTrue(Math.abs(result.targetHeading().getDegrees()) < 5.0);
    }

    @Test
    void stationaryMappedShotUsesDistanceMapForTrajectoryRange() {
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        Translation2d hubTarget = new Translation2d(Units.feetToMeters(7.0), 0.0);

        var result = MovingShotCalculator.calculate(
                robotPose,
                hubTarget,
                new ChassisSpeeds());

        assertTrue(result.simulation().shouldMake());
        assertTrue(result.simulation().horizontalMissFeet() <= result.simulation().allowedHorizontalMissFeet());
        assertEquals(
                result.simulation().requiredCenterHeightFeet(),
                result.simulation().ballCenterHeightAtHubFeet(),
                1e-9);
        assertEquals(25, result.simulation().trajectory().length);
    }

    @Test
    void closeMovingShotDoesNotExplode() {
        var result = MovingShotCalculator.calculate(
                ROBOT_POSE,
                new Translation2d(Units.feetToMeters(2.0), 0.0),
                new ChassisSpeeds(2.0, 0.0, 0.0));

        assertTrue(Double.isFinite(result.effectiveDistanceFeet()));
        assertTrue(Double.isFinite(result.timeOfFlightSeconds()));
        assertTrue(result.effectiveDistanceFeet() > 0.0);
    }

    @Test
    void backingAwayShotDoesNotExplode() {
        var result = MovingShotCalculator.calculate(
                ROBOT_POSE,
                HUB_TARGET,
                new ChassisSpeeds(-3.0, 0.0, 0.0));

        assertTrue(Double.isFinite(result.effectiveDistanceFeet()));
        assertTrue(Double.isFinite(result.timeOfFlightSeconds()));
        assertTrue(result.effectiveDistanceFeet() > 0.0);
    }
}
