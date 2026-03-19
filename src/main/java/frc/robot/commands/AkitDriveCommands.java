// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.


package frc.robot.commands;


import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;


public class AkitDriveCommands {
private static final double DEADBAND = 0.1;
private static final double ROTATION_DEADBAND = 0.05; // Smaller deadband for rotation
private static final double SPEED_SCALE = 0.65; // Scale down max speed (0.25 = 25% of max speed)
private static final double ROTATION_SCALE = 0.50; // Scale down max rotation (0.25 = 25% of max rotation)
    private static final double ANGLE_KP = 0.5;
    private static final double ANGLE_KD = 0.0;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    // Slower constraints for auto-aim so it doesn't spin too fast.
    private static final double AIM_ANGLE_MAX_VELOCITY = 4.0;
    private static final double AIM_ANGLE_MAX_ACCELERATION = 8.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1;
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;


    private AkitDriveCommands() {
    }


    static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband to each axis independently to avoid unintended diagonal drift
        double xProcessed = MathUtil.applyDeadband(x, DEADBAND);
        double yProcessed = MathUtil.applyDeadband(y, DEADBAND);


        // Square for finer low-speed control while preserving sign
        xProcessed = Math.copySign(xProcessed * xProcessed, xProcessed);
        yProcessed = Math.copySign(yProcessed * yProcessed, yProcessed);


        // Return Cartesian translation directly
        return new Translation2d(xProcessed, yProcessed);
    }


    /**
     * Robot-relative drive command using two joysticks (controlling linear and
     * angular velocities).
     *
     * @param drive The drive subsystem
     * @param xSupplier Supplier for forward/backward joystick input (-1 to 1)
     * @param ySupplier Supplier for left/right joystick input (-1 to 1)
     * @param omegaSupplier Supplier for rotation joystick input (-1 to 1)
     * @param useFieldRelative Supplier for whether to use field-relative mode (default: robot-relative)
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<Boolean> useFieldRelative) {
        return Commands.run(
                () -> {
                    // Get raw joystick values
                    double rawX = xSupplier.getAsDouble();
                    double rawY = ySupplier.getAsDouble();
                    double rawOmega = omegaSupplier.getAsDouble();
                   
                    // Get linear velocity from joysticks
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(rawX, rawY);


                    // Process rotation input with deadband and squaring
                    double omega = MathUtil.applyDeadband(rawOmega, ROTATION_DEADBAND);
                    omega = Math.copySign(omega * omega, omega);


                    // Create robot-relative chassis speeds (scaled down for slower driving)
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * SPEED_SCALE,
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * SPEED_SCALE,
                            omega * drive.getMaxAngularSpeedRadPerSec() * ROTATION_SCALE);


                    // Convert to field-relative if requested
                    ChassisSpeeds finalSpeeds = speeds;
                    if (useFieldRelative.get()) {
                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == Alliance.Red
                                && !RobotBase.isSimulation();
                       
                        Rotation2d currentRotation = drive.getRotation();
                        finalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                isFlipped
                                        ? currentRotation.plus(new Rotation2d(Math.PI))
                                        : currentRotation);
                    }
                   
                    // Send speeds to drive subsystem
                    drive.runVelocity(finalSpeeds);
                },
                drive);
    }


    /**
     * Robot-relative drive command (default, no field-relative option).
     * Convenience method that defaults to robot-relative mode.
     */
   


    /**
     * Field relative drive command using joystick for linear control and PID for
     * angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target,
     * or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {


        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);


        // Construct command
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());


                    // Calculate angular speed
                    double omega = angleController.calculate(
                            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());


                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red
                            && !RobotBase.isSimulation();
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive)


                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }
public static Command joystickDriveWithAim(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier, // Added back for manual control
        Supplier<Boolean> aimButton,
        Supplier<Translation2d> aimTargetSupplier) {


    ProfiledPIDController angleController = new ProfiledPIDController(
            ANGLE_KP, 0.0, ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);


    return Commands.run(
            () -> {
                Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(),
                        ySupplier.getAsDouble());


                boolean aimEnabled = false;
                double aimErrorRad = 0.0;
                double aimTargetDeg = drive.getRotation().getDegrees();
                boolean aimWithinThreshold = false;
                double omega;
                if (aimButton.get()) {
                    aimEnabled = true;
                    // AUTO-AIM MODE
                    Translation2d robotPoint = drive.getPose().getTranslation();
                    Rotation2d targetAngle = aimTargetSupplier.get().minus(robotPoint).getAngle();
                   targetAngle = targetAngle.plus(Rotation2d.fromDegrees(180));
                  double error = targetAngle.minus(drive.getRotation()).getRadians();
                    aimErrorRad = error;
                    aimTargetDeg = targetAngle.getDegrees();
                    aimWithinThreshold = Math.abs(error) < Units.degreesToRadians(20);


                        if (Math.abs(error) % Math.PI < Units.degreesToRadians(20)) {
                   omega = 0.2; 
                        } else {
                 omega = angleController.calculate(drive.getRotation().getRadians(), drive.getRotation().getRadians() + error);
                }


                } else {
                    // MANUAL MODE
                    double rawOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ROTATION_DEADBAND);
                    omega = Math.copySign(rawOmega * rawOmega, rawOmega)
                            * drive.getMaxAngularSpeedRadPerSec() * ROTATION_SCALE;
                   
                    // Reset PID so it doesn't "accumulate" while you're driving manually
                    angleController.reset(drive.getRotation().getRadians());
                }


                Logger.recordOutput("Drive/Aim/Enabled", aimEnabled);
                Logger.recordOutput("Drive/Aim/TargetAngleDeg", aimTargetDeg);
                Logger.recordOutput("Drive/Aim/RobotAngleDeg", drive.getRotation().getDegrees());
                Logger.recordOutput("Drive/Aim/ErrorRad", aimErrorRad);
                Logger.recordOutput("Drive/Aim/ErrorDeg", Units.radiansToDegrees(aimErrorRad));
                Logger.recordOutput("Drive/Aim/WithinThreshold", aimWithinThreshold);
                Logger.recordOutput("Drive/Aim/OmegaCmdRadPerSec", omega);


                ChassisSpeeds speeds = new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * SPEED_SCALE,
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * SPEED_SCALE,
                        omega);


                // Use current alliance for field-relative flipping
                boolean isFlipped = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
               
                drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                isFlipped
                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                        : drive.getRotation()));
            },
            drive)
            .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
            .until(() ->  {
                Translation2d targetPos = aimTargetSupplier.get();
                Rotation2d targetAngle = targetPos.minus(drive.getPose().getTranslation())
                                 .getAngle()
                                 .plus(Rotation2d.fromDegrees(180));
                double error = targetAngle.minus(drive.getRotation()).getRadians();
                return Math.abs(error) < Units.degreesToRadians(20);
            }).finallyDo(interrupted -> {
    joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, aimButton);
});
}
    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();


        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),


                // Allow modules to orient
                Commands.run(
                        () -> {
                            drive.runCharacterization(0.0);
                        },
                        drive)
                        .withTimeout(FF_START_DELAY),


                // Start timer
                Commands.runOnce(timer::restart),


                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            drive.runCharacterization(voltage);
                            velocitySamples.add(drive.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        drive)


                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);


                                    Logger.recordOutput("Tested kS", kS);
                                    Logger.recordOutput("Tested kV", kV);


                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));


                                }));
    }


    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();


        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),


                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),


                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),


                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),


                        // Update gyro delta
                        Commands.run(
                                () -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })


                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS)
                                                    / wheelDelta;


                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(Units.metersToInches(wheelRadius))
                                                            + " inches");
                                        })));
    }


    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}



