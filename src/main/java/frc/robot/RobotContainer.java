package frc.robot;


import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


import frc.robot.Constants;
import frc.robot.commands.AkitDriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
// import frc.robot.subsystems.climb.ClimbIO;
// import frc.robot.subsystems.climb.ClimbIOReal;
// import frc.robot.subsystems.climb.ClimbIOSim;
// import frc.robot.subsystems.climb.climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intakearm.IntakeArm;
import frc.robot.subsystems.intakearm.IntakeArmIO;
import frc.robot.subsystems.intakearm.IntakeArmIOReal;
import frc.robot.subsystems.intakearm.IntakeArmIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterAutoMap;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.*;


public class RobotContainer {
    private static final boolean DRIVE_ENABLED = true;
    private static final double AUTO_INTAKE_EVENT_SECONDS = 5.0;
    private static final double AUTO_SHOOT_EVENT_SECONDS = 5.0;
    private static final double SHOOT_STAGE1_SECONDS = 0.5;
    private static final double SHOOT_STAGE2_SECONDS = 0.5;
    private static final double HARD_AUTO_BACK_METERS = 0.25;
    private static final double HARD_AUTO_BACK_SPEED_MPS = 0.5;


    private final Drive drive;
    private final VisionLocalizer vision;
    private final Shooter shooter;
    private final Intake intake;
    private final IntakeArm intakeArm;
  //  private final climb climbSubsystem;
    private final ShooterAutoMap autoMap;


    private final CommandXboxController controller;
    private final Joystick operatorControl;


    public final JoystickButton BUTTON_1;
    public final JoystickButton BUTTON_2;
    public final JoystickButton BUTTON_3;
    public final JoystickButton BUTTON_4;
    public final JoystickButton BUTTON_5;
    public final JoystickButton BUTTON_6;
    public final JoystickButton BUTTON_7;
    public final JoystickButton BUTTON_8;
    public final JoystickButton BUTTON_9;
    public final JoystickButton BUTTON_10;
    public final JoystickButton BUTTON_11;
    public final JoystickButton BUTTON_12;
    public final JoystickButton BUTTON_13;
    public final JoystickButton BUTTON_14;
    public final JoystickButton BUTTON_15;
    public final JoystickButton BUTTON_16;


    private final POVButton operatorPovUp;
    private final POVButton operatorPovDown;


    private final LoggedDashboardChooser<Command> autoChooser;


   
    private boolean controlsInverted = false;


    public RobotContainer() {
        int driverPort = selectDriverControllerPort();
        controller = new CommandXboxController(driverPort);


        int operatorPort = Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;
        if (Constants.currentMode == Constants.Mode.SIM && operatorPort == driverPort) {
            operatorPort = (driverPort == 0) ? 1 : 0;
        }
        operatorControl = new Joystick(operatorPort);


        BUTTON_1 = new JoystickButton(operatorControl, 1);
        BUTTON_2 = new JoystickButton(operatorControl, 2);
        BUTTON_3 = new JoystickButton(operatorControl, 3);
        BUTTON_4 = new JoystickButton(operatorControl, 4);
        BUTTON_5 = new JoystickButton(operatorControl, 5);
        BUTTON_6 = new JoystickButton(operatorControl, 6);
        BUTTON_7 = new JoystickButton(operatorControl, 7);
        BUTTON_8 = new JoystickButton(operatorControl, 8);
        BUTTON_9 = new JoystickButton(operatorControl, 9);
        BUTTON_10 = new JoystickButton(operatorControl, 10);
        BUTTON_11 = new JoystickButton(operatorControl, 11);
        BUTTON_12 = new JoystickButton(operatorControl, 12);
        BUTTON_13 = new JoystickButton(operatorControl, 13);
        BUTTON_14 = new JoystickButton(operatorControl, 14);
        BUTTON_15 = new JoystickButton(operatorControl, 15);
        BUTTON_16 = new JoystickButton(operatorControl, 16);


        operatorPovUp = new POVButton(operatorControl, 0);
        operatorPovDown = new POVButton(operatorControl, 180);
       
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                vision = createVisionSystem();
                break;


            case SIM:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                vision = createVisionSystem();
                break;


            default:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                vision = new VisionLocalizer(drive::addVisionMeasurement, drive, new VisionIO() {});
        }


        ShooterIO shooterIO = switch (Constants.currentMode) {
            case REAL -> new ShooterIOReal();
            case SIM -> new ShooterIOSim();
            default -> new ShooterIO() {};
        };
        shooter = new Shooter(shooterIO);
        autoMap = new ShooterAutoMap();


        IntakeIO intakeIO = switch (Constants.currentMode) {
            case REAL -> new IntakeIOReal();
            case SIM -> new IntakeIOSim();
            default -> new IntakeIO() {};
        };
        intake = new Intake(intakeIO);


        IntakeArmIO intakeArmIO = switch (Constants.currentMode) {
            case REAL -> new IntakeArmIOReal();
            case SIM -> new IntakeArmIOSim();
            default -> new IntakeArmIO() {};
        };
        intakeArm = new IntakeArm(intakeArmIO);


        // ClimbIO climbIO = switch (Constants.currentMode) {
        //     case REAL -> new ClimbIOReal();
        //     case SIM -> new ClimbIOSim();
        //     default -> new ClimbIO() {};
        // };
        // climbSubsystem = new climb(climbIO);
        vision.setVisionConsumer(drive::addVisionMeasurement);


        configureAutoCommands();
        autoChooser = createAutoChooserSafe();


        configureButtonBindings();
    }


    private static int selectDriverControllerPort() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return 0;
        }
        // In sim, the keyboard joystick is typically port 1 (see simgui-ds.json).
        // Prefer port 1 so keyboard input works without relying on DS connection timing.
        return 1;
    }


    private LoggedDashboardChooser<Command> createAutoChooserSafe() {
        try {
            SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
            chooser.addOption("hardauto", hardAutoCommand());
            return new LoggedDashboardChooser<>("Auto Choices", chooser);
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner autos. Falling back to Do Nothing auto.\n" + e.getMessage(),
                    e.getStackTrace());
            SendableChooser<Command> fallbackChooser = new SendableChooser<>();
            fallbackChooser.setDefaultOption("Do Nothing", Commands.none());
            fallbackChooser.addOption("hardauto", hardAutoCommand());
            return new LoggedDashboardChooser<>("Auto Choices", fallbackChooser);
        }
    }


    private void configureAutoCommands() {
        NamedCommands.registerCommand("Intake", intake.runIntake().withTimeout(AUTO_INTAKE_EVENT_SECONDS));
        // Alias for lowercase marker names in Choreo/PathPlanner.
        NamedCommands.registerCommand("intake", intake.runIntake().withTimeout(AUTO_INTAKE_EVENT_SECONDS));
        NamedCommands.registerCommand("intaketest", intake.runIntake().withTimeout(2));
        NamedCommands.registerCommand("Shoot", autoShootEventCommand());
        NamedCommands.registerCommand("shoot", autoShootEventCommand());
        // NamedCommands.registerCommand("Climb", climbSubsystem.moveOneOutputRevolutionCommand());
        // NamedCommands.registerCommand("climb", climbSubsystem.moveOneOutputRevolutionCommand());
        // NamedCommands.registerCommand("ClimbUp", climbSubsystem.moveOneOutputRevolutionCommand());
        // NamedCommands.registerCommand("climbup", climbSubsystem.moveOneOutputRevolutionCommand());
        // Immediate arm down for starting autos.
        Command armDownAuto = intakeArm.moveDownCommand();
        NamedCommands.registerCommand("ArmDown", armDownAuto);
        NamedCommands.registerCommand("armdown", armDownAuto);
        // delayed version.
        Command armDownDelay1 = Commands.sequence(
                Commands.waitSeconds(1.0),
                intakeArm.moveDownCommand());
        NamedCommands.registerCommand("ArmDownDelay1", armDownDelay1);
        NamedCommands.registerCommand("armdowndelay1", armDownDelay1);
    }


    private Command autoShootEventCommand() {
       
        return Commands.deadline(
                Commands.waitSeconds(AUTO_SHOOT_EVENT_SECONDS),
                shooter.runShoot(),
                Commands.run(drive::stop))
                .finallyDo(interrupted -> {
                    shooter.stop();
                    drive.stop();
                });
    }


    private Command hardAutoCommand() {
        double backSeconds = 0.5;
        double direction = 1.0;
        Command driveBack = Commands.run(
                () -> drive.runVelocity(new ChassisSpeeds(HARD_AUTO_BACK_SPEED_MPS * direction, 0.0, 0.0)),
                drive)
                .withTimeout(backSeconds)
                .andThen(Commands.runOnce(drive::stop, drive));


        return Commands.sequence(
            Commands.race(driveBack,Commands.waitSeconds(0.5)),
               
                Commands.runOnce(drive::stop, drive),
                Commands.waitSeconds(1.0),
                intakeArm.moveDownCommand(),
                intake.runIntake().withTimeout(2),
                stagedShootCommand6ft().withTimeout(AUTO_SHOOT_EVENT_SECONDS));
    }


    /**
     * Creates vision system with automatically detected cameras.
     */
    private VisionLocalizer createVisionSystem() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            return new VisionLocalizer(
                    drive::addVisionMeasurement,
                    drive,
                    new VisionIOPhotonReal(VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0]),
                    new VisionIOPhotonReal(VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1]),
                    new VisionIOPhotonReal(VisionConstants.cameraNames[2], VisionConstants.vehicleToCameras[2]));
        } else {
            return new VisionLocalizer(
                    drive::addVisionMeasurement,
                    drive,
                    new VisionIOPhotonSim(
                            VisionConstants.cameraNames[0],
                            VisionConstants.vehicleToCameras[0],
                            () -> drive.getPose()),
                    new VisionIOPhotonSim(
                            VisionConstants.cameraNames[1],
                            VisionConstants.vehicleToCameras[1],
                            () -> drive.getPose()),
                    new VisionIOPhotonSim(
                            VisionConstants.cameraNames[2],
                            VisionConstants.vehicleToCameras[2],
                            () -> drive.getPose()));
        }
    }


//     private double getAutoShootDistanceFeet() {
//         // Distance from robot to the same hub point used for auto-aim.
//         Translation2d hubTarget = AutoAimCommands.getAimTargetForAlliance();
//         return ShooterAutoMap.getDistanceFeet(
//                 drive.getPose(),
//                 hubTarget);
//     }
    private Command stagedShootCommand() {
        return Commands.sequence(
               
                Commands.run(
                        () -> shooter.setTargets(
                                ShooterConstants.TALON_2_INCH_TARGET_RPM,
                                ShooterConstants.TALON_3_INCH_TARGET_RPM,
                                0.0,
                                0.0),
                        shooter).withTimeout(SHOOT_STAGE1_SECONDS),
                Commands.run(
                        () -> shooter.setTargets(
                                ShooterConstants.TALON_2_INCH_TARGET_RPM,
                                ShooterConstants.TALON_3_INCH_TARGET_RPM,
                                ShooterConstants.NEO_550_SPEED_PERCENT,
                                ShooterConstants.ROLLER_SPEED_PERCENT),
                        shooter).withTimeout(SHOOT_STAGE2_SECONDS),
                shooter.runShoot())
                .finallyDo(interrupted -> shooter.stop());
    }
    private Command stagedShootCommand6ft() {
        return Commands.sequence(
               
                Commands.run(
                        () -> shooter.setTargets(
                                ShooterConstants.TALON_2_INCH_TARGET_RPM_6ft,
                                ShooterConstants.TALON_3_INCH_TARGET_RPM_6ft,
                                0.0,
                                0.0),
                        shooter).withTimeout(SHOOT_STAGE1_SECONDS),
                Commands.run(
                        () -> shooter.setTargets(
                                ShooterConstants.TALON_2_INCH_TARGET_RPM_6ft,
                                ShooterConstants.TALON_3_INCH_TARGET_RPM_6ft,
                                ShooterConstants.NEO_550_SPEED_PERCENT,
                                ShooterConstants.ROLLER_SPEED_PERCENT),
                        shooter).withTimeout(SHOOT_STAGE2_SECONDS),
                shooter.runShoot())
                .finallyDo(interrupted -> shooter.stop());
    }


    private Command stagedShootCommand7ft() {
        return Commands.sequence(
               
                Commands.run(
                        () -> shooter.setTargets(
                                ShooterConstants.TALON_2_INCH_TARGET_RPM_7ft,
                                ShooterConstants.TALON_3_INCH_TARGET_RPM_7ft,
                                0.0,
                                0.0),
                        shooter).withTimeout(SHOOT_STAGE1_SECONDS),
                Commands.run(
                        () -> shooter.setTargets(
                                ShooterConstants.TALON_2_INCH_TARGET_RPM_7ft,
                                ShooterConstants.TALON_3_INCH_TARGET_RPM_7ft,
                                ShooterConstants.NEO_550_SPEED_PERCENT,
                                ShooterConstants.ROLLER_SPEED_PERCENT),
                        shooter).withTimeout(SHOOT_STAGE2_SECONDS),
                shooter.runShoot())
                .finallyDo(interrupted -> shooter.stop());
    }


    private void configureButtonBindings() {
        if (DRIVE_ENABLED) {
            drive.setDefaultCommand(
                    AkitDriveCommands.joystickDrive(
                            drive,
                            () -> controlsInverted ? -controller.getLeftY() : controller.getLeftY(),
                            () -> controlsInverted ? -controller.getLeftX() : controller.getLeftX(),
                            () -> controller.getRightX(),
                            () -> true));
        } else {
            drive.setDefaultCommand(Commands.run(drive::stop, drive));
        }


        // Toggle drive controls inversion with X
        controller.x().onTrue(
                Commands.runOnce(
                        () -> {
                            controlsInverted = !controlsInverted;
                            System.out.println("Drive controls: " + (controlsInverted ? "INVERTED" : "NORMAL"));
                        }));


        // Field-relative is always on (no toggle).


        // Reset robot heading on A
        // controller.a().onTrue(
        //         Commands.runOnce(
        //                 () -> {
        //                     drive.zeroGyro();
        //                     System.out.println("Robot heading reset - current front is now forward");
        //                 },
        //                 drive)
        //                 .ignoringDisable(true));


       
        // controller.start().onTrue(
        //         Commands.runOnce(
        //                 () -> {
        //                     drive.zeroGyro();
        //                     System.out.println("Gyro zeroed - robot heading reset to zero");
        //                 },
        //                 drive)
        //                 .ignoringDisable(true));


       
       


       
        controller.y()
                .whileTrue(intake.runIntake())
                .onFalse(intake.stopIntake());


       


       
      
        var leftBumper = controller.leftBumper();
        var rightBumper = controller.rightBumper();
        var leftTrigger = controller.leftTrigger();


        
        leftBumper.whileTrue(
    AkitDriveCommands.joystickDriveWithAim(
        drive,
        () -> controlsInverted ? -controller.getLeftY() : controller.getLeftY(),
        () -> controlsInverted ? -controller.getLeftX() : controller.getLeftX(),
        // () -> controller.getRightX(),
        // () -> true,
        () -> new Translation2d(11.915129661560059, 4.034599781036377)
    )
);
                

        // Auto-shoot only when BOTH LB and RB are held.
        // rightBumper.and(leftBumper)
        //         .whileTrue(shooter.runAutoShoot(this::getAutoShootDistanceFeet))
        //         .onFalse(shooter.stopCoralIntake());


        // Wheel "X" lock only when LB + RB + LT are all held.
        // leftBumper.and(rightBumper).and(leftTrigger)
        //         .whileTrue(AkitDriveCommands.holdXLock(drive));


       
        controller.povDown()
                .whileTrue(intakeArm.jogDownCommand())
                .onFalse(Commands.runOnce(intakeArm::stop, intakeArm));
        controller.povUp()
                .whileTrue(intakeArm.jogUpCommand())
                .onFalse(Commands.runOnce(intakeArm::stop, intakeArm));


        // controller.povRight().onTrue(climbSubsystem.moveOneOutputRevolutionCommand());
        // controller.povLeft().onTrue(climbSubsystem.moveOneOutputRevolutionDownCommand());
       
       


       
        BUTTON_1.whileTrue(intake.runIntake()).onFalse(intake.stopIntake());
        BUTTON_2.whileTrue(stagedShootCommand()).onFalse(shooter.stopCoralIntake());
        BUTTON_3.whileTrue(stagedShootCommand6ft()).onFalse(shooter.stopCoralIntake());
        BUTTON_4.whileTrue(stagedShootCommand7ft()).onFalse(shooter.stopCoralIntake());
        BUTTON_5.onTrue(intakeArm.moveDownCommand());
        BUTTON_6.onTrue(intakeArm.moveUpCommand());
        BUTTON_7
                .whileTrue(intakeArm.jogDownCommand())
                .onFalse(Commands.runOnce(intakeArm::stop, intakeArm));
        BUTTON_9.onTrue(Commands.runOnce(drive::stopWithX, drive));
        BUTTON_8.whileTrue(intake.runOuttake()).onFalse(intake.stopIntake());
       


        operatorPovDown
                .whileTrue(intakeArm.jogDownCommand())
                .onFalse(Commands.runOnce(intakeArm::stop, intakeArm));
        operatorPovUp
                .whileTrue(intakeArm.jogUpCommand())
                .onFalse(Commands.runOnce(intakeArm::stop, intakeArm));


        // new POVButton(operatorControl, 90).onTrue(climbSubsystem.moveOneOutputRevolutionCommand());
        // new POVButton(operatorControl, 270).onTrue(climbSubsystem.moveOneOutputRevolutionDownCommand());
    }


    public Command getAutonomousCommand() {
        return autoChooser.get();
    }


    public void teleopInit() {
        // Teleop initialization if needed
    }


    public Drive getDrive() {
        return drive;
    }


    public VisionLocalizer getVision() {
        return vision;
    }


    public Shooter getShooter() {
        return shooter;
    }


    public Intake getIntake() {
        return intake;
    }


    public IntakeArm getIntakeArm() {
        return intakeArm;
    }


    // public climb getClimb() {
    //     return climbSubsystem;
    // }
}



