// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Dimensions;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopCommands.DumpFuel;
import frc.robot.commands.TeleopCommands.GoToHome;
import frc.robot.commands.TeleopCommands.Intake;
import frc.robot.commands.TeleopCommands.Shoot;
import frc.robot.commands.TeleopCommands.ShootAuto;
import frc.robot.commands.TeleopCommands.StopShooter;
import frc.robot.commands.TeleopCommands.StowIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Transition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakeFuelRamp;
import frc.robot.subsystems.intake.IntakeMainRoller;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPivot;
import frc.robot.subsystems.shooter.ShooterTransition;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FuelSim;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;
  private final Vision vision;
  public static IntakeFuelRamp fuelRamp;
  public static IntakeMainRoller intakeMainRoller;
  public static IntakePivot intakePivot;
  public static Shooter shooter;
  public static ShooterPivot shooterPivot;
  public static ShooterTransition shooterTransition;
  public static Transition transition;
  public static RobotSim sim;
  public FuelSim fuelSim;


  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(camera0Name, drive::getRotation));
        fuelRamp = new IntakeFuelRamp();
        intakeMainRoller = new IntakeMainRoller();
        intakePivot = new IntakePivot();
        shooter = new Shooter();
        shooterPivot = new ShooterPivot();
        shooterTransition = new ShooterTransition();
        transition = new Transition();
        //sim = new RobotSim();
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        configureFuelSim();
        drive = new Drive(
                new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        fuelRamp = new IntakeFuelRamp();
        intakeMainRoller = new IntakeMainRoller();
        intakePivot = new IntakePivot();
        shooter = new Shooter();
        shooterPivot = new ShooterPivot();
        shooterTransition = new ShooterTransition();
        transition = new Transition();
        sim = new RobotSim();
        configureFuelSimRobot(intakePivot::increaseFuelCount);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        fuelRamp = new IntakeFuelRamp();
        intakeMainRoller = new IntakeMainRoller();
        intakePivot = new IntakePivot();
        shooter = new Shooter();
        shooterPivot = new ShooterPivot();
        shooterTransition = new ShooterTransition();
        transition = new Transition();
        //sim = new RobotSim();
        break;
    }

    NamedCommands.registerCommand("Shoot", new ShootAuto(RPM.of(2000), RPM.of(1500), RPM.of(800)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> drive.getAngleToHub()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.rightBumper().whileTrue(new Intake(RPM.of(3000), RPM.of(-800)));
    controller.rightBumper().whileFalse(new Intake(RPM.of(0), RPM.of(0)));

    controller.leftBumper().whileTrue(new Shoot(RPM.of(3000), RPM.of(1500), RPM.of(800)));
    controller.leftBumper().whileFalse(new StopShooter(RPM.of(0), RPM.of(0), RPM.of(0)));

    controller2.a().onTrue(new StowIntake());
    controller2.b().onTrue(new GoToHome());

    controller.rightTrigger().whileTrue(new DumpFuel(RPM.of(-3000), RPM.of(-800)));
    controller.rightTrigger().whileFalse(new DumpFuel(RPM.of(0), RPM.of(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();

    fuelSim.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
    }

    private void configureFuelSimRobot(Runnable intakeCallback) {
        fuelSim.registerRobot(
                Dimensions.FULL_WIDTH.in(Meters),
                Dimensions.FULL_LENGTH.in(Meters),
                Dimensions.BUMPER_HEIGHT.in(Meters),
                drive::getPose,
                drive::getFieldSpeeds);
        fuelSim.registerIntake(
                Dimensions.FULL_LENGTH.in(Meters),
                Dimensions.FULL_LENGTH.plus(Inches.of(7)).in(Meters),
                -Dimensions.FULL_WIDTH.div(2).in(Meters),
                Dimensions.FULL_WIDTH.div(2).in(Meters),
                () -> intakePivot.isDeployed(),
                intakeCallback);
    }

    public Drive getDrive() {
        return drive;
    }
}
