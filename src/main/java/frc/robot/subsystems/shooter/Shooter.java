package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Shooter extends SubsystemBase {

  double kP = 50;
  double kI = 0;
  double kD = 0;
  double kS = 0;
  double kV = 0;
  double kA = 0;

  private MechanismPositionConfig shooterLeftPositionConfig =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(Meters.convertFrom(29.47, Inches)))
          .withMaxRobotLength(Meters.of(Meters.convertFrom(27.5, Inches)))
          .withMovementPlane(Plane.XY)
          .withRelativePosition(new Translation3d(0.286, 0, 0.497)); // TODO: Get Real position

  private MechanismPositionConfig shooterRightPositionConfig =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(Meters.convertFrom(29.47, Inches)))
          .withMaxRobotLength(Meters.of(Meters.convertFrom(27.5, Inches)))
          .withMovementPlane(Plane.XZ)
          .withRelativePosition(new Translation3d(0.286, 0, 0.497)); // TODO: Get Real position

  private SmartMotorControllerConfig leftConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              kP, kI, kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              kP, kI, kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          .withSimFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          // Telemetry name and verbosity level
          .withTelemetry("ShooterLeftMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorControllerConfig rightConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              kP, kI, kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              kP, kI, kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          .withSimFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          // Telemetry name and verbosity level
          .withTelemetry("ShooterRightMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  @AutoLog
  public static class ShooterInputs {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final ShooterInputsAutoLogged shooterRightInputs = new ShooterInputsAutoLogged();
  private final ShooterInputsAutoLogged shooterLeftInputs = new ShooterInputsAutoLogged();

  private TalonFX leftMotor = new TalonFX(14);
  private TalonFX rightMotor = new TalonFX(15);

  private SmartMotorController leftMotorController =
      new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60(1), leftConfig);
  private SmartMotorController rightMotorController =
      new TalonFXWrapper(rightMotor, DCMotor.getKrakenX60(1), rightConfig);

  private FlyWheelConfig leftShooterConfig =
      new FlyWheelConfig(leftMotorController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(1000))
          // Telemetry name and verbosity for the shooter.
          .withTelemetry("ShooterLeftMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(shooterLeftPositionConfig);

  private FlyWheelConfig rightShooterConfig =
      new FlyWheelConfig(rightMotorController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(1000))
          // Telemetry name and verbosity for the shooter.
          .withTelemetry("ShooterRightMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(shooterRightPositionConfig);

  private FlyWheel leftShooter = new FlyWheel(leftShooterConfig);
  private FlyWheel rightShooter = new FlyWheel(rightShooterConfig);

  private void updateInputs() {
    shooterRightInputs.velocity = rightShooter.getSpeed();
    shooterRightInputs.setpoint =
        rightMotorController.getMechanismSetpointVelocity().orElse(RPM.of(0));
    shooterRightInputs.volts = rightMotorController.getVoltage();
    shooterRightInputs.current = rightMotorController.getStatorCurrent();
    shooterLeftInputs.velocity = leftShooter.getSpeed();
    shooterLeftInputs.setpoint =
        leftMotorController.getMechanismSetpointVelocity().orElse(RPM.of(0));
    shooterLeftInputs.volts = leftMotorController.getVoltage();
    shooterLeftInputs.current = leftMotorController.getStatorCurrent();
  }

  public AngularVelocity getRightVelocity() {
    return rightShooter.getSpeed();
  }

  public AngularVelocity getLeftVelocity() {
    return leftShooter.getSpeed();
  }

  public Command setRightVelocity(AngularVelocity speed) {
    return rightShooter.setSpeed(speed);
  }

  public Command setLeftVelocity(AngularVelocity speed) {
    return leftShooter.setSpeed(speed);
  }

  public Command setRight(double dutyCycle) {
    return rightShooter.set(dutyCycle);
  }

  public Command setLeft(double dutyCycle) {
    return leftShooter.set(dutyCycle);
  }

  public LoggedMechanism2d getLeftShooterGeneratedMechanism2d() {
    return new LoggedMechanism2d(
        leftShooter.getMechanismLigament().getLineWeight(),
        leftShooter.getMechanismLigament().getLength(),
        leftShooter.getMechanismLigament().getColor());
  }

  public LoggedMechanism2d getRightShooterGeneratedMechanism2d() {
    return new LoggedMechanism2d(
        rightShooter.getMechanismLigament().getLineWeight(),
        rightShooter.getMechanismLigament().getLength(),
        rightShooter.getMechanismLigament().getColor());
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.recordOutput("Mech2D/LeftShooter", getLeftShooterGeneratedMechanism2d());
    Logger.recordOutput("Mech2D/RightShooter", getRightShooterGeneratedMechanism2d());
    Logger.processInputs("RobotState/ShooterRight", shooterRightInputs);
    rightShooter.updateTelemetry();
    Logger.processInputs("RobotState/ShooterLeft", shooterLeftInputs);
    leftShooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    leftShooter.simIterate();
    rightShooter.simIterate();
  }
}
