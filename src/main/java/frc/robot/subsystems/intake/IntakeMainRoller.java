package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
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

public class IntakeMainRoller extends SubsystemBase {

  double kP = 0.25;
  double kI = 0;
  double kD = 0;
  double kS = 0;
  double kV = 0.2;
  double kA = 0;

  public IntakeMainRoller() {
    
  }

  private MechanismPositionConfig positionConfig =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(Meters.convertFrom(29.47, Inches)))
          .withMaxRobotLength(Meters.of(Meters.convertFrom(27.5, Inches)))
          .withMovementPlane(Plane.XZ)
          .withRelativePosition(new Translation3d(-0.209, 0, 0.342)); // TODO: Get Real position

  private SmartMotorControllerConfig rollerConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              kP, kI, kD, RPM.of(6000), RotationsPerSecondPerSecond.of(500))
          .withSimClosedLoopController(
              kP, kI, kD, RPM.of(6000), RotationsPerSecondPerSecond.of(500))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          .withSimFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
          // Telemetry name and verbosity level
          .withTelemetry("IntakeStaticMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(80));

  @AutoLog
  public static class MainRollerInputs {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final MainRollerInputsAutoLogged mainRollerInputs = new MainRollerInputsAutoLogged();

  private TalonFX mainRollerMotor = new TalonFX(Constants.CanIds.INTAKE_MAIN_ROLLERS_ID, Constants.CanIds.MECH_BUS);

  private SmartMotorController mainRollerMotorController =
      new TalonFXWrapper(mainRollerMotor, DCMotor.getKrakenX44(1), rollerConfig);

  private FlyWheelConfig mainRollerConfig =
      new FlyWheelConfig(mainRollerMotorController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(1))
          // Mass of the flywheel.
          .withMass(Pounds.of(5))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(6000))
          // Telemetry name and verbosity for the shooter.
          .withTelemetry("IntakeMainMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(positionConfig);

  private FlyWheel mainRoller = new FlyWheel(mainRollerConfig);

  private void updateInputs() {
    mainRollerInputs.velocity = mainRoller.getSpeed();
    mainRollerInputs.setpoint =
        mainRollerMotorController.getMechanismSetpointVelocity().orElse(RPM.of(0));
    mainRollerInputs.volts = mainRollerMotorController.getVoltage();
    mainRollerInputs.current = mainRollerMotorController.getStatorCurrent();
  }

  public AngularVelocity getVelocity() {
    return mainRoller.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return mainRoller.run(speed);
  }

  public Command set(double dutyCycle) {
    return mainRoller.set(dutyCycle);
  }

  public LoggedMechanismLigament2d getGeneratedMechanism2d() {
    return new LoggedMechanismLigament2d(
        mainRoller.getName(),
        mainRoller.getMechanismLigament().getLength(),
        mainRoller.getMechanismLigament().getAngle(),
        mainRoller.getMechanismLigament().getLineWeight(),
        mainRoller.getMechanismLigament().getColor());
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("RobotState/IntakeMainRoller", mainRollerInputs);
    mainRoller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    mainRoller.simIterate();
  }
}
