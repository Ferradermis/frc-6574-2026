package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterPivot extends SubsystemBase {

  @AutoLog
  public static class ShooterPivotInputs {
    public Angle pivotPosition = Degrees.of(0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0);
    public Angle pivotDesiredPosition = Degrees.of(0);
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrent = Amps.of(0);
  }

  private final ShooterPivotInputsAutoLogged shooterPivotInputs =
      new ShooterPivotInputsAutoLogged();

  private MechanismPositionConfig positionConfig =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(Meters.convertFrom(29.47, Inches)))
          .withMaxRobotLength(Meters.of(Meters.convertFrom(27.5, Inches)))
          .withMovementPlane(Plane.XZ)
          .withRelativePosition(new Translation3d(0.286, 0, 0.497)); // TODO: Get Real position

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withTelemetry("ShooterPivotMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private TalonFX shooterPivotMotor = new TalonFX(16);

  private SmartMotorController shooterPivotMotorController =
      new TalonFXWrapper(shooterPivotMotor, DCMotor.getKrakenX44(1), smcConfig);

  private PivotConfig pivotConfig =
      new PivotConfig(shooterPivotMotorController)
          .withSoftLimits(Degrees.of(-20), Degrees.of(10))
          .withHardLimit(Degrees.of(-30), Degrees.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withMOI(KilogramSquareMeters.of(1))
          .withTelemetry("ShooterPivotMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(positionConfig);

  private Pivot pivot = new Pivot(pivotConfig);

  private void updateInputs() {
    shooterPivotInputs.pivotPosition = pivot.getAngle();
    shooterPivotInputs.pivotVelocity = shooterPivotMotorController.getMechanismVelocity();
    shooterPivotInputs.pivotAppliedVolts = shooterPivotMotorController.getVoltage();
    shooterPivotInputs.pivotCurrent = shooterPivotMotorController.getStatorCurrent();
  }

  public Command setAngle(Angle angle) {
    return pivot.setAngle(angle);
  }

  public Command set(double dutycycle) {
    return pivot.set(dutycycle);
  }

  public Command sysId() {
    return pivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("RobotState/ShooterPivot", shooterPivotInputs);
    pivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    pivot.simIterate();
  }

  @AutoLogOutput
  public Angle getAngleSetPoint() {
    return shooterPivotMotorController.getMechanismPositionSetpoint().orElse(null);
  }

  public Angle getAngle() {
    return shooterPivotInputs.pivotPosition;
  }

  public AngularVelocity getVelocity() {
    return shooterPivotInputs.pivotVelocity;
  }

  public Angle getSetpointAngle() {
    return shooterPivotInputs.pivotDesiredPosition;
  }

  public Voltage getVoltage() {
    return shooterPivotInputs.pivotAppliedVolts;
  }

  public Current getCurrent() {
    return shooterPivotInputs.pivotCurrent;
  }
}
