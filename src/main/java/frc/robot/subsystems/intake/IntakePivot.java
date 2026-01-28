package frc.robot.subsystems.intake;

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
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
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

public class IntakePivot extends SubsystemBase {

  @AutoLog
  public static class IntakePivotInputs {
    public Angle pivotPosition = Degrees.of(0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0);
    public Angle pivotDesiredPosition = Degrees.of(0);
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrent = Amps.of(0);
  }

  private final IntakePivotInputsAutoLogged intakePivotInputs = new IntakePivotInputsAutoLogged();

  private MechanismPositionConfig positionConfig =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(Meters.convertFrom(29.47, Inches)))
          .withMaxRobotLength(Meters.of(Meters.convertFrom(27.5, Inches)))
          .withMovementPlane(Plane.XZ)
          .withRelativePosition(new Translation3d(-0.3048, 0, 0.26035)); // TODO: Get Real position

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private TalonFX intakePivotMotor = new TalonFX(12);

  private SmartMotorController intakePivotMotorController =
      new TalonFXWrapper(intakePivotMotor, DCMotor.getKrakenX44(1), smcConfig);

  private PivotConfig pivotConfig =
      new PivotConfig(intakePivotMotorController)
          .withSoftLimits(Degrees.of(-20), Degrees.of(10))
          .withHardLimit(Degrees.of(-30), Degrees.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withMOI(KilogramSquareMeters.of(1))
          .withTelemetry("IntakePivotMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(positionConfig);

  private Pivot pivot = new Pivot(pivotConfig);

  private void updateInputs() {
    intakePivotInputs.pivotPosition = pivot.getAngle();
    intakePivotInputs.pivotVelocity = intakePivotMotorController.getMechanismVelocity();
    intakePivotInputs.pivotAppliedVolts = intakePivotMotorController.getVoltage();
    intakePivotInputs.pivotCurrent = intakePivotMotorController.getStatorCurrent();
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

  public LoggedMechanismLigament2d getGeneratedMechanism2d() {
    return new LoggedMechanismLigament2d(
        pivot.getName(),
        pivot.getMechanismLigament().getLength(),
        pivot.getMechanismLigament().getAngle(),
        pivot.getMechanismLigament().getLineWeight(),
        pivot.getMechanismLigament().getColor());
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("RobotState/IntakePivot", intakePivotInputs);
    pivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    pivot.simIterate();
  }

  @AutoLogOutput
  public Angle getAngleSetPoint() {
    return intakePivotMotorController.getMechanismPositionSetpoint().orElse(null);
  }

  public Angle getAngle() {
    return intakePivotInputs.pivotPosition;
  }

  public AngularVelocity getVelocity() {
    return intakePivotInputs.pivotVelocity;
  }

  public Angle getSetpointAngle() {
    return intakePivotInputs.pivotDesiredPosition;
  }

  public Voltage getVoltage() {
    return intakePivotInputs.pivotAppliedVolts;
  }

  public Current getCurrent() {
    return intakePivotInputs.pivotCurrent;
  }
}
