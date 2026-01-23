package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterPivotSubsystem extends SubsystemBase {

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

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig()
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

  private TalonFX shooterPivotMotor = new TalonFX(0);

  private SmartMotorController shooterPivotMotorController =
      new TalonFXWrapper(shooterPivotMotor, DCMotor.getKrakenX60(1), smcConfig);

  private ArmConfig pivotConfig =
      new ArmConfig(shooterPivotMotorController)
          .withSoftLimits(Degrees.of(-20), Degrees.of(10))
          .withHardLimit(Degrees.of(-30), Degrees.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withLength(Feet.of(3))
          .withMass(Pounds.of(1))
          .withTelemetry("ShooterPivot", TelemetryVerbosity.HIGH);

  private Arm pivot = new Arm(pivotConfig);

  public void updateShooterPivotInputs() {
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
    updateShooterPivotInputs();
    Logger.processInputs("RobotState/ShooterPivot", shooterPivotInputs);
    pivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    pivot.simIterate();
  }
@AutoLogOutput
  public Angle getAngleSetPoint(){
    return shooterPivotMotorController.getMechanismPositionSetpoint().orElse(null);
  }
  
}
