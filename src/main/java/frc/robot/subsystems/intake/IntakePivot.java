package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.ironmaple.simulation.IntakeSimulation;
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

  private Boolean isDeployed = false;
  private int fuelCount = 0;

  double kP = 13;
  double kI = 0;
  double kD = 0;
  double kS = 0;
  double kV = 0;
  double kG = 0.5;

  public IntakePivot() {
    
  }

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
              kP, kI, kD, RPM.of(50), RotationsPerSecondPerSecond.of(50))
          .withSimClosedLoopController(
              kP, kI, kD, RPM.of(50), RotationsPerSecondPerSecond.of(50))
          .withFeedforward(new ArmFeedforward(kS, kG, kV))
          .withSimFeedforward(new ArmFeedforward(kS, kG, kV))
          .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(21)))
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(120))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private TalonFX intakePivotMotor = new TalonFX(Constants.CanIds.INTAKE_PIVOT_ID, Constants.CanIds.MECH_BUS);

  private SmartMotorController intakePivotMotorController =
      new TalonFXWrapper(intakePivotMotor, DCMotor.getKrakenX44(1), smcConfig);

  private PivotConfig pivotConfig =
      new PivotConfig(intakePivotMotorController)
          .withSoftLimits(Degrees.of(95), Degrees.of(-10))
          .withHardLimit(Degrees.of(95), Degrees.of(-10))
          .withStartingPosition(Degrees.of(90))
          .withMOI(Inches.of(12), Pounds.of(8))
          .withTelemetry("IntakePivotMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(positionConfig);

  private Pivot pivot = new Pivot(pivotConfig);

  private void updateInputs() {
    intakePivotInputs.pivotPosition = pivot.getAngle();
    intakePivotInputs.pivotDesiredPosition = pivot.getMechanismSetpoint().orElse(Degrees.of(0));
    intakePivotInputs.pivotVelocity = intakePivotMotorController.getMechanismVelocity();
    intakePivotInputs.pivotAppliedVolts = intakePivotMotorController.getVoltage();
    intakePivotInputs.pivotCurrent = intakePivotMotorController.getStatorCurrent();
  }

  public Command setAngle(Angle angle) {
    return pivot.setAngle(angle);
  }

  public Command deployIntake() {
    return this.runOnce(() -> {
        setDeployed(true);
        System.out.println("Deploy Intake"); 
        setAngle(Degrees.of(90));
      }
    );
  }

  public Command retractIntake() {
    setDeployed(false);
    return setAngle(Degrees.of(0));
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
    Logger.recordOutput("Fuel Count", fuelCount);
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

  public void setDeployed(boolean deployed) {
    isDeployed = deployed;
  }

  public Boolean isDeployed() {
    return isDeployed;
  }

  public void increaseFuelCount() {
    fuelCount++;
    System.out.println(fuelCount);
  }

  public void decreaseFuelCount() {
    fuelCount--;
  }

  public int getFuelCount() {
    return fuelCount;
  }
}
