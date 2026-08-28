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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

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
  Drive driveSubsystem;
  AngularVelocity velocity = RPM.of(2500);
  boolean isOn = false;

  public IntakeMainRoller(Drive drive) {
    driveSubsystem = drive;
  }


  private TalonFX mainRollerLeftMotor = new TalonFX(Constants.CanIds.INTAKE_MAIN_ROLLER_LEFT_ID, Constants.CanIds.MECH_BUS);
  private TalonFX mainRollerRightMotor = new TalonFX(Constants.CanIds.INTAKE_MAIN_ROLLER_RIGHT_ID, Constants.CanIds.MECH_BUS);

  private MechanismPositionConfig positionConfig =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(Meters.convertFrom(29.47, Inches)))
          .withMaxRobotLength(Meters.of(Meters.convertFrom(27.5, Inches)))
          .withMovementPlane(Plane.XZ)
          .withRelativePosition(new Translation3d(-0.209, 0, 0.342)); // TODO: Get Real position

  private SmartMotorControllerConfig leftRollerConfig =
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
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2, 1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(60))
          .withSupplyCurrentLimit(Amps.of(35))
          .withFollowers(new Pair<Object,Boolean>(mainRollerRightMotor, true));

  private SmartMotorControllerConfig rightRollerConfig =
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
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2, 1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(60))
          .withSupplyCurrentLimit(Amps.of(35));

  @AutoLog
  public static class MainRollerInputs {
    public AngularVelocity leftVelocity = DegreesPerSecond.of(0);
    public AngularVelocity leftSetpoint = DegreesPerSecond.of(0);
    public Voltage leftVolts = Volts.of(0);
    public Current leftCurrent = Amps.of(0);

    public AngularVelocity rightVelocity = DegreesPerSecond.of(0);
    public AngularVelocity rightSetpoint = DegreesPerSecond.of(0);
    public Voltage rightVolts = Volts.of(0);
    public Current rightCurrent = Amps.of(0);
  }

  private final MainRollerInputsAutoLogged mainRollerInputs = new MainRollerInputsAutoLogged();

  private SmartMotorController mainRollerMotorLeftController =
      new TalonFXWrapper(mainRollerLeftMotor, DCMotor.getKrakenX60(1), leftRollerConfig);
      private SmartMotorController mainRollerMotorRightController =
      new TalonFXWrapper(mainRollerRightMotor, DCMotor.getKrakenX60(1), rightRollerConfig);

  private FlyWheelConfig mainRollerLeftConfig =
      new FlyWheelConfig(mainRollerMotorLeftController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(1))
          // Mass of the flywheel.
          .withMass(Pounds.of(5))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(6000))
          // Telemetry name and verbosity for the shooter.
          .withTelemetry("IntakeMainMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(positionConfig);

  private FlyWheelConfig mainRollerRightConfig =
      new FlyWheelConfig(mainRollerMotorRightController)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(1))
          // Mass of the flywheel.
          .withMass(Pounds.of(5))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(6000))
          // Telemetry name and verbosity for the shooter.
          .withTelemetry("IntakeMainMech", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(positionConfig);

  private FlyWheel mainRollerLeft = new FlyWheel(mainRollerLeftConfig);

  private FlyWheel mainRollerRight = new FlyWheel(mainRollerRightConfig);

  private void updateInputs() {
    mainRollerInputs.leftVelocity = mainRollerLeft.getSpeed();
    mainRollerInputs.leftSetpoint =
        mainRollerMotorLeftController.getMechanismSetpointVelocity().orElse(RPM.of(0));
    mainRollerInputs.leftVolts = mainRollerMotorLeftController.getVoltage();
    mainRollerInputs.leftCurrent = mainRollerMotorLeftController.getStatorCurrent();

    mainRollerInputs.rightVelocity = mainRollerRight.getSpeed();
    mainRollerInputs.rightSetpoint =
        mainRollerMotorRightController.getMechanismSetpointVelocity().orElse(RPM.of(0));
    mainRollerInputs.rightVolts = mainRollerMotorRightController.getVoltage();
    mainRollerInputs.rightCurrent = mainRollerMotorRightController.getStatorCurrent();
  }

  public AngularVelocity getVelocity() {
    return mainRollerLeft.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return mainRollerLeft.run(speed);
  }

  public Command set(double dutyCycle) {
    return mainRollerLeft.set(dutyCycle);
  }

  public LoggedMechanismLigament2d getGeneratedMechanism2d() {
    return new LoggedMechanismLigament2d(
        mainRollerLeft.getName(),
        mainRollerLeft.getMechanismLigament().getLength(),
        mainRollerLeft.getMechanismLigament().getAngle(),
        mainRollerLeft.getMechanismLigament().getLineWeight(),
        mainRollerLeft.getMechanismLigament().getColor());
  }

  public AngularVelocity getVelocityBasedOnDrive(){
    return velocity;
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("RobotState/IntakeMainRoller", mainRollerInputs);
    mainRollerLeft.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    mainRollerLeft.simIterate();
  }
}
