package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.FuelSim;

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

public class Shooter extends SubsystemBase {

  double kP = 0.25;
  double kI = 0;
  double kD = 0;
  double kS = 0;
  double kV = 0.1;
  double kA = 0;

  Pose2d botPose;
  Pose3d hubCenter = new Pose3d();
  double theta = 0;
  double velocity = 0;
  double distance = 0;
  public static double ballVelocity = 0;
  public static Angle hoodAngle = Degrees.of(70);
  Alliance alliance = Alliance.Red;

  public Shooter() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Blue) {
            hubCenter = Constants.HUB_CENTER_BLUE;
            SmartDashboard.putString("Alliance", "You are on Blue Alliance.");
        }
        else if (alliance == Alliance.Red){
            hubCenter = Constants.HUB_CENTER_RED;
            SmartDashboard.putString("Alliance", "You are on Red Alliance.");
        }
  }

  private TalonFX leftMotor = new TalonFX(Constants.CanIds.SHOOTER_LEFT_ID, Constants.CanIds.MECH_BUS);
  private TalonFX rightMotor = new TalonFX(Constants.CanIds.SHOOTER_RIGHT_ID, Constants.CanIds.MECH_BUS);

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
              kP, kI, kD, RPM.of(6000), RotationsPerSecondPerSecond.of(500))
          .withSimClosedLoopController(
              0.1, kI, kD, RPM.of(6000), RotationsPerSecondPerSecond.of(500))
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
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(0.6875)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorControllerConfig rightConfig =
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
          .withTelemetry("ShooterRightMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(0.6875)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withFollowers(new Pair<Object,Boolean>(leftMotor, true));

  @AutoLog
  public static class ShooterInputs {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final ShooterInputsAutoLogged shooterRightInputs = new ShooterInputsAutoLogged();
  private final ShooterInputsAutoLogged shooterLeftInputs = new ShooterInputsAutoLogged();

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
          .withUpperSoftLimit(RPM.of(6000))
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
          .withUpperSoftLimit(RPM.of(6000))
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

  public Command setOptimalVelocity() {
    return rightShooter.setSpeed(this::getDesiredFlywheelVelocity);
  }

  public Command setRightVelocity(AngularVelocity speed) {
    return rightShooter.run(speed);
  }
  

  public Command setLeftVelocity(AngularVelocity speed) {
    return leftShooter.run(speed);
  }

  public Command setRight(double dutyCycle) {
    return rightShooter.set(dutyCycle);
  }

  public Command setLeft(double dutyCycle) {
    return leftShooter.set(dutyCycle);
  }

  public LoggedMechanismLigament2d getLeftShooterGeneratedMechanism2d() {
    return new LoggedMechanismLigament2d(
        leftShooter.getName(),
        leftShooter.getMechanismLigament().getLength(),
        leftShooter.getMechanismLigament().getAngle(),
        leftShooter.getMechanismLigament().getLineWeight(),
        leftShooter.getMechanismLigament().getColor());
  }

  public LoggedMechanismLigament2d getRightShooterGeneratedMechanism2d() {
    return new LoggedMechanismLigament2d(
        rightShooter.getName(),
        rightShooter.getMechanismLigament().getLength(),
        rightShooter.getMechanismLigament().getAngle(),
        rightShooter.getMechanismLigament().getLineWeight(),
        rightShooter.getMechanismLigament().getColor());
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("RobotState/ShooterRight", shooterRightInputs);
    rightShooter.updateTelemetry();
    Logger.processInputs("RobotState/ShooterLeft", shooterLeftInputs);
    leftShooter.updateTelemetry();
    botPose = RobotContainer.drive.getPose();
    double robotX = botPose.getX();
    double robotY = botPose.getY();
    double hubX = hubCenter.getX();
    double hubY = hubCenter.getY();
    double xDifference = robotX - hubX;
    double yDifference = robotY - hubY;
    distance = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2)) - 0.286;
    alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (alliance == Alliance.Blue) {
            hubCenter = Constants.HUB_CENTER_BLUE;
            SmartDashboard.putString("Alliance", "You are on Blue Alliance.");
        }
        else if (alliance == Alliance.Red){
            hubCenter = Constants.HUB_CENTER_RED;
            SmartDashboard.putString("Alliance", "You are on Red Alliance.");
        }
  }

  @Override
  public void simulationPeriodic() {
    leftShooter.simIterate();
    rightShooter.simIterate();
  }

  public static double getTargetVelocity(double d, double t) {
        double g = 9.81;
        double heightDifference = 1.8288 - 0.497;
        double denominator = 2 * Math.cos(t) * Math.cos(t) * (d * Math.tan(t) - heightDifference);

        if (denominator <= 0) {
            return 5000;
        }
        return Math.sqrt((g*d*d) / denominator);
    }

  public static void setShotParams(double d) {
      double lowerBound = Degrees.of(40).in(Radians);
      double upperBound = Degrees.of(70).in(Radians);
      double steps = 60;
      double stepSize = (upperBound - lowerBound) / steps;
      
      double theta = lowerBound;
      double velocity = getTargetVelocity(d, lowerBound);

      for (int i = 1; i <= steps; i++) {
          double x = lowerBound + i * stepSize;
          double y = getTargetVelocity(d, x);
          if (y < velocity) {
              velocity = y;
              theta = x;
          }
      }

      ballVelocity = velocity;
      hoodAngle = Radians.of(theta);
      System.out.println(theta);
      SmartDashboard.putNumber("Hood Angle", Math.toDegrees(hoodAngle.magnitude()));
      Logger.recordOutput("Calculated Angle", hoodAngle);
  }

  public AngularVelocity getDesiredFlywheelVelocity() {
      setShotParams(distance);
      
      // Convert ball velocity (m/s) to flywheel RPM
      // Relationship: ballVelocity = (flywheelRPM / 60) * π * flywheel_radius
      // Therefore: flywheelRPM = (ballVelocity * 60) / (π * flywheel_diameter)
      // desiredRPM is divided by 0.4 to account for external factors like air resistace and wheel slip.
      double flywheelDiameterMeters = Inches.of(4).in(Meters);
      double desiredRPM = ((ballVelocity) * 60) / (Math.PI * flywheelDiameterMeters) / 0.45;
      System.out.println(desiredRPM);
      return RPM.of(Math.max(2000, desiredRPM));
  }

  public static Angle getDesiredHoodAngle() {
    return hoodAngle;
  }
}