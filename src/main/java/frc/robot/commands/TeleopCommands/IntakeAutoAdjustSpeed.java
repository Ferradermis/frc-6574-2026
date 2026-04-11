package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class IntakeAutoAdjustSpeed extends Command {
    AngularVelocity velocity;
    double driveVelocity;
    
    public IntakeAutoAdjustSpeed(Drive drive) {
        double driveVelocity = drive.getModuleVelocityAverage();
        AngularVelocity velocity = RPM.of(2500);
    }

    @Override
    public void initialize() {
        addRequirements(RobotContainer.intakeMainRoller, RobotContainer.intakePivot);
    }

    @Override
    public void execute() {
        if (driveVelocity < 1.5) {
            velocity = RPM.of(2500);
        }
        else if (driveVelocity >= 1.5 && driveVelocity < 3) {
            velocity = RPM.of(3000);
        }
        else {
            velocity = RPM.of(4000);
        }
        RobotContainer.intakePivot.setAngle(Degrees.of(0));
        RobotContainer.intakeMainRoller.setVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isFinished) {

    }
}
