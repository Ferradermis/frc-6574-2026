package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends SequentialCommandGroup {
    public Shoot(Drive drive) {
        addCommands(
            RobotContainer.shooter.setOptimalVelocity().withTimeout(0.25),
            RobotContainer.shooterPivot.setOptimalAngle().withTimeout(1),
            RobotContainer.shooterTransition.setRightVelocity(Constants.MechanismConstants.SHOOTER_TRANSITION_SPEED).withTimeout(0.5),
            RobotContainer.transition.setVelocity(Constants.MechanismConstants.TRANSITION_SPEED_SHOOT).withTimeout(0.5),
            new WiggleFuel().withTimeout(0.5),
            RobotContainer.shooter.setOptimalVelocity()); 
    }
}