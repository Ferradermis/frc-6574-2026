package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class StartShooter extends SequentialCommandGroup {
    public StartShooter(Drive drive) {
        addCommands(
            RobotContainer.shooter.setOptimalVelocity());
    }

}