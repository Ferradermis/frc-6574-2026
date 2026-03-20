package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto(Drive drive) {
        addCommands(
            new Shoot(drive).withTimeout(4)
        );
    }
}
