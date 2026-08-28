package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.RPM;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class StowIntake extends SequentialCommandGroup {

    public StowIntake() {
        addCommands(
            RobotContainer.intakePivot.setAngle(Constants.MechanismConstants.INTAKE_STOW_ANGLE).withTimeout(0.5),
            RobotContainer.intakeMainRoller.setVelocity(RPM.of(0))
        );
    }
    
}
