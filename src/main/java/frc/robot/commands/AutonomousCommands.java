package frc.robot.commands;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.drive.Drive;

public class AutonomousCommands {

    Timer timer = new Timer();

    public Command goToPosition(Drive drive, Rotation3d rotation3d, Elevator elevator, Transfer transfer){
        return DriveCommands.goToPosition(drive, rotation3d).
        andThen(ElevatorCommands.goToPosition(elevator, L2_POSITION)).
        andThen(() -> timer.restart()).
        andThen(TransferCommands.coralOutake(transfer)).until(() -> (timer.get() > 3));
    }

    
}
