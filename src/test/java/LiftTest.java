import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LiftTest {
    CommandScheduler commandScheduler;
    Elevator elevator;

    @BeforeEach
    void initialize() {
        assert HAL.initialize(500, 0);
        commandScheduler = CommandScheduler.getInstance();
        elevator = new Elevator(new ElevatorIOSim());
    }

    @AfterEach
    void reset() {
        commandScheduler.schedule(ElevatorCommands.stopElevator(elevator));
        commandScheduler.cancelAll();
        elevator = null; // an attempt of creating a new elevator object each iteration
    }

    @Test
    void l2LiftTest() {
        commandScheduler.schedule(ElevatorCommands.goToPosition(elevator, ElevatorConstants.L2_POSITION));
        commandScheduler.run();
        assert elevator.getIO().getPosition() == ElevatorConstants.L2_POSITION;
    }

    @Test
    void l3LiftTest() {
        commandScheduler.schedule(ElevatorCommands.goToPosition(elevator, ElevatorConstants.L3_POSITION));
        commandScheduler.run();
        assert elevator.getIO().getPosition() == ElevatorConstants.L3_POSITION;
    }

    @Test
    void closeElevatorTest() {
        commandScheduler.schedule(ElevatorCommands.closeElevator(elevator));
        commandScheduler.run();
        assert elevator.getIO().isPressed();
    }
}
