import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MultiSystemCommands;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.drive.Drive;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.mockito.Mockito.*;

public class MultiSystemCommandsTest {

    @Test
    public void testClearAlgaeLow() {
        Drive drive = mock(Drive.class);
        Elevator elevator = mock(Elevator.class);
        // Stub elevator.getIO() to prevent NPE in ElevatorCommands.goToPosition()
        ElevatorIO elevatorIO = mock(ElevatorIO.class);
        when(elevatorIO.atGoal()).thenReturn(() -> true);
        when(elevator.getIO()).thenReturn(elevatorIO);

        AlgaeOuttake algaeOuttake = mock(AlgaeOuttake.class);

        Command command = MultiSystemCommands.ClearAlgeaLow(drive, elevator, algaeOuttake);
        assertNotNull(command);
    }

    @Test
    public void testClearAlgaeHigh() {
        Drive drive = mock(Drive.class);
        Elevator elevator = mock(Elevator.class);
        // Stub elevator.getIO() to prevent NPE in ElevatorCommands.goToPosition()
        ElevatorIO elevatorIO = mock(ElevatorIO.class);
        when(elevatorIO.atGoal()).thenReturn(() -> true);
        when(elevator.getIO()).thenReturn(elevatorIO);

        AlgaeOuttake algaeOuttake = mock(AlgaeOuttake.class);

        Command command = MultiSystemCommands.ClearAlgeaHigh(drive, elevator, algaeOuttake);
        assertNotNull(command);
    }

    @Test
    public void testGetClosestReefAlgae() {
        Pose2d currentPose = new Pose2d(5, 5, new Rotation2d(15));

        Pose2d closestReef = MultiSystemCommands.getClosestReefAlgae(currentPose);
        assertNotNull(closestReef);
    }

    @Test
    public void testGoToBranch() {
        Drive drive = mock(Drive.class);

        Command command = MultiSystemCommands.goToBranch(drive, true);
        assertNotNull(command);
    }

    @Test
    public void testGetClosestReefBranch() {
        Pose2d currentPose = new Pose2d(5, 5, new Rotation2d(15));

        Pose2d closestBranch = MultiSystemCommands.getClosestReefBranch(currentPose, true);
        assertNotNull(closestBranch);
    }
}