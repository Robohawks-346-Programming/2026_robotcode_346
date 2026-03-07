package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climb extends SubsystemBase {
	private enum ClimbState {
		IDLE,
		MOVING_UP,
		MOVING_DOWN,
		HOLDING
	}

	private final ClimbIO io;
	private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

	private ClimbState state = ClimbState.IDLE;
	private double targetMotorRotations = 0.0;

	public climb(ClimbIO io) {
		this.io = io;
	}

	public void moveOneOutputRevolution() {
		targetMotorRotations = inputs.motorPositionRotations + ClimbConstants.MOTOR_ROTATIONS_PER_COMMAND;
		state = ClimbState.MOVING_UP;
	}

	public void moveOneOutputRevolutionDown() {
		targetMotorRotations = inputs.motorPositionRotations - ClimbConstants.MOTOR_ROTATIONS_PER_COMMAND;
		state = ClimbState.MOVING_DOWN;
	}

	@AutoLogOutput(key = "Climb/AtTarget")
	public boolean atTarget() {
		return Math.abs(targetMotorRotations - inputs.motorPositionRotations) <= ClimbConstants.POSITION_TOLERANCE_ROTATIONS;
	}

	public void stop() {
		state = ClimbState.IDLE;
		io.stop();
	}

	@AutoLogOutput(key = "Climb/State")
	public String getStateName() {
		return state.toString();
	}

	public Command moveOneOutputRevolutionCommand() {
		return Commands.sequence(
				Commands.runOnce(this::moveOneOutputRevolution, this),
				Commands.waitUntil(this::atTarget));
	}

	public Command moveOneOutputRevolutionDownCommand() {
		return Commands.sequence(
				Commands.runOnce(this::moveOneOutputRevolutionDown, this),
				Commands.waitUntil(this::atTarget));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Climb", inputs);

		switch (state) {
			case IDLE -> io.stop();
			case MOVING_UP, MOVING_DOWN -> {
				io.setTargetPositionRotations(targetMotorRotations);
				if (atTarget()) {
					state = ClimbState.HOLDING;
				}
			}
			case HOLDING -> io.setTargetPositionRotations(targetMotorRotations);
		}

		Logger.recordOutput("Climb/TargetMotorRotations", targetMotorRotations);
	}
}
