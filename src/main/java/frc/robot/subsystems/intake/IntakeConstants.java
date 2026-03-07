package frc.robot.subsystems.intake;

public final class IntakeConstants {
	private IntakeConstants() {
	}

	// CAN ID layout (non-drive devices)
	public static final int INTAKE_MOTOR_ID = 56;

	public static final double INTAKE_SPEED_PERCENT = 10.0;
	public static final double INTAKE_DIRECTION = 1.0;

	public static final double INTAKE_SUPPLY_CURRENT_LIMIT_A = 40.0;
	public static final double INTAKE_STATOR_CURRENT_LIMIT_A = 80.0;
}
