#pragma once

namespace motormaster {

	public enum class CommandType : System::Int16
	{
		None = 0,
		GoHome = 1,
		Delay = 2,
		MoveAbsolute = 3,
		Push = 4,
		MoveRelative = 5,
		PrecisePush = 6
	};

	public struct Command
	{
		CommandType Type;
		float Position;
		float Velocity;
		float Acceleration;
		float Deacceleration;
		float Band;
		float PushForce;
		float PushDistance;
		int Delay;
		int NextCommandIndex;
	};

	public ref class Axis
	{
	public:
		static Axis^ CreateModbusRtu(System::String^ device, int baudrate, int slave_id);
		static Axis^ CreateModbusTcp(System::String^ address, int port, int slave_id);
		static void Destroy(Axis^ axis);

	private:
		void* ctx;

		// io
	public:
		void SetInputSignal(System::String^ signal, bool level);
		bool GetOutputSignal(System::String^ signal);

		// motion
	public:
		void ConfigMotion(float velocity, float acceleration, float deacceleration);
		void MoveTo(float position);

		void GoHome();
		void MoveAbsolute(
			float position, float velocity,
			float acceleration, float deacceleration, float band);
		void MoveRelative(
			float position, float velocity,
			float acceleration, float deacceleration, float band);
		void Push(
			float force,
			float distance, float velocity);
		void PrecisePush(
			float force,
			float distance, float velocity,
			float force_band, unsigned int force_check_time);

		bool IsMoving();
		bool IsReached();
		bool IsPushEmpty();

		// command
	public:
		void SetCommand(int index, Command command);
		Command GetCommand(int index);
		void ExecuteCommand(Command command);
		void TrigCommand(int index);
		void LoadCommands();
		void SaveCommands();

		// property
	public:
		float Position();
		float Velocity();
		float Torque();
		float ForceSensor();
		int ErrorCode();

		// parameters
	public:
		void LoadParameters();
		void SaveParameters();

		// misc
	public:
		void ResetError();
		void SetServoOnOff(bool on_off);
		void Stop();

	};
}
