// Fill out your copyright notice in the Description page of Project Settings.


#include "OtterUSV.h"
#include "Components/PrimitiveComponent.h"

/**
 * @brief Sets default values for this pawn's properties.
 * Configures the actor to tick every frame, allowing for continuous physics calculation.
 */
AOtterUSV::AOtterUSV()
{
	PrimaryActorTick.bCanEverTick = true;

}

/**
 * @brief Called when the game starts or when spawned.
 * Initializes the UDP socket for the ROS2 network bridge and overrides the
 * default Unreal Engine physics properties (Mass and Inertia Tensor) to match
 * the strict Fossen mathematical model for the Otter USV.
 */
void AOtterUSV::BeginPlay()
{
	Super::BeginPlay();

	// Create UDP Socket for ROS2 Bridge
	UDPSocket = FUdpSocketBuilder(TEXT("ROS2BridgeSocket"))
		.AsNonBlocking()
		.AsReusable()
		.WithBroadcast();

	UPrimitiveComponent* BoatMesh = Cast<UPrimitiveComponent>(GetRootComponent());
	if (BoatMesh) {
		// Force UE to this 55kg mass
		BoatMesh->SetMassOverrideInKg(NAME_None, Mass, true);

		// Calculate true Yaw Inertia (Iz + Added Mass Z)
		float TotalYawInertia = InertiaTensor.Z + AddedMassAngular.Z;

		// Get UE default inertia tensor
		FVector CurrentInertia = BoatMesh->GetInertiaTensor();

		//Calculate the scale factor needed to match the Fossen model's yaw inertia and apply it
		if (CurrentInertia.Z != 0.0f) { 

			float ScaleZ = (TotalYawInertia * 10000.0f) / CurrentInertia.Z; 

			BoatMesh->BodyInstance.InertiaTensorScale = FVector(1.0f, 1.0f, ScaleZ);
			BoatMesh->BodyInstance.UpdateMassProperties();
		}

	}
	
}

/**
 * @brief Calculates non-linear thrust based on the official BlueRobotics T200 curves.
 * * @param CurrentAmps The simulated current draw in Amperes (typically -20A to 20A).
 * @return The resulting thrust force in Newtons.
 */
float AOtterUSV::GetT200Thrust(float CurrentAmps)
{
	float ThrustNewtons = 0.0f;

	if (CurrentAmps > 0.0f) {
		//Oficial T200 Forward thrust curve
		ThrustNewtons = (0.088f * FMath::Square(CurrentAmps)) + (0.45f * CurrentAmps);
	}
	else if (CurrentAmps < 0.0f) {
		// Oficial T200 Reverse thrust curvev (less eficient)
		ThrustNewtons = - (0.06f * FMath::Square(CurrentAmps)) - (0.4f * FMath::Abs(CurrentAmps));
	}

	return ThrustNewtons;
}

/**
 * @brief Called every frame.
 * Handles the core physics integration: reads inputs, calculates T200 thruster forces,
 * applies Fossen hydrodynamic damping (Surge, Sway, Yaw), and triggers sensor simulation.
 * * @param DeltaTime The time elapsed since the last frame.
 */
void AOtterUSV::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (GEngine) //Print current input values to the screen for debugging
	{
		GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Yellow, 
			FString::Printf(TEXT("Throttle: %f | Steering: %f"), ForwardInput, RightInput));
	}

	// Grab the 3 Model
	UPrimitiveComponent* BoatMesh = Cast<UPrimitiveComponent>(GetRootComponent());
	
	if (BoatMesh && BoatMesh->IsSimulatingPhysics())
	{
		//Forces the boat to stay awake and responsive to input
		BoatMesh->WakeAllRigidBodies();

		// Normalize inputs so we never exceed 1.0 total before multiplying by 20A
		float LeftMix = ForwardInput + RightInput;
		float RightMix = ForwardInput - RightInput;

		//Find the highest absolute value to scale down if we exceed 1.0 total input
		// (maintains the ratio between forward and turning input while ensuring we don't exceed the max current limit)
		float MaxMix = FMath::Max(FMath::Abs(LeftMix), FMath::Abs(RightMix));
		if (MaxMix > 1.0f) {
			LeftMix /= MaxMix;
			RightMix /= MaxMix;
		}

		// Apply the 20A limit
		float LeftCurrent = LeftMix * 20.0f;
		float RightCurrent = RightMix * 20.0f;

		//Clamp the values to strict physical limits of the T200 thrusters 
		LeftCurrent = FMath::Clamp(LeftCurrent, -20.0f, 20.0f);
		RightCurrent = FMath::Clamp(RightCurrent, -20.0f, 20.0f);

		//Calculate the (non linear) thrust on each motor
		float LeftThrustNewtons = GetT200Thrust(LeftCurrent);
		float RightThrustNewtons = GetT200Thrust(RightCurrent);

		// Combine into surge and yaw components 
		// (assuming the thrusters are aligned with the surge axis and symmetrically placed)
		float TotalSurgeForce = LeftThrustNewtons + RightThrustNewtons;
		float TotalYawTorque = (LeftThrustNewtons - RightThrustNewtons) * (ThrusterDistance / 2.0f);

		//Apply to the boat: surge as a forward force, yaw as a torque around the vertical axis
		FVector ForwardForce = GetActorForwardVector() * TotalSurgeForce * 100.0f;
		BoatMesh->AddForce(ForwardForce);

		FVector TurnTorque = FVector(0.0f, 0.0f, TotalYawTorque * 10000.0f);
		BoatMesh->AddTorqueInRadians(TurnTorque);


		//HYDRODYNAMIC DAMPING (FOSSEN 3-DOF):

		// Get the boat's velocity (local to the boat, not the world)
		FVector GlobalVelocity = BoatMesh->GetComponentVelocity();
		FVector LocalVelocity = GetActorTransform().InverseTransformVectorNoScale(GlobalVelocity);

		FVector GlobalAngularVelocity = BoatMesh->GetPhysicsAngularVelocityInRadians();
		float YawRate = GetActorTransform().InverseTransformVectorNoScale(GlobalAngularVelocity).Z;

		float u = LocalVelocity.X / 100.0f; //UE uses cm/s 
		float v = LocalVelocity.Y / 100.0f;
		float r = YawRate;

		//Calculate the Drag Forces based on the Fossen formulas (result in N / Nm)
		float SurgeDrag = -(LinearDampingTensor.X * u);
		float SwayDrag = -(LinearDampingTensor.Y * v);
		float YawDrag = -(AngularDampingTensor.Z * r);

		//Convert the local drag back to global space and push backwards
		FVector LocalDragForce = FVector(SurgeDrag * 100.0f, SwayDrag * 100.0f, 0.0f);
		FVector GlobalDragForce = GetActorTransform().TransformVectorNoScale(LocalDragForce);

		FVector LocalDragTorque = FVector(0.0f, 0.0f, YawDrag * 10000.f);
		FVector GlobalDragTorque = GetActorTransform().TransformVectorNoScale(LocalDragTorque);

		BoatMesh->AddForce(GlobalDragForce);
		BoatMesh->AddTorqueInRadians(GlobalDragTorque);
	}

	// Update Sensors, apply noise, and format for ROS2
	SimulateSensors(DeltaTime);
}

/**
 * @brief Binds standard Unreal Engine input axes to the local movement functions.
 */
void AOtterUSV::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &AOtterUSV::MoveForward);
	PlayerInputComponent->BindAxis("TurnRight", this, &AOtterUSV::TurnRight);

}


void AOtterUSV::MoveForward(float Value)
{
	ForwardInput = Value;
}

void AOtterUSV::TurnRight(float Value)
{
	RightInput = Value;
}

/**
 * @brief Helper function to generate pseudo-random Gaussian noise.
 * * @param Variance The maximum variance to apply.
 * @return A randomized float between -Variance and +Variance.
 */
float AOtterUSV::GenerateNoise(float Variance) {
	return FMath::FRandRange(-Variance, Variance);
}

/**
 * @brief Packages current physical state into degraded sensor readings.
 * Injects noise, simulates network latency (10Hz capping), and handles random dropouts.
 * Serializes the output into a JSON payload for the Python ROS2 bridge.
 * * @param DeltaTime Time elapsed since last frame.
 */
void AOtterUSV::SimulateSensors(float DeltaTime) {
	
	//Simulate Signal Latency
	LatencyTimer += DeltaTime;
	if (LatencyTimer < 0.1f) return; // 10Hz limit
	LatencyTimer = 0.0f;

	//Check for Sensor Failure
	int32 Status = 1; // 1 = OK, 0 = Failed
	if (FMath::FRand() < SensorFailureProbability) {
		Status = 0;
		GEngine->AddOnScreenDebugMessage(3, 0.5f, FColor::Red, TEXT("[ROS2 BRIDGE] SENSOR DROP OUT!"));
	}

	//Gather Base Sensor Data (without noise) from the boat's current state
	UPrimitiveComponent* BoatMesh = Cast<UPrimitiveComponent>(GetRootComponent());
	if (!BoatMesh) return;

	FVector Velocity = BoatMesh->GetComponentVelocity() / 100.0f; // Convert from cm/s to m/s
	FVector AngularVel = BoatMesh->GetPhysicsAngularVelocityInDegrees();// Convert from rad/s to deg/s
	FVector Location = GetActorLocation(); // Get GPS position in Unreal units (cm)

	//Apply Noise & Math
	float NoisyAccelX = Velocity.X + GenerateNoise(IMUNoiseVariance);
	float NoisyYawRate = AngularVel.Z + GenerateNoise(IMUNoiseVariance);

	float BaseLat = 38.7369f;
	float BaseLon = -9.1388f;
	float CurrentLat = BaseLat + ((Location.X / 100.0f) / 111320.0f) + GenerateNoise(GPSNoiseVariance);
	float CurrentLon = BaseLon + ((Location.Y / 100.0f) / (111320.0f * FMath::Cos(FMath::DegreesToRadians(BaseLat)))) + GenerateNoise(GPSNoiseVariance);

	// LiDAR Hits (Simplified for the payload)
	int32 Hits = 0; // In a full version, run the raycast loop here

	// Format a JSON payload for ROS2 
	// (this is just a string, in a real implementation we would serialize an actual message)
	FString JsonPayload = FString::Printf(TEXT("{\"status\": %d, \"lat\": %f, \"lon\": %f, \"accel_x\": %f, \"yaw_z\": %f, \"lidar\": %d}"),
		Status, CurrentLat, CurrentLon, NoisyAccelX, NoisyYawRate, Hits);

	// Send the payload over UDP to the ROS2 bridge
	SendUDPMessage(JsonPayload);

	// Local UI Feedback
	if (GEngine) {
		GEngine->AddOnScreenDebugMessage(5, 0.1f, FColor::Orange, FString::Printf(TEXT("UDP Sent: %s"), *JsonPayload));
	}
}


/**
 * @brief Overrides the standard EndPlay sequence to clean up networking components.
 * Prevents memory leaks and zombie network ports when stopping the simulator.
 */
void AOtterUSV::EndPlay(const EEndPlayReason::Type EndPlayReason) {
	
	Super::EndPlay(EndPlayReason);

	if (UDPSocket) {
		UDPSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(UDPSocket);
	}
}


/**
 * @brief Formats a standard Unreal Engine string and broadcasts it via UDP.
 * Targeted to localhost (127.0.0.1) on Port 9090 where the Python ROS2 bridge listens.
 * * @param Message The string/JSON payload to broadcast.
 */
void AOtterUSV::SendUDPMessage(FString Message) {
	
	if (!UDPSocket) return;

	// Target Address: 127.0.0.1 (Localhost / WSL) on Port 9090
	TSharedRef<FInternetAddr> TargetAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	FIPv4Address IP;
	FIPv4Address::Parse(TEXT("127.0.0.1"), IP);
	TargetAddr->SetIp(IP.Value);
	TargetAddr->SetPort(9090);

	// Convert UE String to Network Bytes
	TArray<uint8> SendBuffer;
	FTCHARToUTF8 Converted(*Message);
	SendBuffer.Append((uint8*)Converted.Get(), Converted.Length());

	int32 BytesSent = 0;
	UDPSocket->SendTo(SendBuffer.GetData(), SendBuffer.Num(), BytesSent, *TargetAddr);
}
