// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Networking.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "OtterUSV.generated.h"

UCLASS()
class TSBPHASE2_API AOtterUSV : public APawn
{
	GENERATED_BODY()

public:
	AOtterUSV();// Sets default values for this pawn's properties

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;


	//FOSSENS 3DOF DINAMIC PARAMETERS:
	
	// (UPROPERTY allows you to edit the variable in the Unreal Editor)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fossen Model|Mass")
	float Mass = 55.0f; // Mass of the USV in kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fossen Model|Mass")
	FVector InertiaTensor = FVector(12.4643f, 18.15f, 15.95f); // I_xx, I_yy, I_zz in kg*m^2

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fossen Model|Added Mass")
	FVector AddedMassLinear = FVector(5.2815f, 82.5f, 55.0f); // X_u_dot, Y_v_dot, N_r_dot in kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fossen Model|Added Mass")
	FVector AddedMassAngular = FVector(2.4929f, 14.52f, 27.115f); // K_p_dot, M_q_dot, N_r_dot in kg*m^

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fossen Model|Damping")
	FVector LinearDampingTensor = FVector(77.0f, 137.0f, 546.0f); // X_u, Y_v, N_r in kg/s

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fossen Model|Damping")
	FVector AngularDampingTensor = FVector(54.0f, 246.0f, 46.0f); // K_p, M_q, N_r in kg*m^2/s 

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fossen Model|Thrusters")
	float ThrusterDistance = 0.395f; // Distance between left and right thrusters in meters
	
	//Function to calculate non-linear thrust based on Amp Current
	float GetT200Thrust(float CurrentAmps);


private:
	float ForwardInput = 0.0f; // Forward/backward input value
	float RightInput = 0.0f; // Right/left input value

	void MoveForward(float Value); // Function to handle forward/backward movement input
	void TurnRight(float Value); // Function to handle right/left movement input


public:	
	//VIRTUAL SENSORS and NOISE SIMULATION:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Config")
	float SensorFailureProbability = 0.05f; //5% chanche of intermittent failure

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Config")
	float GPSNoiseVariance = 0.00001f; // Noise added to GPS readings (in degrees) (Lat/Lon)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Config")
	float IMUNoiseVariance = 0.5f; //Noise added to accelerometer and gyroscope readings

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LidarRange = 5000.0f; // Maximum range of the LiDAR sensor in cm

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	int32 LidarRays = 36; //number of rays per 360 sweep (10-degree resolution)

	bool bSensorsOnline = true; // Whether the sensor is currently online (not failed)
	float LatencyTimer = 0.0f; // Timer to track sensor latency

	//Sensor Functions:
	void SimulateSensors(float DeltaTime); // Function to simulate sensor readings, noise and failure

	// Helper function to generate Gaussian noise with specified variance
	float GenerateNoise(float Variance); 

	//UDP ROS2 BRIDGE
	FSocket* UDPSocket;
	void SendUDPMessage(FString Message); // Function to send a message via UDP to the ROS2 bridge
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override; // Override to clean up socket on end play


	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

};
