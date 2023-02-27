// Copyright (c) 2022, Li Huang. All rights reserved.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RigidBunnyShapeMatchingComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class LAB1_API URigidBunnyShapeMatchingComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	URigidBunnyShapeMatchingComponent();

	UFUNCTION(BlueprintCallable)
  void LaunchBunny();

  UFUNCTION(BlueprintCallable)
  void ResetBunny();

	// Collide with a plane
  void Collision(const FVector& P, const FVector& N);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void Update_Mesh(const FVector& c, const FMatrix& R, float inv_dt);
	void Collision(float inv_dt);

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Rigid Bunny Shape Matching")
	bool launched = false;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny Shape Matching")
  float dt = 0.001f;
	// for velocity decay
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float linear_decay = 0.999f;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float mass_per_vertex = 1.f;
	// UE adopts cm as length unit, so gravity should be -981 cm/^s2 here.
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  FVector gravity = FVector(0.f, 0.f, -981.f);

	// positions of all vertices in global space
	TArray<FVector> X;
  // centerized positions of all vertices in global space
  TArray<FVector> Q;
	// velocities of all vertices
  TArray<FVector> V;
	TArray<FVector> Y;
  FMatrix QQt;

	// Used to recover UStaticMesh when ending PIE mode
	TArray<FVector> original_pos;
	FTransform original_transform;
	TArray<FVector> reset_X;

	UStaticMeshComponent* static_mesh_component_;
};
