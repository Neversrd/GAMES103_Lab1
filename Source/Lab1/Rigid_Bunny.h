// Copyright (c) 2022, Li Huang. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Math/Matrix.h"
#include "Rigid_Bunny.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class LAB1_API URigid_Bunny : public UActorComponent {
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	URigid_Bunny();

  UFUNCTION(BlueprintCallable)
  void LaunchBunny();

  UFUNCTION(BlueprintCallable)
  void ResetBunny();

  void Collision_Impulse(const FVector& P, const FVector& N);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

  UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Rigid Bunny")
	bool launched = false;
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float dt = 0.001f;
  FVector v;  // velocity
  FVector w;  // angular velocity

  float mass;       // mass
  FMatrix I_ref;  // reference inertia

  // for velocity decay
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float linear_decay = 0.999f;
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float angular_decay = 0.98f;
  // mu_n, for collision
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float restitution = 0.5f;
  // for friction
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float mu_t = 0.5f;
  
  // UE adopts cm as length unit, so gravity should be -981 cm/^s2 here.
  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  FVector gravity = FVector(0.f, 0.f, -981.f);

  UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Rigid Bunny")
  float mass_per_vertex = 1.f;

  UStaticMeshComponent* static_mesh_component_;
};
