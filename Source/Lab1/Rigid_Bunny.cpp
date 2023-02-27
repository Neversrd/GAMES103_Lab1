// Copyright (c) 2022, Li Huang. All rights reserved.

#include "Rigid_Bunny.h"
#include <vector>
#include "MathUtil.h"

// Sets default values for this component's properties
URigid_Bunny::URigid_Bunny() {
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void URigid_Bunny::BeginPlay() {
	Super::BeginPlay();

	v = FVector::ZeroVector;
  w = FVector::ZeroVector;
  I_ref = FMatrix(EForceInit::ForceInitToZero);

	static_mesh_component_ =
      GetOwner()->FindComponentByClass<UStaticMeshComponent>();
	if (!static_mesh_component_ || !static_mesh_component_->GetStaticMesh() ||
		  !static_mesh_component_->GetStaticMesh()->RenderData) {
		return;
	}

	const FPositionVertexBuffer& vertex_buffer =
      static_mesh_component_->GetStaticMesh()->RenderData->LODResources[0]
          .VertexBuffers.PositionVertexBuffer;

	float m = mass_per_vertex;
	mass = 0.f;

	for (size_t i = 0; i < vertex_buffer.GetNumVertices(); i++) {
		mass += m;
		FVector cur_pos = vertex_buffer.VertexPosition(i);
		float diag = m * cur_pos.SizeSquared();

		I_ref.M[0][0] += diag;
		I_ref.M[1][1] += diag;
		I_ref.M[2][2] += diag;
		I_ref.M[0][0] -= m * cur_pos[0] * cur_pos[0];
		I_ref.M[0][1] -= m * cur_pos[0] * cur_pos[1];
		I_ref.M[0][2] -= m * cur_pos[0] * cur_pos[2];
		I_ref.M[1][0] -= m * cur_pos[1] * cur_pos[0];
		I_ref.M[1][1] -= m * cur_pos[1] * cur_pos[1];
		I_ref.M[1][2] -= m * cur_pos[1] * cur_pos[2];
		I_ref.M[2][0] -= m * cur_pos[2] * cur_pos[0];
		I_ref.M[2][1] -= m * cur_pos[2] * cur_pos[1];
		I_ref.M[2][2] -= m * cur_pos[2] * cur_pos[2];
	}
	I_ref.M[3][3] = 1.f;
}

void URigid_Bunny::Collision_Impulse(const FVector& P, const FVector& N) {
  if (!static_mesh_component_ || !static_mesh_component_->GetStaticMesh() ||
      !static_mesh_component_->GetStaticMesh()->RenderData || !launched) {
    return;
  }
	const FPositionVertexBuffer& vertex_buffer = static_mesh_component_->GetStaticMesh()
		  ->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer;
  FMatrix rotation_matrix =
      FRotationMatrix::Make(GetOwner()->GetActorRotation().Quaternion());

	// 1. collision detection
	TArray<uint32> colliding_indices;
	FVector avg_pos_local = FVector::ZeroVector;
  FVector avg_pos_global = FVector::ZeroVector;
  float min_z = 140;
	for (size_t i = 0; i < vertex_buffer.GetNumVertices(); i++) {
		// First, check if this vertex is inside the sdf we are testing
		FVector cur_pos_local = vertex_buffer.VertexPosition(i);
    FVector cur_pos_global =
        GetOwner()->GetActorTransform().TransformPosition(
            vertex_buffer.VertexPosition(i));
		float phi = ComputePhiPlane(cur_pos_global, P, N);
    if (phi >= 0.f) {
    	continue;
		}
		// Second, check if this vertex is escaping from the sdf
    FVector Rri = MatrixVectorMultiplication(
        rotation_matrix, vertex_buffer.VertexPosition(i));
    FVector velocity = v + FVector::CrossProduct(w, Rri);
		
		if (FVector::DotProduct(velocity, N) >= 0.f) {
      continue;
		}
    colliding_indices.Add(i);
		avg_pos_local += cur_pos_local;
		avg_pos_global += cur_pos_global;
    min_z = FMath::Min(min_z, cur_pos_global.Z);
	}

	if (colliding_indices.Num() == 0) {
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("min_z = %f"), min_z);
	UE_LOG(LogTemp, Log, TEXT("v = {%f, %f, %f}"), v.X, v.Y, v.Z);
	
	avg_pos_local /= colliding_indices.Num();
	avg_pos_global /= colliding_indices.Num();
  UE_LOG(LogTemp, Log, TEXT("Average position of %u colliding vertices is (%f, %f, "
						 			   			  "%f)(local), (%f, %f, %f)(global)"),
									   			  colliding_indices.Num(), avg_pos_local.X, avg_pos_local.Y,
									   			  avg_pos_local.Z, avg_pos_global.X, avg_pos_global.Y,
									   			  avg_pos_global.Z);

	// 2. collision response by impluse
	// Push colliding vertex back to surface of sdf
  float d = -ComputePhiPlane(avg_pos_global, P, N);
  GetOwner()->SetActorLocation((GetOwner()->GetActorLocation() + d * N));

	FVector avg_linear_impluse = FVector::ZeroVector;
	FVector avg_angular_impluse = FVector::ZeroVector;

	FMatrix I = rotation_matrix * I_ref * rotation_matrix.GetTransposed();
  FMatrix I_inv = I.Inverse();
  FVector Rri = MatrixVectorMultiplication(rotation_matrix, avg_pos_local);
  FVector velocity = v + FVector::CrossProduct(w, Rri);
  FVector velocity_normal = FVector::DotProduct(velocity, N) * N;
  FVector velocity_tangent = velocity - velocity_normal;
  // decrease mu_n according to velocity
  float mu_n = restitution * FMath::Min(1.f, velocity.Size() / 100.f);
  float a = FMath::Max(0.f,
      1 - mu_t * (1 + mu_n) * velocity_normal.Size() / velocity_tangent.Size());
  velocity_normal = -mu_n * velocity_normal;
  velocity_tangent = a * velocity_tangent;
  FVector velocity_new = velocity_normal + velocity_tangent;
  // Note that FMatrix is acctually a 4x4 matrix
  FMatrix Rri_star = ToSkewSymmetricMatrix(Rri);

  FMatrix K = MatrixSubstraction(
      FMatrix::Identity.ApplyScale(1.f / mass), Rri_star * I_inv * Rri_star);

  FMatrix K_inv = K.Inverse();
  FVector j = MatrixVectorMultiplication(K_inv, velocity_new - velocity);
  avg_linear_impluse = j;
  avg_angular_impluse = MatrixVectorMultiplication(Rri_star, j);

	v += avg_linear_impluse / mass;
	w += MatrixVectorMultiplication(I_inv, avg_angular_impluse);
}

// Called every frame
void URigid_Bunny::TickComponent(float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!static_mesh_component_ || !launched) {
    return;
	}

	// Part I: Update velocities
  v += dt * gravity;
	// Decay linear velocity and angular velocity
  v = linear_decay * v;
  w = angular_decay * w;

	// Part II: Collision Impulse
	// floor
  Collision_Impulse(FVector(0.f, 0.f, 130.277084f), FVector(0.f, 0.f, 1.f));
	// side wall
  Collision_Impulse(FVector(1050.f, 0.f, 530.f), FVector(-1.f, 0.f, 0.f));
	
	// Part III: Update position & orientation
  // Update linear status
  FVector x = GetOwner()->GetActorTransform().GetLocation();
  x += dt * v;
	// Update angular status
  FQuat q = GetOwner()->GetActorTransform().GetRotation();
  q += QuaternionCrossProduct(FQuat(w.X, w.Y, w.Z, 0.f) * dt / 2.f, q);
  q.Normalize();
	
	// Part IV: Assign new location and rotation to the object
  GetOwner()->SetActorLocationAndRotation(x, q);
}

void URigid_Bunny::LaunchBunny() {
  if (!static_mesh_component_ || launched) {
    return;
	}
	v = FVector(500.f, 200.f, 0.f);
  w = FVector(0.f, 0.f, 0.f);
  launched = true;
}

void URigid_Bunny::ResetBunny() {
	if (!static_mesh_component_) {
		return;
	}
  static_mesh_component_->SetWorldLocation(FVector(500.f, 370.f, 530.f));
  FRotator rotator(0.f, 180.f, 0.f);
  static_mesh_component_->SetWorldRotation(rotator);
	launched = false;
  v = FVector::ZeroVector;
  w = FVector::ZeroVector;
}