// Copyright (c) 2022, Li Huang. All rights reserved.
#include "RigidBunnyShapeMatchingComponent.h"

#include "MathUtil.h"

// Sets default values for this component's properties
URigidBunnyShapeMatchingComponent::URigidBunnyShapeMatchingComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void URigidBunnyShapeMatchingComponent::BeginPlay()
{
	Super::BeginPlay();

	static_mesh_component_ =
      GetOwner()->FindComponentByClass<UStaticMeshComponent>();
	if (!static_mesh_component_ || !static_mesh_component_->GetStaticMesh() ||
		  !static_mesh_component_->GetStaticMesh()->RenderData) {
		return;
	}

  static_mesh_component_->GetStaticMesh()->ExtendedBounds =
            FBoxSphereBounds(FBox(FVector(-1000.f, -1000.f, 0.f),
                                  FVector(1000.f, 1000.f, 800.f)));

	const FPositionVertexBuffer& vertex_buffer = static_mesh_component_->GetStaticMesh()
		  ->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer;

	V.SetNumZeroed(vertex_buffer.GetNumVertices());
	X.SetNumZeroed(vertex_buffer.GetNumVertices());
	Q.SetNumZeroed(vertex_buffer.GetNumVertices());
	Y.SetNumZeroed(vertex_buffer.GetNumVertices());
	original_pos.SetNumZeroed(vertex_buffer.GetNumVertices());
	FVector c = FVector::ZeroVector;
	for (size_t i = 0; i < vertex_buffer.GetNumVertices(); i++) {
		X[i] = vertex_buffer.VertexPosition(i);
		Q[i] = vertex_buffer.VertexPosition(i);
		c += Q[i];

		original_pos[i] = vertex_buffer.VertexPosition(i);
	}

	// Centerizing Q.
	c /= Q.Num();
	for (size_t i = 0; i < Q.Num(); i++) {
		Q[i] -= c;
	}
	UE_LOG(LogTemp, Log, TEXT("c = {%f, %f, %f}"), c.X, c.Y, c.Z);
	// Get QQ^t ready.
	for (size_t i = 0; i < Q.Num(); i++) {
    QQt.M[0][0] += Q[i][0] * Q[i][0];
		QQt.M[0][1] += Q[i][0] * Q[i][1];
    QQt.M[0][2] += Q[i][0] * Q[i][2];
		QQt.M[1][0] += Q[i][1] * Q[i][0];
    QQt.M[1][1] += Q[i][1] * Q[i][1];
		QQt.M[1][2] += Q[i][1] * Q[i][2];
    QQt.M[2][0] += Q[i][2] * Q[i][0];
		QQt.M[2][1] += Q[i][2] * Q[i][1];
		QQt.M[2][2] += Q[i][2] * Q[i][2];
	}
	QQt.M[3][3] = 1.f;

  reset_X = X;
  original_transform = GetOwner()->GetActorTransform();

	FMatrix rotation_matrix =
      FRotationMatrix::Make(GetOwner()->GetActorRotation().Quaternion());
	Update_Mesh(GetOwner()->GetActorLocation(), rotation_matrix.Inverse(), 0.f);
	GetOwner()->SetActorTransform(FTransform::Identity);
}


// Called every frame
void URigidBunnyShapeMatchingComponent::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) {
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!static_mesh_component_ || !launched) {
    return;
	}

	// Step 1: run a simple particle system.
  for (size_t i = 0; i < V.Num(); i++) {
		V[i] += dt * gravity;
    V[i] *= linear_decay;
    Y[i] = X[i] + V[i] * dt;
  }

	// Step 2: Perform simple particle collision.
  Collision(1.f / dt);

	// Step 3: Use shape matching to get new translation c and
  // new rotation R. Update the mesh by c and R.
  // Shape Matching (translation)
  FVector c = FVector::ZeroVector;
  // Sigma_i((y_i - c) * r_i^T)
	FMatrix m1(EForceInit::ForceInitToZero);
  // Sigma_i(r_i * r_i^T)
  FMatrix m2 = QQt;

	for (size_t i = 0; i < Y.Num(); i++) {
    c += Y[i];
	}
  c /= Y.Num();

  // Shape Matching (rotation)
	for (size_t i = 0; i < Y.Num(); i++) {
    m1 += VectorOuterProduct(Y[i] - c, Q[i]);
	}
	FMatrix A = m1 * m2.Inverse();
	FMatrix R = Get_Rotation(A);

  Update_Mesh(c, R, 1.f / dt);
}

void URigidBunnyShapeMatchingComponent::Update_Mesh(const FVector& c,
                                                    const FMatrix& R,
                                                    float inv_dt) {
  if (!static_mesh_component_ || !static_mesh_component_->GetStaticMesh() ||
      !static_mesh_component_->GetStaticMesh()->RenderData) {
    return;
  }

	FPositionVertexBuffer& vertex_buffer = static_mesh_component_->GetStaticMesh()
		  ->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer;

  for (size_t i = 0; i < Q.Num(); i++) {
    FVector x = MatrixVectorMultiplication(R, Q[i]) + c;

    V[i] = (x - X[i]) * inv_dt;
    X[i] = x;
  }

  // Below code is to write vertex positions directly to GPU render buffer
  ENQUEUE_RENDER_COMMAND(FUpdatePositions)
  ([&vertex_buffer, this](FRHICommandListImmediate& RHICmdList) {
    // Get GPU pointer of VertexBuffer
    void* VertexBufferData = RHILockVertexBuffer(
        vertex_buffer.VertexBufferRHI, 0,
        vertex_buffer.GetNumVertices() * vertex_buffer.GetStride(),
        RLM_WriteOnly);
    // Copy data from CPU to GPU
    FMemory::Memcpy(VertexBufferData, X.GetData(), X.Num() * X.GetTypeSize());
    RHIUnlockVertexBuffer(vertex_buffer.VertexBufferRHI);
    static_mesh_component_->MarkRenderDynamicDataDirty();
  });
}

void URigidBunnyShapeMatchingComponent::Collision(float inv_dt) {
  // floor
  Collision(FVector(0.f, 0.f, 130.277084f), FVector(0.f, 0.f, 1.f));
  // side wall
  Collision(FVector(1050.f, 0.f, 530.f), FVector(-1.f, 0.f, 0.f));
}

void URigidBunnyShapeMatchingComponent::LaunchBunny() {
  if (!static_mesh_component_ || launched) {
    return;
  }
  for (size_t i = 0; i < X.Num(); i++) {
    V[i] = FVector(500.f, 200.f, 0.f);
  }
  launched = true;
}
void URigidBunnyShapeMatchingComponent::ResetBunny() {
  if (!static_mesh_component_) {
    return;
  }

  for (size_t i = 0; i < X.Num(); i++) {
    V[i] = FVector::ZeroVector;
  }
  X = reset_X;
  FMatrix rotation_matrix =
      FRotationMatrix::Make(original_transform.GetRotation());
  Update_Mesh(original_transform.GetLocation(), rotation_matrix.Inverse(), 0.f);
  launched = false;
}

void URigidBunnyShapeMatchingComponent::Collision(const FVector& P,
                                                  const FVector& N) {
  for (size_t i = 0; i < Y.Num(); i++) {
    float phi = ComputePhiPlane(Y[i], P, N);
    if (phi >= 0.f) {
      continue;
    }
    if (FVector::DotProduct(V[i], N) >= 0.f) {
      continue;
    }
		// Push colliding vertex back to surface of sdf
    float d = -phi;
		Y[i] = Y[i] + d * N;
  }
}

void URigidBunnyShapeMatchingComponent::EndPlay(
    const EEndPlayReason::Type EndPlayReason) {
  Super::EndPlay(EndPlayReason);
	if (!static_mesh_component_ || !static_mesh_component_->GetStaticMesh() ||
      !static_mesh_component_->GetStaticMesh()->RenderData) {
    return;
  }

	FPositionVertexBuffer& vertex_buffer = static_mesh_component_->GetStaticMesh()
		  ->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer;

  // Below code is to write vertex positions directly to GPU render buffer
	ENQUEUE_RENDER_COMMAND(FRestoreOriginalPositions)
    ([&vertex_buffer, this](FRHICommandListImmediate& RHICmdList) {
      // Get GPU pointer of VertexBuffer
			void* VertexBufferData = RHILockVertexBuffer(
					vertex_buffer.VertexBufferRHI, 0,
					vertex_buffer.GetNumVertices() * vertex_buffer.GetStride(),
					RLM_WriteOnly);
			// Copy data from CPU to GPU
			FMemory::Memcpy(VertexBufferData, original_pos.GetData(),
											original_pos.Num() * original_pos.GetTypeSize());
			RHIUnlockVertexBuffer(vertex_buffer.VertexBufferRHI);
			static_mesh_component_->MarkRenderDynamicDataDirty();
    });

  GetOwner()->SetActorTransform(original_transform);
  static_mesh_component_->GetStaticMesh()->ExtendedBounds =
      FBoxSphereBounds(ForceInitToZero);
}