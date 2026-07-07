// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include <mujoco/mujoco.h>
#include "MjEndEffectorController.generated.h"

class FMujocoSpecWrapper;

/**
 * @class UMjEndEffectorController
 * @brief Drives an articulation's end-effector to a target Cartesian pose using
 *        MuJoCo's mocap + weld "fake IK" pattern — the single point of entry for
 *        VR teleoperation.
 *
 * Drop this component on any AMjArticulation and set TargetBodyName to the body
 * you want to drive (e.g. the gripper / tool flange). At spec-build time the
 * component injects:
 *   - a free-floating `mocap` body placed coincident with the target body, and
 *   - a `weld` equality constraint tying the mocap body to the target body.
 *
 * The MuJoCo constraint solver then drags the end-effector toward wherever the
 * mocap body is. You command it with a single call:
 *
 *     Controller->SetEndEffectorTarget(WorldPos, WorldRot);
 *
 * which submits a global pose (xyz + wxyz, expressed in Unreal world space) to
 * the mocap body via the engine's thread-safe command queue. VR is wired in
 * later by calling this every frame with the tracked controller pose.
 *
 * NOTE: this is NOT a UMjArticulationController. That base class binds actuators
 * and writes d->ctrl (joint-space control). This controller never touches ctrl —
 * it is constraint-driven and writes d->mocap_pos / d->mocap_quat. The two are
 * complementary mechanisms.
 *
 * CAVEAT (actuators): the weld supplies the motion forces. If the arm has stiff
 * position actuators holding a fixed qpos, they fight the weld. For pure fake-IK
 * teleop, use low/zero actuation on the driven chain (gravity-comp is fine).
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo End-Effector Controller"))
class URLAB_API UMjEndEffectorController : public USceneComponent
{
	GENERATED_BODY()

public:
	UMjEndEffectorController();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Name of the end-effector UMjBody to drive (its MjName, or component name if MjName is empty). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop", meta = (GetOptions = "GetBodyOptions"))
	FString TargetBodyName;

	/** If false, the mocap + weld are still injected but no spec is added (controller is inert). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop")
	bool bEnabled = true;

	/**
	 * Neutralize the articulation's actuators so the weld can move the arm freely
	 * (the classic mocap fake-IK setup). Applied at compile by clamping every
	 * actuator's output force to zero — this kills force for any actuator type,
	 * unlike zeroing ctrl which leaves position actuators holding their target.
	 *
	 * NOTE: this disables ALL actuators on the articulation, including a gripper.
	 * Turn it off if you need the gripper (or other) actuators live, and instead
	 * slacken just the arm's actuators yourself.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop")
	bool bDisableActuators = true;

	/**
	 * Weld solver reference [timeconst, dampratio]. Smaller timeconst = stiffer /
	 * tighter tracking. Default is stiffer than MuJoCo's so teleop feels rigid.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop|Weld")
	FVector2D Solref = FVector2D(0.01, 1.0);

	/** Weld solver impedance [dmin, dmax, width, midpoint, power]. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop|Weld")
	TArray<float> Solimp = {0.9f, 0.95f, 0.001f, 0.5f, 2.0f};

	/**
	 * Relative weight of orientation vs position error in the weld. 1.0 welds
	 * orientation fully; 0.0 makes it a point (position-only) constraint.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop|Weld", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TorqueScale = 1.0f;

	// =========================================================================
	// Live drive — for playing with the target by hand (no VR yet).
	//
	// Turn on bDriveTarget and the component pushes a target every tick. Either
	// drag this component's transform gizmo (bUseGizmo, best in Simulate-In-Editor)
	// or type into TargetPosition / TargetRotation in the Details panel during play.
	// =========================================================================

	/** When true, push the target to MuJoCo every tick from the source below. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop|Live")
	bool bDriveTarget = false;

	/**
	 * Drive from this component's own world transform — grab its gizmo and move
	 * it (use Simulate-In-Editor so you can drag components while physics runs).
	 * When false (default), the TargetPosition / TargetRotation fields below drive
	 * the end-effector, so you can just type values in the Details panel.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop|Live", meta = (EditCondition = "bDriveTarget"))
	bool bUseGizmo = false;

	/** Target position in Unreal world space (cm). Used when bUseGizmo is false. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop|Live", meta = (EditCondition = "bDriveTarget && !bUseGizmo"))
	FVector TargetPosition = FVector::ZeroVector;

	/** Target orientation in Unreal world space. Used when bUseGizmo is false. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Teleop|Live", meta = (EditCondition = "bDriveTarget && !bUseGizmo"))
	FRotator TargetRotation = FRotator::ZeroRotator;

	// =========================================================================
	// Single point of entry — call this (later, from VR) to drive the EE.
	// =========================================================================

	/**
	 * Command the end-effector to a global pose. Thread-safe (queued and applied
	 * before the next physics step). Pose is in Unreal world space.
	 * @param WorldPos End-effector target position (Unreal world, cm).
	 * @param WorldRot End-effector target orientation (Unreal world).
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Teleop")
	void SetEndEffectorTarget(FVector WorldPos, FQuat WorldRot);

	/**
	 * Re-center the target on the end-effector's current pose, so enabling teleop
	 * (or recovering from a tracking jump) doesn't yank the arm.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Teleop")
	void SnapTargetToEndEffector();

	/** True once the synthetic mocap body has been resolved in the compiled model. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Teleop")
	bool IsReady() const { return MocapBodyId >= 0; }

	// =========================================================================
	// Called by AMjArticulation during its setup phases (not for direct use).
	// =========================================================================

	/** Pre-compile: inject the mocap body + weld equality into the child spec. */
	void InjectMocapAndWeld(FMujocoSpecWrapper& Wrapper);

	/** Post-compile: resolve the mocap body's id in the compiled model. */
	void ResolveAfterCompile(mjModel* Model, mjData* Data, const FString& Prefix);

#if WITH_EDITOR
	UFUNCTION()
	TArray<FString> GetBodyOptions() const;
#endif

private:
	/** Unprefixed name of the synthetic mocap body we create in the child spec. */
	FString MocapBodyName;

	/** Compiled body id of the mocap body (NOT the mocap index). -1 until resolved. */
	int32 MocapBodyId = -1;
};
