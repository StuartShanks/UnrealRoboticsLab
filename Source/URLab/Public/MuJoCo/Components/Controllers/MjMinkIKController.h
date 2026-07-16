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
#include "Templates/PimplPtr.h"
#include "HAL/ThreadSafeCounter.h"
#include "MuJoCo/Components/Controllers/MjArticulationController.h"
#include "MjMinkIKController.generated.h"

// URLabMink is a private dependency of this module, so no Mink header may appear
// here — all Mink-typed state lives behind a TPimplPtr whose type-erased deleter
// keeps UHT-generated code compiling against the incomplete impl type.
class UMjComponent;
class UMjJoint;
class UMjBody;
class UMjTwistController;

/** Which mink task a spec entry builds. Mirrors the ported task catalogue. */
UENUM(BlueprintType)
enum class EMinkTaskKind : uint8
{
	/** Drive a frame (site/body/geom) to a Cartesian target pose. */
	Frame,
	/** Regularize joints toward a reference posture (redundancy resolution). */
	Posture,
	/** Penalize joint velocity — heavy cost on a subset freezes it (e.g. fix-base). */
	Damping,
	/**
	 * Consume the sibling UMjTwistController bus (the same signal WASD, set_twist,
	 * and the nav stack's pure pursuit produce): each physics step the robot-frame
	 * twist is rotated by the base yaw from the IK reference and integrated into a
	 * base pose target with UMjBaseDriveController's leash/reseed semantics
	 * (shared MjBaseIntegrate helpers), then tracked as a posture task over the
	 * base DOFs. Joints must list EXACTLY the 3 base joints IN ORDER: x, y, th.
	 * Makes the mink a drop-in whole-body consumer of navigation intent — no
	 * controller swap, no target streaming. Pair with a base velocity limit and
	 * max_iters=1 so the QP paces the base at true wall-clock speed.
	 */
	TwistFollow
};

/** Which mink limit a spec entry builds. */
UENUM(BlueprintType)
enum class EMinkLimitKind : uint8
{
	/** Joint range limits (mink ConfigurationLimit). */
	Configuration,
	/** Hard cap on solved joint velocities (mink VelocityLimit). */
	Velocity,
	/**
	 * Velocity-level obstacle clearance (mink CollisionAvoidanceLimit): an
	 * inequality on the normal velocity between geom pairs — geoms may not
	 * approach each other faster than the limit allows, and stop at
	 * MinDistance. Pairs = GeomsA x GeomsB (one pair of groups per spec entry;
	 * add more entries for more pairs). This is the layer the navmesh cannot
	 * give (the arm envelope, reach-over-obstacle); it is LOCAL and GREEDY —
	 * it prevents penetration, it does not plan around obstacles.
	 */
	CollisionAvoidance
};

/**
 * One task in the IK stack, described as data. The controller builds the actual
 * URLabMink task from this at Bind() — swap robots by repointing the component
 * references, no C++ changes.
 */
USTRUCT(BlueprintType)
struct FMinkTaskSpec
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task")
	EMinkTaskKind Kind = EMinkTaskKind::Frame;

	/** Enable/disable this task live (e.g. toggle a fix-base Damping task). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task")
	bool bEnabled = true;

	/**
	 * Frame only: the driven frame — a UMjSite, UMjBody, or UMjGeom on this
	 * articulation. The mink frame type is inferred from the component class.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task",
		meta = (EditCondition = "Kind==EMinkTaskKind::Frame", UseComponentPicker))
	TObjectPtr<UMjComponent> Frame;

	/**
	 * Frame only: MjName of the driven frame (site/body/geom) — the
	 * duplication-safe fallback for Frame. A PIE/Simulate world copy nulls the
	 * TObjectPtr above; this plain string survives and is resolved by name at
	 * Bind. The component ref wins when valid; Bind self-captures the compiled
	 * name here after the first successful resolve.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task",
		meta = (EditCondition = "Kind==EMinkTaskKind::Frame"))
	FString FrameName;

	/**
	 * Frame only: mocap body whose live pose is the target (e.g. a
	 * "pinch_site_target" body). None => target comes from SetIKTarget().
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task",
		meta = (EditCondition = "Kind==EMinkTaskKind::Frame", UseComponentPicker))
	TObjectPtr<UMjBody> TargetMocapBody;

	/**
	 * Frame only: MjName of the mocap target body — duplication-safe fallback
	 * for TargetMocapBody (survives a PIE/Simulate world copy). Ref wins when
	 * valid; Bind self-captures the compiled name here after a good resolve.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task",
		meta = (EditCondition = "Kind==EMinkTaskKind::Frame"))
	FString TargetMocapBodyName;

	/** Frame only: position error weight. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task|Gains",
		meta = (EditCondition = "Kind==EMinkTaskKind::Frame", ClampMin = "0.0", UIMin = "0.0", UIMax = "10.0"))
	float PositionCost = 1.0f;

	/** Frame only: orientation error weight (0 = position-only IK). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task|Gains",
		meta = (EditCondition = "Kind==EMinkTaskKind::Frame", ClampMin = "0.0", UIMin = "0.0", UIMax = "10.0"))
	float OrientationCost = 1.0f;

	/** Posture/Damping: scalar cost applied to the DOFs selected below. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task|Gains",
		meta = (EditCondition = "Kind!=EMinkTaskKind::Frame", ClampMin = "0.0", UIMin = "0.0", UIMax = "1000.0"))
	float Cost = 0.01f;

	/** Task gain in [0,1] — fraction of the error corrected per step. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task|Gains",
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float Gain = 1.0f;

	/** Levenberg-Marquardt damping (stabilizes near singularities). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task|Gains",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "5.0"))
	float LmDamping = 0.0f;

	/**
	 * Posture/Damping: joints the scalar cost applies to; other DOFs get 0.
	 * Empty => all DOFs. TidyBot's posture-off-base / damp-base-only both fall
	 * out of picking the base joints here.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task",
		meta = (EditCondition = "Kind!=EMinkTaskKind::Frame", UseComponentPicker))
	TArray<TObjectPtr<UMjJoint>> Joints;

	/**
	 * Posture/Damping: MjNames of the joints above — duplication-safe fallback
	 * for Joints (survives a PIE/Simulate world copy). Refs win when valid; Bind
	 * self-captures the compiled joint names here after a good resolve.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Task",
		meta = (EditCondition = "Kind!=EMinkTaskKind::Frame"))
	TArray<FString> JointNames;
};

/** One limit in the IK stack. Empty Limits array => mink's default joint-range limit. */
USTRUCT(BlueprintType)
struct FMinkLimitSpec
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit")
	EMinkLimitKind Kind = EMinkLimitKind::Configuration;

	/** Fraction of the max joint-range step allowed per timestep, in (0, 1]. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float Gain = 0.95f;

	/** Keep joints at least this far (rad/m) inside their range. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit", meta = (ClampMin = "0.0"))
	float MinDistance = 0.0f;

	/** Velocity only: max joint speed (rad/s or m/s), applied per selected DOF. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (EditCondition = "Kind==EMinkLimitKind::Velocity", ClampMin = "0.0", UIMin = "0.0", UIMax = "10.0"))
	float MaxVelocity = 3.0f;

	/** Velocity only: joints to cap. Empty => every hinge/slide/ball joint. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (EditCondition = "Kind==EMinkLimitKind::Velocity"))
	TArray<TObjectPtr<UMjJoint>> Joints;

	/**
	 * Velocity only: MjNames of the capped joints above — duplication-safe
	 * fallback for Joints (survives a PIE/Simulate world copy). Refs win when
	 * valid; Bind self-captures the compiled joint names here after a good
	 * resolve.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (EditCondition = "Kind==EMinkLimitKind::Velocity"))
	TArray<FString> JointNames;

	/**
	 * CollisionAvoidance only: side A of the pair (typically robot geoms).
	 * Each entry resolves as a GEOM name first, else a BODY name that expands
	 * to all of that body's geoms — required in practice, since imported mesh
	 * geoms are commonly unnamed. Suffix-tolerant against import prefixes.
	 * For this kind, Gain is the avoidance gain (mink default 0.85) and
	 * MinDistance the standoff to hold (mink default 0.005 m) — the
	 * add_controller op applies those defaults when the fields are omitted.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (EditCondition = "Kind==EMinkLimitKind::CollisionAvoidance"))
	TArray<FString> GeomsA;

	/** CollisionAvoidance only: side B of the pair (typically obstacle geoms).
	 *  Same geom-or-body resolution as GeomsA. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (EditCondition = "Kind==EMinkLimitKind::CollisionAvoidance"))
	TArray<FString> GeomsB;

	/** CollisionAvoidance only: distance (m) at which pairs enter the QP
	 *  (mink collision_detection_distance). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (EditCondition = "Kind==EMinkLimitKind::CollisionAvoidance", ClampMin = "0.0"))
	float DetectionDistance = 0.01f;

	/** CollisionAvoidance only: bound relaxation offset (mink bound_relaxation). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Limit",
		meta = (EditCondition = "Kind==EMinkLimitKind::CollisionAvoidance"))
	float BoundRelaxation = 0.0f;
};

/**
 * @class UMjMinkIKController
 * @brief Data-driven differential-IK controller built on URLabMink.
 *
 * Add to an AMjArticulation and compose the task stack in the Details panel:
 * typically one Frame task on the end-effector site, a low-cost Posture task,
 * and optionally a heavy Damping task on the base joints (fix-base). Every
 * physics step it solves the stack (MinkSolveIK), integrates an internal
 * reference configuration, and writes the solved joint positions to the bound
 * actuators' d->ctrl — the actuators drive the robot; qpos is never written.
 *
 * One class covers arms, mobile manipulators, and hands: swapping a similar
 * embodiment within the same task means repointing the component references
 * (frame, joints) and nudging sliders — no subclassing. Target sources are
 * external by design (a mocap body, SetIKTarget from Blueprint/VR, or the
 * bridge); the controller only consumes targets.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Mink IK Controller"))
class URLAB_API UMjMinkIKController : public UMjArticulationController
{
	GENERATED_BODY()

public:
	UMjMinkIKController();
	virtual ~UMjMinkIKController() override;

	/** The IK task stack, solved together each step in array order. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK")
	TArray<FMinkTaskSpec> Tasks;

	/** Inequality limits. Empty => mink's default joint-range limit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK")
	TArray<FMinkLimitSpec> Limits;

	/**
	 * Actuated joints the solver commands (their position actuators get the
	 * solved q). Empty => every actuator bound on this articulation.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK",
		meta = (UseComponentPicker))
	TArray<TObjectPtr<UMjJoint>> DriveJoints;

	/**
	 * MjNames of DriveJoints — duplication-safe fallback (survives a PIE/Simulate
	 * world copy that nulls the TObjectPtrs above). Refs win when valid; Bind
	 * self-captures the compiled joint names here after a good resolve.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK")
	TArray<FString> DriveJointNames;

	/** Inner solve/integrate iterations per physics step. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver",
		meta = (ClampMin = "1", UIMin = "1", UIMax = "50"))
	int32 MaxIters = 5;

	/** QP regularization damping. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver",
		meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "0.01"))
	float QpDamping = 1e-3f;

	/**
	 * 0 = integrate the IK reference with the elapsed sim-time delta (default).
	 * >0 = integrate with this fixed dt regardless of sim timestep, e.g. 0.005
	 * to match the mink example's 200 Hz rate.dt exactly (for trace-parity
	 * testing against the Python golden run).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver",
		meta = (ClampMin = "0.0", UIMax = "0.02"))
	float IntegrateDtOverride = 0.0f;

	/** Early-out thresholds on the first Frame task's error (m / rad). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver", meta = (ClampMin = "0.0"))
	float PosThreshold = 1e-4f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver", meta = (ClampMin = "0.0"))
	float OriThreshold = 1e-4f;

	/**
	 * Servo mode: re-sync the internal reference from the live qpos each step
	 * (tracks disturbances) instead of integrating open-loop like the mink
	 * examples. Open-loop gives the cleanest tracking under position control.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver")
	bool bSyncFromLiveState = false;

	/**
	 * Draw a debug marker (sphere + coordinate axes) at each enabled Frame
	 * task's current target pose, whatever the source — a mocap body, a
	 * streamed manual target (SetIKTarget / bridge target_pos), or the held
	 * bind pose. Useful during a live demo: a streamed target bypasses the
	 * mocap body's visible box, so without this the robot appears to chase
	 * nothing. Editor/dev builds only (compiles to a no-op in Shipping).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Debug")
	bool bDrawTarget = false;

	/** Debug marker radius, in centimeters (UE units). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Debug", meta = (ClampMin = "0.1"))
	float DrawTargetSize = 5.0f;

	/**
	 * Command the target of the Frame task at Tasks[TaskIndex] in Unreal world
	 * space. Thread-safe. Ignored while that spec has a TargetMocapBody. Wire
	 * VR controller poses in here.
	 */
	UFUNCTION(BlueprintCallable, Category = "Mink IK")
	void SetIKTarget(int32 TaskIndex, FVector WorldPos, FQuat WorldRot);

	/**
	 * Bump the spec generation so the running solver rebuilds its task stack from
	 * the current Tasks / Limits / DriveJoints on the next physics step. Call this
	 * after mutating the spec arrays on a live controller (e.g. add_controller
	 * reconfigure; configure_controller's "task_costs" field calls it for you).
	 * Task costs, damping, and joint subsets are baked into the solver when it
	 * builds, so they only take effect via this; per-task bEnabled, targets, and
	 * the solver sliders are read live and don't need it.
	 */
	void MarkSpecsChanged() { SpecGeneration.Increment(); }

	// --- UMjArticulationController ---
	virtual void Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& ActuatorIdMap) override;
	virtual void ComputeAndApply(mjModel* m, mjData* d, uint8 Source) override;

	// --- UActorComponent ---
	/** Game-thread only: draws the bDrawTarget debug markers from the physics-
	 *  thread snapshot written by ComputeAndApply. Cheap no-op when bDrawTarget
	 *  is false (the snapshot write itself is also gated on bDrawTarget). */
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	// --- bridge config surface (configure_controller) ---
	virtual FString GetKindName() const override { return TEXT("mink_ik"); }
	virtual void GetConfigSchema(TSharedPtr<FJsonObject>& OutSchema) const override;

protected:
	// Called with the base class's ConfigMutex held; ComputeAndApply snapshots
	// everything these write under the same mutex before solving.
	virtual void GetCurrentConfigInternal(TSharedPtr<FJsonObject>& OutParams) const override;
	virtual void ApplyConfigInternal(const TSharedPtr<FJsonObject>& InParams) override;

private:
	/** Manual (SetIKTarget) target for a Frame spec, MuJoCo world coords. */
	struct FManualTarget
	{
		double Pos[3] = {0.0, 0.0, 0.0};
		double Quat[4] = {1.0, 0.0, 0.0, 0.0};
		bool bSet = false;
	};

	/** Latched manual joint-space posture target (the "posture_target" wire
	 *  param). JointQ: MuJoCo joint name -> qpos value. Re-applied to the
	 *  routed posture spec EVERY solve in reference space — unmentioned
	 *  joints carry the current reference q (TwistFollow's pattern).
	 *  bSet=false means no latch. */
	struct FManualPostureTarget
	{
		TMap<FString, double> JointQ;
		int32 SpecIndex = INDEX_NONE; // INDEX_NONE = first non-TwistFollow Posture spec
		bool bSet = false;
	};

	/** All Mink-typed solver state (configuration, built tasks/limits) — defined
	 *  in the .cpp so this public header stays free of URLabMink includes. */
	struct FMinkIKState;
	TPimplPtr<FMinkIKState> Mink;

	/** Rebuilds BuiltTasks / BuiltLimits / driven actuators from the current specs
	 *  against the live model. Called by Bind, and — on a spec-generation change —
	 *  from ComputeAndApply on the physics thread, so all solver-state mutation
	 *  stays on one thread (no lock needed vs ApplyControls). */
	void RebuildFromSpecs(mjModel* m, mjData* d);

	/** ctrl index / qpos address per driven actuator, resolved at Bind. */
	TArray<int32> DriveCtrlIds;
	TArray<int32> DriveQposAddrs;

	/** Every actuator on the articulation, keyed by MuJoCo actuator id — captured
	 *  verbatim from Bind()'s ActuatorIdMap, UNFILTERED by transmission type.
	 *  The inherited Bindings array (MjArticulationController) only covers
	 *  joint-transmission actuators (Bind() skips tendon/site/etc. transmissions
	 *  with a warning), so tendon-driven actuators like a 2f85 gripper's
	 *  "fingers_actuator" never appear there. ComputeAndApply's non-drive
	 *  pass-through needs the full roster to honor the BaseDrive contract for
	 *  every actuator, not just the joint-driven subset. */
	UPROPERTY()
	TMap<int32, TObjectPtr<UMjActuator>> AllActuatorIdMap;

	/** Sibling twist source for TwistFollow tasks; resolved in Bind, read on
	 *  the physics thread via its thread-safe GetTwist() (BaseDrive pattern).
	 *  Null when the actor has no twist controller — TwistFollow tasks then
	 *  hold their seed pose. */
	UPROPERTY()
	TObjectPtr<UMjTwistController> TwistSource = nullptr;

	/** Spec revision, incremented on the game thread by MarkSpecsChanged() and
	 *  compared on the physics thread to trigger a live rebuild. */
	FThreadSafeCounter SpecGeneration;

	/** Generation the live BuiltTasks were built from (physics thread only). */
	int32 BuiltGeneration = -1;

	/** Manual targets keyed by spec index; guarded by TargetMutex. */
	TMap<int32, FManualTarget> ManualTargets;
	mutable FCriticalSection TargetMutex;

	FManualPostureTarget ManualPosture; // guarded by TargetMutex
	/** Re-armed to 8 on each posture_target apply; decremented per unknown-name
	 *  warning on the physics thread. Benign race, log-budget only. */
	int32 PostureNameWarnBudget = 0;

	/** Physics-thread snapshot of one enabled Frame task's current target
	 *  pose, MuJoCo world coords (whatever the source: mocap, manual, or held
	 *  bind pose) — written by ComputeAndApply, read by TickComponent on the
	 *  game thread. Never draw from ComputeAndApply itself; it runs off the
	 *  game thread and DrawDebug* is not safe to call from there. */
	struct FMinkTargetSnapshot
	{
		double Pos[3] = {0.0, 0.0, 0.0};
		double Quat[4] = {1.0, 0.0, 0.0, 0.0};
	};

	/** Debug-draw snapshot; own lock (deliberately separate from TargetMutex —
	 *  different producer/consumer, no reason to contend). Only written when
	 *  bDrawTarget is set, so it costs nothing when the feature is off. */
	TArray<FMinkTargetSnapshot> DebugTargets;
	FCriticalSection DebugTargetMutex;

	/** Diagnostic call counter (first calls + every Nth are logged). */
	int32 DiagCounter = 0;

	/** Solves discarded by the absurd-velocity sanity clamp. */
	int32 BadSolveCount = 0;

	/** Remaining budget of "[MinkIK] solve failed" warnings this run. Was a
	 *  function-static in ComputeAndApply — shared across every instance and
	 *  never reset; now per-instance and reset in Bind() so each bound
	 *  controller gets a fresh budget. */
	int32 ErrorLogBudget = 8;

	/** Last mjData.time we integrated at — the engine calls ComputeAndApply on
	 *  idle physics-thread iterations too, and integrating the open-loop
	 *  reference on those races it ahead of the sim. */
	double LastSimTime = -1.0;

	/** GetSimResetEpoch() value at the last integration (seeded at Bind). A
	 *  change means the sim was reset/restored since we last solved, so the
	 *  open-loop reference is stale and must be re-based on the live state. */
	uint64 LastSeenResetEpoch = 0;
};
