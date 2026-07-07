# Mobile TidyBot IK — C++ controller + setup

A faithful UE/C++ translation of `mink/examples/mobile_tidybot.py`. It's a
`UMjArticulationController` (so it binds actuators and writes `d->ctrl` — never
`qpos`), holds the mink task stack, and drives the whole robot (planar base +
7-DoF arm) to an end-effector target. Base can be locked with a damping task
(the example's Enter-key `fix_base` toggle).

Line-for-line correspondence with the example:

| `mobile_tidybot.py` | this controller |
|---|---|
| `configuration = mink.Configuration(model)` | `Config` (built in `Bind`) |
| `FrameTask("pinch_site", "site", 1, 1, lm=1)` | `EeTask` |
| `PostureTask(model, cost[3:]=1e-3)` | `Posture` (0 on base DOFs) |
| `DampingTask(model, cost[:3]=100)` | `BaseDamping` (added when `bFixBase`) |
| `ConfigurationLimit(model)` | `ConfigLimit` |
| `T_wt = SE3.from_mocap_name(...)` | `FMinkSE3::FromMocapId(d, MocapIndex)` |
| `for i in range(max_iters): solve_ik; integrate_inplace; break on err` | inner loop in `ComputeAndApply` |
| `data.ctrl[actuator_ids] = configuration.q[dof_ids]` | `d->ctrl[B.ActuatorMjID] = q[B.QposAddr]` |

---

## `MjMinkIKController.h`

```cpp
#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Components/Controllers/MjArticulationController.h"
#include "MjMinkIKController.generated.h"

// URLabMink types — forward-declared to keep Eigen out of this header.
class FMinkConfiguration;
class FMinkFrameTask;
class FMinkPostureTask;
class FMinkDampingTask;
class FMinkLimit;

/**
 * Differential-IK controller (URLabMink). Drop on an AMjArticulation; drives the
 * bound actuators so the end-effector frame tracks a target. Mobile base + arm.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Mink IK Controller"))
class URLAB_API UMjMinkIKController : public UMjArticulationController
{
	GENERATED_BODY()

public:
	UMjMinkIKController();
	virtual ~UMjMinkIKController();

	/** End-effector frame (site). As authored; import prefixes are resolved by suffix. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK")
	FString EndEffectorFrame = TEXT("pinch_site");

	/** Base joints — excluded from posture (so the base is free) and damped when bFixBase. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK")
	TArray<FString> BaseJointNames = {TEXT("joint_x"), TEXT("joint_y"), TEXT("joint_th")};

	/** Mocap body whose pose is the live target (tidybot: "pinch_site_target").
	 *  Leave empty to drive via SetIKTarget() instead. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK")
	FString MocapTargetBody = TEXT("pinch_site_target");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Gains")
	float PositionCost = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Gains")
	float OrientationCost = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Gains")
	float PostureCost = 1e-3f; // non-base DOFs
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Gains")
	float LmDamping = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Gains")
	float QpDamping = 1e-3f;

	/** Lock the mobile base (adds a heavy DampingTask on the base DOFs). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Base")
	bool bFixBase = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Base")
	float FixBaseDampingCost = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver")
	int32 MaxIters = 20;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver")
	float PosThreshold = 1e-4f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mink IK|Solver")
	float OriThreshold = 1e-4f;

	/** Command a target in Unreal world space (converted to MuJoCo). Thread-safe.
	 *  Ignored while MocapTargetBody is set. Wire this to VR controllers. */
	UFUNCTION(BlueprintCallable, Category = "Mink IK")
	void SetIKTarget(FVector WorldPos, FQuat WorldRot);

	// --- UMjArticulationController ---
	virtual void Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& ActuatorIdMap) override;
	virtual void ComputeAndApply(mjModel* m, mjData* d, uint8 Source) override;

	// --- config surface (bridge configure_controller / Blueprint) ---
	virtual FString GetKindName() const override { return TEXT("mink_ik"); }
	virtual void GetConfigSchema(TSharedPtr<FJsonObject>& OutSchema) const override;
	virtual void GetCurrentConfig(TSharedPtr<FJsonObject>& OutParams) const override;
	virtual void ApplyConfig(const TSharedPtr<FJsonObject>& InParams) override;

private:
	TUniquePtr<FMinkConfiguration> Config;
	TUniquePtr<FMinkFrameTask> EeTask;
	TUniquePtr<FMinkPostureTask> Posture;
	TUniquePtr<FMinkDampingTask> BaseDamping;
	TUniquePtr<FMinkLimit> ConfigLimit;

	int32 MocapIndex = -1; // m->body_mocapid[...] of MocapTargetBody, or -1
	double ManualTargetPos[3] = {0, 0, 0};
	double ManualTargetQuat[4] = {1, 0, 0, 0};
	bool bHasManualTarget = false;
	FCriticalSection TargetMutex;

	void BuildPostureAndDamping(const mjModel* m); // cost vectors from current props
};
```

## `MjMinkIKController.cpp`

```cpp
#include "MjMinkIKController.h"

#include "MinkConfiguration.h"
#include "MinkSolveIK.h"
#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkPostureTask.h"
#include "Tasks/MinkDampingTask.h"
#include "Limits/MinkConfigurationLimit.h"
#include "Lie/MinkSE3.h"
#include "Lie/MinkSO3.h"
#include "MuJoCo/Utils/MjUtils.h"

#include <mujoco/mujoco.h>

namespace
{
/** Resolve a name against the compiled model, tolerating URLab's import prefix
 *  (exact match, else a body/joint whose compiled name ends with "_name" / "/name"). */
int32 ResolveByName(const mjModel* m, int Type, const FString& Name)
{
	int32 Id = mj_name2id(m, Type, TCHAR_TO_UTF8(*Name));
	if (Id >= 0)
		return Id;
	const int32 N = (Type == mjOBJ_JOINT) ? m->njnt : (Type == mjOBJ_SITE ? m->nsite : m->nbody);
	const FString US = TEXT("_") + Name, SL = TEXT("/") + Name;
	for (int32 i = 0; i < N; ++i)
	{
		const char* Nm = mj_id2name(m, Type, i);
		if (!Nm)
			continue;
		const FString S = UTF8_TO_TCHAR(Nm);
		if (S == Name || S.EndsWith(US) || S.EndsWith(SL))
			return i;
	}
	return -1;
}
} // namespace

UMjMinkIKController::UMjMinkIKController() = default;
UMjMinkIKController::~UMjMinkIKController() = default;

void UMjMinkIKController::BuildPostureAndDamping(const mjModel* m)
{
	const int32 Nv = m->nv;

	// base DOF addresses (posture 0 there, damping heavy there)
	TSet<int32> BaseDofs;
	for (const FString& Jn : BaseJointNames)
	{
		const int32 Jid = ResolveByName(m, mjOBJ_JOINT, Jn);
		if (Jid >= 0)
			BaseDofs.Add(m->jnt_dofadr[Jid]);
	}

	FMinkVec PostureCostVec = FMinkVec::Constant(Nv, PostureCost);
	FMinkVec DampCostVec = FMinkVec::Zero(Nv);
	for (int32 A : BaseDofs)
	{
		if (A >= 0 && A < Nv)
		{
			PostureCostVec[A] = 0.0;
			DampCostVec[A] = FixBaseDampingCost;
		}
	}

	Posture = MakeUnique<FMinkPostureTask>(m, PostureCostVec);
	BaseDamping = MakeUnique<FMinkDampingTask>(m, DampCostVec);
}

void UMjMinkIKController::Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& Map)
{
	Super::Bind(m, d, Map); // fills Bindings (ActuatorMjID / QposAddr / QvelAddr)

	Config = MakeUnique<FMinkConfiguration>(m);
	Config->Update(d->qpos);

	const int32 SiteId = ResolveByName(m, mjOBJ_SITE, EndEffectorFrame);
	const FString ResolvedFrame = (SiteId >= 0) ? FString(UTF8_TO_TCHAR(mj_id2name(m, mjOBJ_SITE, SiteId)))
												: EndEffectorFrame;
	EeTask = MakeUnique<FMinkFrameTask>(ResolvedFrame, EMinkFrameType::Site,
		FMinkVec::Constant(1, PositionCost), FMinkVec::Constant(1, OrientationCost),
		/*gain*/ 1.0, /*lm_damping*/ LmDamping);

	BuildPostureAndDamping(m);
	Posture->SetTargetFromConfiguration(*Config); // posture target = home, like the example
	ConfigLimit = MakeUnique<FMinkConfigurationLimit>(m);

	MocapIndex = -1;
	if (!MocapTargetBody.IsEmpty())
	{
		const int32 Bid = ResolveByName(m, mjOBJ_BODY, MocapTargetBody);
		if (Bid >= 0)
			MocapIndex = m->body_mocapid[Bid];
	}
}

void UMjMinkIKController::SetIKTarget(FVector WorldPos, FQuat WorldRot)
{
	double P[3], Q[4];
	MjUtils::UEToMjPosition(WorldPos, P);
	MjUtils::UEToMjRotation(WorldRot, Q);
	FScopeLock Lock(&TargetMutex);
	for (int32 i = 0; i < 3; ++i)
		ManualTargetPos[i] = P[i];
	for (int32 i = 0; i < 4; ++i)
		ManualTargetQuat[i] = Q[i];
	bHasManualTarget = true;
}

void UMjMinkIKController::ComputeAndApply(mjModel* m, mjData* d, uint8 Source)
{
	if (!bEnabled || !Config.IsValid() || !EeTask.IsValid())
		return;

	// --- resolve the target SE3 ---
	FMinkSE3 Target;
	if (MocapIndex >= 0)
	{
		Target = FMinkSE3::FromMocapId(d, MocapIndex); // matches SE3.from_mocap_name
	}
	else
	{
		FScopeLock Lock(&TargetMutex);
		if (!bHasManualTarget)
			return;
		const double Wxyz[4] = {ManualTargetQuat[0], ManualTargetQuat[1], ManualTargetQuat[2], ManualTargetQuat[3]};
		const FMinkVec3 P(ManualTargetPos[0], ManualTargetPos[1], ManualTargetPos[2]);
		Target = FMinkSE3::FromRotationAndTranslation(FMinkSO3::FromWxyz(Wxyz), P);
	}
	EeTask->SetTarget(Target);

	// --- task + limit lists ---
	TArray<const FMinkBaseTask*> Tasks;
	Tasks.Add(EeTask.Get());
	Tasks.Add(Posture.Get());
	if (bFixBase && BaseDamping.IsValid())
		Tasks.Add(BaseDamping.Get()); // the fix_base branch

	TArray<const FMinkLimit*> Limits;
	Limits.Add(ConfigLimit.Get());

	// --- iterate solve_ik into the reference configuration ---
	const double Dt = m->opt.timestep;
	for (int32 i = 0; i < MaxIters; ++i)
	{
		const FMinkIKResult R = MinkSolveIK(*Config, Tasks, Dt, QpDamping, /*safety*/ false, &Limits);
		if (!R.IsSuccess())
			break;
		Config->IntegrateInplace(R.Velocity, Dt);

		FMinkVec Err;
		if (EeTask->ComputeError(*Config, Err) && Err.size() >= 6)
		{
			if (Err.head(3).norm() <= PosThreshold && Err.tail(3).norm() <= OriThreshold)
				break; // pos + ori achieved
		}
	}

	// --- drive the ACTUATORS (not qpos): data.ctrl[act] = configuration.q[dof] ---
	const FMinkVec Q = Config->GetQ();
	for (const FActuatorBinding& B : Bindings)
	{
		if (B.ActuatorMjID >= 0 && B.QposAddr >= 0 && B.QposAddr < Q.size())
			d->ctrl[B.ActuatorMjID] = Q[B.QposAddr];
	}
}

// ---- config surface (optional but this is the bridge/Blueprint exposure) ----
void UMjMinkIKController::GetConfigSchema(TSharedPtr<FJsonObject>& OutSchema) const
{
	OutSchema = MakeShared<FJsonObject>();
	OutSchema->SetStringField(TEXT("kind"), GetKindName());
	for (const TCHAR* F : {TEXT("position_cost"), TEXT("orientation_cost"), TEXT("posture_cost"),
			 TEXT("lm_damping"), TEXT("qp_damping"), TEXT("fix_base_damping_cost")})
		OutSchema->SetStringField(F, TEXT("number"));
	OutSchema->SetStringField(TEXT("fix_base"), TEXT("bool"));
}

void UMjMinkIKController::GetCurrentConfig(TSharedPtr<FJsonObject>& OutParams) const
{
	OutParams = MakeShared<FJsonObject>();
	OutParams->SetNumberField(TEXT("position_cost"), PositionCost);
	OutParams->SetNumberField(TEXT("orientation_cost"), OrientationCost);
	OutParams->SetNumberField(TEXT("posture_cost"), PostureCost);
	OutParams->SetNumberField(TEXT("lm_damping"), LmDamping);
	OutParams->SetNumberField(TEXT("qp_damping"), QpDamping);
	OutParams->SetNumberField(TEXT("fix_base_damping_cost"), FixBaseDampingCost);
	OutParams->SetBoolField(TEXT("fix_base"), bFixBase);
}

void UMjMinkIKController::ApplyConfig(const TSharedPtr<FJsonObject>& In)
{
	if (!In.IsValid())
		return;
	double V;
	if (In->TryGetNumberField(TEXT("position_cost"), V))       PositionCost = V;
	if (In->TryGetNumberField(TEXT("orientation_cost"), V))    OrientationCost = V;
	if (In->TryGetNumberField(TEXT("posture_cost"), V))        PostureCost = V;
	if (In->TryGetNumberField(TEXT("lm_damping"), V))          LmDamping = V;
	if (In->TryGetNumberField(TEXT("qp_damping"), V))          QpDamping = V;
	if (In->TryGetNumberField(TEXT("fix_base_damping_cost"), V)) FixBaseDampingCost = V;
	bool B;
	if (In->TryGetBoolField(TEXT("fix_base"), B))              bFixBase = B;
	// gains that need task rebuild take effect on next Bind / sim (re)start.
}
```

**Notes on fidelity / choices**
- Drives **`d->ctrl`** via the base class's `Bindings`, so physics stays real — no `qpos` writes, no actuator neutralization. This is the whole point vs the demo.
- The reference `Config` is integrated **open-loop** (only `Update`d once in `Bind`, like the example — the robot tracks via position control). To reject disturbances, add `Config->Update(d->qpos);` at the top of the loop to servo from the live state instead.
- `ComputeAndApply` runs on the physics thread; the mocap-target path is lock-free (the manual-target path takes `TargetMutex`).
- Changing gains that resize task cost vectors (`posture_cost`, `fix_base_damping_cost`) rebuilds cleanly on the next sim start; live per-step cost edits would need `BuildPostureAndDamping` re-called under the callback mutex.

---

## Setup

1. **Get the model.** `mujoco_menagerie/stanford_tidybot`, or Kevin's example scene
   `mink/examples/stanford_tidybot/scene.xml` — the latter already defines the
   `pinch_site_target` **mocap** body and a `home` keyframe, so it's the easiest.
   It has base joints `joint_x` (slide), `joint_y` (slide), `joint_th`, arm
   `joint_1..joint_7`, `position` actuators of the same names, EE site `pinch_site`.

2. **Import into URLab.** Drag `scene.xml` into the Content Browser (or
   `client.scene.import_xml(...)`). You get a robot Blueprint with the base + arm
   joints, their position actuators, `pinch_site`, and the `pinch_site_target`
   mocap body. (URLab may prefix names on import; the controller resolves by
   suffix, so `pinch_site` / `joint_x` etc. still match.)

3. **Add the controller.** Open the generated Blueprint → **Add Component → MuJoCo
   Mink IK Controller**. Defaults already target `pinch_site`, base joints
   `joint_x/joint_y/joint_th`, mocap target `pinch_site_target`. Adjust costs in
   the Details panel if you like.

4. **Level.** Place the robot Blueprint + one **MjManager**. On begin-play the
   manager compiles the model and calls `Bind` on the controller; reset to the
   `home` keyframe for a sane start pose.

5. **Drive it.** Move the `pinch_site_target` mocap body — any of:
   - **Bridge:** `client.runtime.set_mocap_pose("pinch_site_target", pos=[x,y,z], quat=[w,x,y,z])`
   - **VR / Blueprint:** call `SetIKTarget(WorldPos, WorldRot)` each frame (clear
     `MocapTargetBody` so the manual path is used), or drag the mocap actor's
     gizmo in Simulate-In-Editor.
   The base drives toward far targets; the arm handles the rest. Set **`bFixBase`**
   (or `configure_controller {"fix_base": true}`) to pin the base and solve
   arm-only — the example's Enter-key toggle.

6. **Tune at runtime (optional).** `client.runtime.configure_controller(...)` maps
   to `ApplyConfig` — nudge `position_cost` / `orientation_cost` / `posture_cost`
   / `fix_base` live.

## Build wiring

Both files go in the `URLab` runtime module (which already depends on `URLabMink`
and `Eigen`), e.g. `Source/URLab/{Public,Private}/MuJoCo/Components/Controllers/`.
No `Build.cs` change needed.
