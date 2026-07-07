# Fully data-driven mink IK controller — no hardcoded strings, no child classes

Yes — everything should live on the component as `UPROPERTY`s (arrays + sliders +
`UMjComponent` refs), and the C++ builds the mink task stack *from that data* at
`Bind`. One `UMjMinkIKController` covers every robot; swapping a TidyBot for a
different-arm/base bot, or turning the frame task into an "engraving" setup, is
Details-panel edits — no new `.cpp`, no subclasses.

The earlier sketch hardcoded `"pinch_site"`, `joint_x/y/th`, etc. — that was the
mistake. Here's the general version.

## The idea

- The task stack is a `TArray<FMinkTaskSpec>` — each entry is a task *description*
  (kind + frame + costs + which joints it weights), editable in the panel.
- Limits are a `TArray<FMinkLimitSpec>`.
- Driven / base joints are **`UMjComponent` arrays** (`TArray<TObjectPtr<UMjJoint>>`) —
  you pick the actual joint components, so it survives renames and needs no string
  matching. (Frames that are *sites* stay as name-with-dropdown, since sites aren't
  components — see note at the end.)
- Every scalar is a slider (`UIMin`/`UIMax` meta). Every name is a dropdown
  (`GetOptions`, the same idiom `UMjEndEffectorController::GetBodyOptions` already uses).
- `Bind()` walks the specs and constructs the real `FMinkFrameTask` / `FMinkPostureTask`
  / … via a factory `switch`. That's the only place tasks are created.

## Spec structs (BlueprintType, so they show in the panel)

```cpp
UENUM(BlueprintType)
enum class EMinkTaskKind : uint8
{
    Frame, RelativeFrame, Posture, Damping, Com, DofFreezing, KineticEnergyReg
};

USTRUCT(BlueprintType)
struct FMinkTaskSpec
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task")
    EMinkTaskKind Kind = EMinkTaskKind::Frame;

    /** Frame/RelativeFrame: the driven frame — a UMjComponent (UMjSite / UMjBody /
        UMjGeom / UMjFrame). The mink frame type is inferred from the component class. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task",
        meta=(EditCondition="Kind==EMinkTaskKind::Frame||Kind==EMinkTaskKind::RelativeFrame"))
    TObjectPtr<UMjComponent> Frame;

    /** RelativeFrame only: the root the frame is measured against. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task",
        meta=(EditCondition="Kind==EMinkTaskKind::RelativeFrame"))
    TObjectPtr<UMjComponent> RootFrame;

    /** Where the Frame task reads its target from. Empty => driven via SetIKTarget()/VR. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task",
        meta=(EditCondition="Kind==EMinkTaskKind::Frame"))
    TObjectPtr<UMjBody> TargetMocapBody;

    // ---- gains (sliders) ----
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task|Gains", meta=(ClampMin="0.0", UIMin="0.0", UIMax="10.0"))
    float PositionCost = 1.f;   // Frame
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task|Gains", meta=(ClampMin="0.0", UIMin="0.0", UIMax="10.0"))
    float OrientationCost = 1.f; // Frame
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task|Gains", meta=(ClampMin="0.0", UIMin="0.0", UIMax="1000.0"))
    float Cost = 1e-2f;         // Posture/Damping/Com scalar
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task|Gains", meta=(ClampMin="0.0", UIMin="0.0", UIMax="5.0"))
    float Gain = 1.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task|Gains", meta=(ClampMin="0.0", UIMin="0.0", UIMax="5.0"))
    float LmDamping = 0.f;

    /** Joints this task's scalar cost applies to (Posture/Damping/DofFreezing).
        Empty => all DOFs. This is how you get tidybot's posture[3:]/damping[:3]
        without any hardcoding — just pick the base joints here. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task",
        meta=(EditCondition="Kind==EMinkTaskKind::Posture||Kind==EMinkTaskKind::Damping||Kind==EMinkTaskKind::DofFreezing"))
    TArray<TObjectPtr<UMjJoint>> Joints;

    /** Enable/disable at runtime (e.g. a "fix base" damping task) without deleting it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Task")
    bool bEnabled = true;
};

UENUM(BlueprintType)
enum class EMinkLimitKind : uint8 { Configuration, Velocity, CollisionAvoidance };

USTRUCT(BlueprintType)
struct FMinkLimitSpec
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite) EMinkLimitKind Kind = EMinkLimitKind::Configuration;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(ClampMin="0.0", UIMin="0.0", UIMax="1.0")) float Gain = 0.95f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta=(ClampMin="0.0")) float MinDistance = 0.f;
    // Velocity: per-joint max; CollisionAvoidance: geom pairs + min-dist — same pattern.
};
```

## The component — all data, no per-robot code

```cpp
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="MuJoCo Mink IK Controller"))
class URLAB_API UMjMinkIKController : public UMjArticulationController
{
    GENERATED_BODY()
public:
    /** The whole task stack — compose in the panel. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mink IK")
    TArray<FMinkTaskSpec> Tasks;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mink IK")
    TArray<FMinkLimitSpec> Limits;

    /** Actuated joints to command (empty => every actuator the base bound). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mink IK")
    TArray<TObjectPtr<UMjJoint>> DriveJoints;

    // solver params — sliders
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mink IK|Solver", meta=(ClampMin="1", UIMin="1", UIMax="50"))
    int32 MaxIters = 20;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mink IK|Solver", meta=(ClampMin="0.0", UIMin="0.0", UIMax="1e-2"))
    float QpDamping = 1e-3f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mink IK|Solver", meta=(ClampMin="0.0"))
    float PosThreshold = 1e-4f, OriThreshold = 1e-4f;

    UFUNCTION(BlueprintCallable) void SetIKTarget(int32 TaskIndex, FVector WorldPos, FQuat WorldRot);

    virtual void Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& Map) override;
    virtual void ComputeAndApply(mjModel* m, mjData* d, uint8 Source) override;

    // No GetOptions helpers needed — every reference above is a UMjComponent picker.

private:
    TUniquePtr<FMinkConfiguration> Config;
    TArray<TUniquePtr<FMinkBaseTask>> BuiltTasks;   // parallel to Tasks
    TArray<TUniquePtr<FMinkLimit>>    BuiltLimits;
    TArray<int32> DriveActuatorIds, DriveQposAddrs; // resolved from DriveJoints (or all Bindings)
    // per-frame-task target routing (mocap index or manual SE3) …
};
```

## `Bind()` — the only place tasks are constructed

```cpp
void UMjMinkIKController::Bind(mjModel* m, mjData* d, const TMap<int32,UMjActuator*>& Map)
{
    Super::Bind(m, d, Map);
    Config = MakeUnique<FMinkConfiguration>(m);
    Config->Update(d->qpos);

    auto DofsOf = [&](const TArray<TObjectPtr<UMjJoint>>& JJ){
        TSet<int32> S; for (UMjJoint* J : JJ) if (J) S.Add(m->jnt_dofadr[J->GetMjID()]); return S; };

    for (const FMinkTaskSpec& S : Tasks)
    {
        TUniquePtr<FMinkBaseTask> T;
        switch (S.Kind)
        {
        case EMinkTaskKind::Frame:
        {
            const EMinkFrameType FT = FrameTypeOf(S.Frame);       // UMjSite->Site, UMjGeom->Geom, else Body
            const FString FrameName = CompiledName(m, S.Frame);   // mj_id2name(m, objType, S.Frame->GetMjID())
            auto F = MakeUnique<FMinkFrameTask>(FrameName, FT,
                FMinkVec::Constant(1, S.PositionCost), FMinkVec::Constant(1, S.OrientationCost),
                S.Gain, S.LmDamping);
            // target routing: S.TargetMocapBody -> m->body_mocapid[GetMjID()], else manual SE3
            T = MoveTemp(F);
            break;
        }
        case EMinkTaskKind::Posture:
        {
            FMinkVec c = FMinkVec::Zero(m->nv);
            if (S.Joints.Num()==0) c.setConstant(S.Cost);
            else for (int32 a : DofsOf(S.Joints)) c[a] = S.Cost;   // tidybot posture[3:] etc, generalised
            auto P = MakeUnique<FMinkPostureTask>(m, c);
            P->SetTargetFromConfiguration(*Config);
            T = MoveTemp(P);
            break;
        }
        case EMinkTaskKind::Damping:
        {
            FMinkVec c = FMinkVec::Zero(m->nv);
            if (S.Joints.Num()==0) c.setConstant(S.Cost);
            else for (int32 a : DofsOf(S.Joints)) c[a] = S.Cost;   // fix-base = pick base joints here
            T = MakeUnique<FMinkDampingTask>(m, c);
            break;
        }
        // Com / RelativeFrame / DofFreezing / KineticEnergyReg: same shape, different ctor
        default: break;
        }
        BuiltTasks.Add(MoveTemp(T));
    }

    for (const FMinkLimitSpec& L : Limits)
        if (L.Kind == EMinkLimitKind::Configuration)
            BuiltLimits.Add(MakeUnique<FMinkConfigurationLimit>(m, L.Gain, L.MinDistance));
        // Velocity / CollisionAvoidance likewise

    // resolve which actuators to write: DriveJoints, else everything the base bound
    if (DriveJoints.Num()==0)
        for (const FActuatorBinding& B : Bindings) { DriveActuatorIds.Add(B.ActuatorMjID); DriveQposAddrs.Add(B.QposAddr); }
    else { /* match DriveJoints -> their actuators via m->actuator_trnid */ }
}
```

`ComputeAndApply` is unchanged in spirit: set each Frame task's target, run the
`MaxIters` solve/integrate loop over `BuiltTasks` (+enabled `BuiltLimits`), then
`d->ctrl[DriveActuatorIds[i]] = Config->GetQ()[DriveQposAddrs[i]]`.

## Why this hits Buzz's asks

- **No hardcoded strings** — every frame/joint is a `UMjComponent` ref; costs are sliders.
- **No child classes** — the task stack is *data*. Different arm+base TidyBot →
  repoint `DriveJoints` and the Frame/Posture/Damping specs, done. Engraving task →
  add a Frame task on the tool tip + an orientation/plane constraint spec. Same class.
- **`UMjComponent` arrays** — `TArray<TObjectPtr<UMjJoint>>` for driven/base joints,
  robust to renames, no `mj_name2id` guessing.
- **Play with it live** — the specs are `EditAnywhere`/`BlueprintReadWrite`, and the
  same array serialises to JSON for `GetConfigSchema`/`ApplyConfig`, so the bridge's
  `configure_controller` gets the general surface for free.

**Correction (Buzz was right — I'd guessed wrong earlier):** sites *are* `UMjComponent`s.
`UMjSite`, `UMjBody`, `UMjGeom`, and `UMjFrame` all derive from `UMjComponent`, so the
frame is a component ref like everything else (`TObjectPtr<UMjComponent> Frame`) and its
mink **frame type is inferred from the component's class** (`UMjSite`→Site, `UMjGeom`→Geom,
else Body). No `FrameType` field, no name strings anywhere on the component — every
reference is a `UMjComponent` picker.
