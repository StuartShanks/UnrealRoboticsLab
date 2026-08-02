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

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "EngineUtils.h"
#include "Transport/NetworkManager.h"
#include "MuJoCo/Input/MjInputHandler.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "Replay/MjReplayManager.h"
#include "mujoco/mujoco.h"

#include "Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "Transport/ZmqPublishTransport.h"
#include "Transport/ZmqSubscribeTransport.h"
#include "Transport/ZmqRpcTransport.h"
#include "Bridge/RpcDispatcher.h"
#include "Transport/SnapshotProducer.h"
#include "Transport/ShmPublishTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "MuJoCo/Core/MjSimulationState.h"
#include "Utils/URLabLogging.h"
#include "Bridge/BridgeServerProvider.h"
#if WITH_EDITOR
#include "Misc/MessageDialog.h"
#endif

AAMjManager* AAMjManager::Instance = nullptr;

AAMjManager::AAMjManager()
{
	PrimaryActorTick.bCanEverTick = true;

	PhysicsEngine = CreateDefaultSubobject<UMjPhysicsEngine>(TEXT("PhysicsEngine"));
	DebugVisualizer = CreateDefaultSubobject<UMjDebugVisualizer>(TEXT("DebugVisualizer"));
	NetworkManager = CreateDefaultSubobject<UMjNetworkManager>(TEXT("NetworkManager"));
	InputHandler = CreateDefaultSubobject<UMjInputHandler>(TEXT("InputHandler"));
	Perturbation = CreateDefaultSubobject<UMjPerturbation>(TEXT("Perturbation"));
}

// --- Forwarding shims: PreCompile, PostCompile, Compile, ApplyOptions ---
// Actual implementations live in UMjPhysicsEngine.

void AAMjManager::PreCompile()
{
	if (PhysicsEngine)
		PhysicsEngine->PreCompile();
}

void AAMjManager::PostCompile()
{
	if (PhysicsEngine)
		PhysicsEngine->PostCompile();
	BuildEntityCache();
}

void AAMjManager::BuildEntityCache()
{
	EntityCache.Reset();
	if (!PhysicsEngine || !PhysicsEngine->m_model)
		return;
	UWorld* World = GetWorld();
	if (!World)
		return;

	mjModel* m = PhysicsEngine->m_model;

	TSet<AMjArticulation*> ArticSet;
	for (AMjArticulation* A : PhysicsEngine->m_articulations)
		if (A)
			ArticSet.Add(A);

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
			continue;
		if (AMjArticulation* AsArt = Cast<AMjArticulation>(Actor))
		{
			if (ArticSet.Contains(AsArt))
				continue;
		}
		TArray<UMjBody*> Bodies;
		Actor->GetComponents<UMjBody>(Bodies);
		for (UMjBody* B : Bodies)
		{
			if (!B || B->bIsDefault)
				continue;
			int32 Id = B->GetMjID();
			if (Id < 0 || Id >= m->nbody)
			{
				// A scene body that exists as a component but is not bound to
				// the compiled model would silently vanish from the bridge's
				// entity roster (frozen mirror reads client-side) -- say so.
				UE_LOG(LogURLab, Warning,
					TEXT("[BuildEntityCache] Skipping '%s' on actor '%s': unbound mj id %d"),
					*B->GetName(), *Actor->GetName(), Id);
				continue;
			}

			FMjEntityRecord Rec;
			Rec.MjId = Id;
			// Name from the COMPILED model, not GetMjName(): quick-convert
			// bodies never set the MjName property, so GetMjName() returns ""
			// and every record collapses onto the same empty JSON key in the
			// bridge's entities block. mj_id2name is authoritative and unique.
			const char* CompiledName = mj_id2name(m, mjOBJ_BODY, Id);
			Rec.Name = CompiledName ? UTF8_TO_TCHAR(CompiledName) : B->GetName();
			Rec.BodyComp = B;
			if (m->body_jntnum && m->body_jntadr)
			{
				int FirstJnt = m->body_jntadr[Id];
				int NumJnt = m->body_jntnum[Id];
				Rec.bHasFreeBase = (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt && m->jnt_type[FirstJnt] == mjJNT_FREE);
			}
			EntityCache.Add(Rec);
		}
	}

	UE_LOG(LogURLab, Log, TEXT("[BuildEntityCache] %d scene entit%s registered for the bridge"),
		EntityCache.Num(), EntityCache.Num() == 1 ? TEXT("y") : TEXT("ies"));
}

void AAMjManager::Compile()
{
	if (!PhysicsEngine)
		return;

	PhysicsEngine->Compile();

	// Sync discovery lists from PhysicsEngine
	m_MujocoComponents = PhysicsEngine->m_MujocoComponents;
	m_articulations = PhysicsEngine->m_articulations;
	m_heightfieldActors = PhysicsEngine->m_heightfieldActors;
	m_ArticulationMap = PhysicsEngine->m_ArticulationMap;

	// Build the entity roster HERE, after PhysicsEngine->Compile() has run its
	// internal PostCompile (which binds every quick-convert UMjBody's mj id).
	// This used to live only behind AAMjManager::PostCompile(), which nothing
	// calls -- so the cache stayed empty forever, the bridge's step replies
	// shipped an empty `entities` block (the physics-thread path returns empty
	// rather than walking actors), and every client-side mirror read of a
	// quick-converted body silently froze at handshake values.
	BuildEntityCache();
}

void AAMjManager::BeginPlay()
{
	Super::BeginPlay();

	if (Instance != nullptr && Instance != this)
	{
		UE_LOG(LogURLab, Error, TEXT("[AAMjManager] Multiple AAMjManager actors detected in level. Only one is supported — this instance (%s) will be ignored."), *GetName());
		return;
	}
	Instance = this;

	// Auto-create ReplayManager BEFORE the dispatcher / ZMQ components so the
	// dispatcher's Init can cache its pointer. (Transport worker threads later
	// read this cache to avoid TActorIterator, which asserts IsInGameThread.)
	{
		AMjReplayManager* ExistingReplay = Cast<AMjReplayManager>(
			UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
		if (!ExistingReplay)
		{
			FActorSpawnParameters SpawnParams;
			AMjReplayManager* ReplayMgr = GetWorld()->SpawnActor<AMjReplayManager>(SpawnParams);
			if (ReplayMgr)
			{
				UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Auto-created AMjReplayManager"));
			}
		}
	}

	// Resolve a bridge server. In editor builds the URLabEditor module
	// installs a resolver via URLabBridgeProvider that hands back the
	// subsystem's server (lifetime spans PIE sessions). Cooked builds
	// have no resolver, fall through to creating our own and marking it
	// as owned-by-manager so EndPlay tears it down.
	UURLabBridgeServer* ResolvedServer = URLabBridgeProvider::ResolveEditorServer();
	if (ResolvedServer)
	{
		BridgeServer = ResolvedServer;
		// Don't claim ownership — subsystem controls lifetime.
	}
	else
	{
		// Cooked path: no editor subsystem. Manager owns its bridge and
		// brings up both RPC transports inline. Bridge owns all RPC
		// transports; manager only owns publish/subscribe streams.
		BridgeServer = NewObject<UURLabBridgeServer>(this, TEXT("BridgeServer"));
		BridgeServer->SetOwnedByManager(true);
		BridgeServer->Start();          // ZMQ on tcp://0.0.0.0:5559
		BridgeServer->EnsureShmBound(); // SHM under "live"
	}
	BridgeServer->RegisterManager(this);

	// Auto-create the streaming transports (PIE-only producers — they
	// tap PhysicsEngine pre/post-step callbacks). Bridge-style UObject
	// lifecycle: NewObject + SetOwningManager + TransportInit. Manager
	// owns streaming; bridge owns RPC.
	{
		UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Creating manager-owned streaming transports"));

		UURLabZmqPublishTransport* Broadcaster = NewObject<UURLabZmqPublishTransport>(
			this, TEXT("AutoZmqBroadcaster"));
		if (Broadcaster)
		{
			Broadcaster->SetOwningManager(this);
			if (Broadcaster->TransportInit())
			{
				ManagerOwnedPublishTransports.Add(Broadcaster);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabZmqPublishTransport (tcp://0.0.0.0:5555)"));
			}
		}

		UURLabShmPublishTransport* SmPub = NewObject<UURLabShmPublishTransport>(
			this, TEXT("AutoSmSnapshotPublisher"));
		if (SmPub)
		{
			SmPub->SetOwningManager(this);
			if (SmPub->TransportInit())
			{
				ManagerOwnedPublishTransports.Add(SmPub);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabShmPublishTransport (state.shm)"));
			}
		}

		UURLabZmqSubscribeTransport* Subscriber = NewObject<UURLabZmqSubscribeTransport>(
			this, TEXT("AutoZmqSubscriber"));
		if (Subscriber)
		{
			Subscriber->SetOwningManager(this);
			if (Subscriber->TransportInit())
			{
				ManagerOwnedSubscribeTransports.Add(Subscriber);
				UE_LOG(LogURLab, Log,
					TEXT("[AAMjManager] Created UURLabZmqSubscribeTransport (tcp://127.0.0.1:5556)"));
			}
		}
	}

	// Compile via PhysicsEngine (also discovers ZMQ components in PreCompile)
	Compile();
	if (NetworkManager)
		NetworkManager->UpdateCameraStreamingState();

	// Register ONE PreStep + ONE PostStep callback that walks both
	// manager-owned transport arrays.
	if (PhysicsEngine)
	{
		TWeakObjectPtr<AAMjManager> WeakSelf(this);
		PhysicsEngine->RegisterPreStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				AAMjManager* Self = WeakSelf.Get();
				if (!Self)
					return;
				for (const TObjectPtr<UURLabSubscribeTransport>& T : Self->ManagerOwnedSubscribeTransports)
				{
					if (T)
						T->PreStep(m, d);
				}
				for (const TObjectPtr<UURLabPublishTransport>& T : Self->ManagerOwnedPublishTransports)
				{
					if (T)
						T->PreStep(m, d);
				}
			});
		PhysicsEngine->RegisterPostStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				AAMjManager* Self = WeakSelf.Get();
				if (!Self)
					return;
				for (const TObjectPtr<UURLabPublishTransport>& T : Self->ManagerOwnedPublishTransports)
				{
					if (T)
						T->PostStep(m, d);
				}
				for (const TObjectPtr<UURLabSubscribeTransport>& T : Self->ManagerOwnedSubscribeTransports)
				{
					if (T)
						T->PostStep(m, d);
				}
			});
	}

	if (PhysicsEngine)
	{
		// Register debug data capture as a post-step callback. Fires whenever any
		// debug overlay needs fresh mjData — contact forces (key 1), body shader
		// overlays (Island / Segmentation modes), or tendon/muscle rendering.
		PhysicsEngine->RegisterPostStepCallback([this](mjModel* m, mjData* d) {
			if (!DebugVisualizer)
				return;
			const bool bNeedsCapture =
				DebugVisualizer->bShowDebug || DebugVisualizer->DebugShaderMode != EMjDebugShaderMode::Off || DebugVisualizer->bGlobalDrawTendons;
			if (bNeedsCapture)
			{
				DebugVisualizer->CaptureDebugData();
			}
		});

		// Build the state_full snapshot once per physics step and fan it
		// out to every IMjSnapshotPublisher (ZMQ PUB, SHM ring, ...). This
		// is the only place BuildStateSnapshot runs per step.
		TWeakObjectPtr<AAMjManager> WeakSelf(this);
		PhysicsEngine->RegisterPostStepCallback(
			[WeakSelf](mjModel* m, mjData* d) {
				AAMjManager* Self = WeakSelf.Get();
				if (!Self)
					return;

				TArray<IMjSnapshotPublisher*> Pubs;
				{
					FScopeLock Lock(&Self->SnapshotPublishersMutex);
					Pubs.Reserve(Self->SnapshotPublishers.Num());
					for (const FRegisteredSnapshotPublisher& R : Self->SnapshotPublishers)
					{
						if (R.Publisher && R.Owner.IsValid())
							Pubs.Add(R.Publisher);
					}
				}
				if (Pubs.Num() == 0)
					return;

				FURLabRpcDispatcher* Disp = Self->GetStepDispatcher();
				const int64 StepIdx = Disp ? Disp->GetStepCounter() : 0;
				TArray<uint8> Buf = FMjSnapshotProducer::BuildStateSnapshot(
					Self, m, d, StepIdx);
				if (Buf.Num() == 0)
					return;
				for (IMjSnapshotPublisher* Pub : Pubs)
					Pub->PublishSnapshot(Buf);
			});

		// ---- Suction weld-on-attach (pre-step, mocap-style pinning) --------
		// MuJoCo's adhesion actuator holds an object through a compliant
		// contact field: the hold is xy-strong but has NO tilt restoration —
		// a flat cup on a crowned convex hull is a point contact, so a held
		// object ratchets into a corner-dangle under transit accelerations
		// (validated live at every margin/gap/gain/friction tuning point).
		// Real vacuum grippers behave rigidly once sealed, so: while an
		// adhesion actuator's ctrl > 0.5 and a FLOATING body is in contact
		// with (or inside the margin+gap capture band of) the actuator's
		// body, pin that body rigidly to the cup frame each step — exactly
		// how mocap bodies are driven. Release (ctrl < 0.5) restores free
		// dynamics at rest. State lives in the lambda; re-scans on model
		// change; drops attachments on sim reset (time going backwards).
		{
			struct FSuctionWeldSlot
			{
				int ActId = -1;
				int CupBodyId = -1;
				int HeldRootId = -1;
				int QposAdr = -1;
				int DofAdr = -1;
				mjtNum RelPos[3] = {0, 0, 0};
				mjtNum RelQuat[4] = {1, 0, 0, 0};
			};
			struct FSuctionWeldState
			{
				const mjModel* Model = nullptr;
				mjtNum LastTime = -1.0;
				TArray<FSuctionWeldSlot> Slots;
			};
			TSharedPtr<FSuctionWeldState, ESPMode::ThreadSafe> Weld =
				MakeShared<FSuctionWeldState, ESPMode::ThreadSafe>();
			PhysicsEngine->RegisterPreStepCallback([Weld](mjModel* m, mjData* d) {
				if (Weld->Model != m)
				{
					Weld->Model = m;
					Weld->Slots.Reset();
					for (int i = 0; i < m->nu; ++i)
					{
						if (m->actuator_trntype[i] != mjTRN_BODY)
							continue; // adhesion is the only body-transmission actuator
						FSuctionWeldSlot S;
						S.ActId = i;
						S.CupBodyId = m->actuator_trnid[2 * i];
						Weld->Slots.Add(S);
					}
				}
				if (d->time < Weld->LastTime)
					for (FSuctionWeldSlot& S : Weld->Slots)
						S.HeldRootId = -1;
				Weld->LastTime = d->time;

				for (FSuctionWeldSlot& S : Weld->Slots)
				{
					if (d->ctrl[S.ActId] <= 0.5)
					{
						if (S.HeldRootId >= 0)
							UE_LOG(LogURLab, Log,
								TEXT("[SuctionWeld] released body %d"), S.HeldRootId);
						S.HeldRootId = -1;
						continue;
					}
					if (S.HeldRootId < 0)
					{
						const int CupRoot = m->body_rootid[S.CupBodyId];
						for (int ci = 0; ci < d->ncon; ++ci)
						{
							const mjContact& C = d->contact[ci];
							const int B1 = m->geom_bodyid[C.geom1];
							const int B2 = m->geom_bodyid[C.geom2];
							int Other = -1;
							if (B1 == S.CupBodyId)
								Other = B2;
							else if (B2 == S.CupBodyId)
								Other = B1;
							if (Other < 0)
								continue;
							const int Root = m->body_rootid[Other];
							if (Root == CupRoot || Root == 0)
								continue; // never weld to self or the world
							if (m->body_jntnum[Root] != 1)
								continue;
							const int J = m->body_jntadr[Root];
							if (m->jnt_type[J] != mjJNT_FREE)
								continue; // floating bodies only
							mjtNum NegP[3], NegQ[4];
							mju_negPose(NegP, NegQ,
								d->xpos + 3 * S.CupBodyId, d->xquat + 4 * S.CupBodyId);
							mju_mulPose(S.RelPos, S.RelQuat, NegP, NegQ,
								d->xpos + 3 * Root, d->xquat + 4 * Root);
							S.HeldRootId = Root;
							S.QposAdr = m->jnt_qposadr[J];
							S.DofAdr = m->jnt_dofadr[J];
							UE_LOG(LogURLab, Log,
								TEXT("[SuctionWeld] attached body %d to cup body %d"),
								Root, S.CupBodyId);
							break;
						}
					}
					if (S.HeldRootId >= 0)
					{
						mjtNum P[3], Q[4];
						mju_mulPose(P, Q,
							d->xpos + 3 * S.CupBodyId, d->xquat + 4 * S.CupBodyId,
							S.RelPos, S.RelQuat);
						mju_copy3(d->qpos + S.QposAdr, P);
						mju_copy4(d->qpos + S.QposAdr + 3, Q);
						mju_zero(d->qvel + S.DofAdr, 6);
					}
				}
			});
		}

		PhysicsEngine->RunMujocoAsync();
	}

	// Auto-create simulate widget AFTER Compile so articulations are registered
	if (bAutoCreateSimulateWidget && !SimulateWidget)
	{
		static const TCHAR* WidgetBPPath = TEXT("/UnrealRoboticsLab/UI/WBP_MjSimulate.WBP_MjSimulate_C");
		UClass* WidgetClass = LoadClass<UUserWidget>(nullptr, WidgetBPPath);
		if (WidgetClass)
		{
			APlayerController* PC = GetWorld()->GetFirstPlayerController();
			if (PC)
			{
				UUserWidget* Widget = CreateWidget<UUserWidget>(PC, WidgetClass);
				if (Widget)
				{
					Widget->AddToViewport(0);
					SimulateWidget = Widget;
					UE_LOG(LogURLab, Log, TEXT("[AAMjManager] Auto-created MjSimulate widget (press Tab to show)"));
				}
			}
		}
		else
		{
			UE_LOG(LogURLab, Warning, TEXT("[AAMjManager] Could not load WBP_MjSimulate Blueprint class. Widget not created."));
		}
	}
}

void AAMjManager::ToggleSimulateWidget()
{
	if (SimulateWidget)
	{
		bool bIsVisible = SimulateWidget->IsVisible();
		SimulateWidget->SetVisibility(bIsVisible ? ESlateVisibility::Collapsed : ESlateVisibility::Visible);
	}
}

void AAMjManager::RegisterSnapshotPublisher(IMjSnapshotPublisher* Publisher,
	UObject* OwnerObj)
{
	if (!Publisher || !OwnerObj)
		return;
	FScopeLock Lock(&SnapshotPublishersMutex);
	for (const FRegisteredSnapshotPublisher& R : SnapshotPublishers)
	{
		if (R.Publisher == Publisher)
			return; // already registered
	}
	SnapshotPublishers.Add({OwnerObj, Publisher});
}

void AAMjManager::UnregisterSnapshotPublisher(IMjSnapshotPublisher* Publisher)
{
	if (!Publisher)
		return;
	FScopeLock Lock(&SnapshotPublishersMutex);
	SnapshotPublishers.RemoveAll([Publisher](const FRegisteredSnapshotPublisher& R) {
		return R.Publisher == Publisher;
	});
}

void AAMjManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Stop the physics async thread BEFORE Super::EndPlay so PostStep
	// callbacks don't race into resources child components tear down.
	// Bounded wait: a pathological mj_step can take many seconds; an
	// unbounded Wait() would freeze PIE-stop. On timeout we detach and
	// leak m_model/m_data to avoid use-after-free in the still-running
	// step (one-time per session).
	bool bAsyncExited = true;
	if (PhysicsEngine)
	{
		PhysicsEngine->bShouldStopTask = true;
		// Wake the worker if parked on the step-request event so it
		// observes bShouldStopTask without burning the Wait timeout.
		if (PhysicsEngine->StepRequestEvent)
			PhysicsEngine->StepRequestEvent->Trigger();
		if (PhysicsEngine->AsyncPhysicsFuture.IsValid())
		{
			constexpr double kShutdownTimeoutSec = 3.0;
			bAsyncExited = PhysicsEngine->AsyncPhysicsFuture.WaitFor(
				FTimespan::FromSeconds(kShutdownTimeoutSec));
			if (!bAsyncExited)
			{
				UE_LOG(LogURLab, Warning,
					TEXT("Physics async thread did not exit within %.1fs — detaching. ")
						TEXT("mj_step is likely stuck; MuJoCo resources will leak for this session ")
							TEXT("to avoid a use-after-free in the still-running step."),
					kShutdownTimeoutSec);
			}
		}
		PhysicsEngine->ClearCallbacks();
	}

	// Manager-owned transports aren't UActorComponents, so EndPlay
	// doesn't propagate to them; explicit TransportShutdown required.
	for (TObjectPtr<UURLabSubscribeTransport>& T : ManagerOwnedSubscribeTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	ManagerOwnedSubscribeTransports.Reset();
	for (TObjectPtr<UURLabPublishTransport>& T : ManagerOwnedPublishTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	ManagerOwnedPublishTransports.Reset();

	Super::EndPlay(EndPlayReason);
	if (Instance == this)
		Instance = nullptr;

	// Manager-owned servers also tear down the dispatcher itself;
	// subsystem-owned servers stay alive across PIE cycles.
	if (BridgeServer)
	{
		BridgeServer->UnregisterManager(this);
		if (BridgeServer->IsOwnedByManager())
		{
			BridgeServer->Stop();
		}
		BridgeServer = nullptr;
	}

	// Clear tracked actors to prevent dangling pointers on level restart
	m_heightfieldActors.Empty();
	m_articulations.Empty();
	m_MujocoComponents.Empty();

	// Only touch MuJoCo resources if the async thread actually exited — a
	// detached thread may still be executing mj_step and reading these.
	if (PhysicsEngine && bAsyncExited)
	{
		if (PhysicsEngine->m_data)
		{
			mj_deleteData(PhysicsEngine->m_data);
			PhysicsEngine->m_data = nullptr;
		}
		if (PhysicsEngine->m_model)
		{
			mj_deleteModel(PhysicsEngine->m_model);
			PhysicsEngine->m_model = nullptr;
		}
		if (PhysicsEngine->m_spec)
		{
			mj_deleteVFS(&PhysicsEngine->m_vfs);
			mj_deleteSpec(PhysicsEngine->m_spec);
			PhysicsEngine->m_spec = nullptr;
		}

		PhysicsEngine->m_heightfieldActors.Empty();
		PhysicsEngine->m_articulations.Empty();
		PhysicsEngine->m_MujocoComponents.Empty();
	}
}

void AAMjManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!PhysicsEngine || !PhysicsEngine->IsInitialized())
	{
		return;
	}

	const TArray<AMjArticulation*> Arts = PhysicsEngine->GetAllArticulations();
	const TArray<UMjQuickConvertComponent*> Quicks = PhysicsEngine->GetAllQuickComponents();

	PhysicsEngine->WithRenderState([&](const FMjRenderSnapshot& Snap) {
		for (AMjArticulation* Art : Arts)
		{
			if (Art)
			{
				Art->ApplyRenderState(Snap);
			}
		}
		for (UMjQuickConvertComponent* Quick : Quicks)
		{
			if (Quick)
			{
				Quick->ApplyRenderState(Snap);
			}
		}
	});
}

AAMjManager* AAMjManager::GetManager()
{
	return Instance;
}

void AAMjManager::SetPaused(bool bPaused)
{
	if (PhysicsEngine)
		PhysicsEngine->SetPaused(bPaused);
}

bool AAMjManager::IsRunning() const
{
	return PhysicsEngine ? PhysicsEngine->IsRunning() : false;
}

bool AAMjManager::IsInitialized() const
{
	return PhysicsEngine ? PhysicsEngine->IsInitialized() : false;
}

FString AAMjManager::GetLastCompileError() const
{
	return PhysicsEngine ? PhysicsEngine->GetLastCompileError() : FString();
}

void AAMjManager::StepSync(int32 NumSteps)
{
	if (PhysicsEngine)
		PhysicsEngine->StepSync(NumSteps);
}

bool AAMjManager::CompileModel()
{
	if (!PhysicsEngine)
		return false;

	bool Result = PhysicsEngine->CompileModel();

	// Re-sync discovery lists after recompile
	m_MujocoComponents = PhysicsEngine->m_MujocoComponents;
	m_articulations = PhysicsEngine->m_articulations;
	m_heightfieldActors = PhysicsEngine->m_heightfieldActors;
	m_ArticulationMap = PhysicsEngine->m_ArticulationMap;

	// Rebuild the entity roster: a recompile invalidates every cached mj id,
	// so a stale cache here would be worse than an empty one (dangling ids in
	// the bridge's entities block). Same rationale as in Compile().
	BuildEntityCache();

	return Result;
}

AMjArticulation* AAMjManager::GetArticulation(const FString& ActorName) const
{
	return PhysicsEngine ? PhysicsEngine->GetArticulation(ActorName) : nullptr;
}

TArray<AMjArticulation*> AAMjManager::GetAllArticulations() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllArticulations() : m_articulations;
}

TArray<UMjQuickConvertComponent*> AAMjManager::GetAllQuickComponents() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllQuickComponents() : m_MujocoComponents;
}

TArray<AMjHeightfieldActor*> AAMjManager::GetAllHeightfields() const
{
	return PhysicsEngine ? PhysicsEngine->GetAllHeightfields() : m_heightfieldActors;
}

float AAMjManager::GetSimTime() const
{
	return PhysicsEngine ? PhysicsEngine->GetSimTime() : 0.0f;
}

float AAMjManager::GetTimestep() const
{
	return PhysicsEngine ? PhysicsEngine->GetTimestep() : 0.002f;
}

// --- Recording / replay ---

void AAMjManager::StartRecording()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StartRecording();
		UE_LOG(LogURLab, Log, TEXT("StartRecording called on ReplayManager."));
	}
	else
	{
		UE_LOG(LogURLab, Warning, TEXT("ReplayManager not found in scene!"));
	}
}

void AAMjManager::StopRecording()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StopRecording();
		UE_LOG(LogURLab, Log, TEXT("StopRecording called on ReplayManager."));
	}
	// OnPostStep callback stays registered; the replay manager gates
	// recording with bIsRecording so it can be restarted without re-registering.
}

void AAMjManager::StartReplay()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StartReplay();
		UE_LOG(LogURLab, Log, TEXT("StartReplay called on ReplayManager."));
	}
}

void AAMjManager::StopReplay()
{
	AMjReplayManager* ReplayMgr = Cast<AMjReplayManager>(UGameplayStatics::GetActorOfClass(GetWorld(), AMjReplayManager::StaticClass()));
	if (ReplayMgr)
	{
		ReplayMgr->StopReplay();
		UE_LOG(LogURLab, Log, TEXT("StopReplay called on ReplayManager."));
	}
}

void AAMjManager::ResetSimulation()
{
	if (PhysicsEngine)
	{
		PhysicsEngine->ResetSimulation();
	}
	UE_LOG(LogURLab, Log, TEXT("MuJoCo Manager: Reset requested."));
}

UMjSimulationState* AAMjManager::CaptureSnapshot()
{
	return PhysicsEngine ? PhysicsEngine->CaptureSnapshot() : nullptr;
}

void AAMjManager::RestoreSnapshot(UMjSimulationState* Snapshot)
{
	if (PhysicsEngine)
		PhysicsEngine->RestoreSnapshot(Snapshot);
}