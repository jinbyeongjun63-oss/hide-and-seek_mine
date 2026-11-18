using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(NavMeshAgent))]
public sealed class HideSeekAgent : Agent
{
    [SerializeField] private HideSeekAgent opponent;
    [SerializeField] private bool seeker;
    [SerializeField] private HideSeekAgent secondSeeker;
    [SerializeField] private bool isRoundAssistant;

    [SerializeField] private bool practicePassageStart = true;
    private int practiceLayoutTicks;

    internal const float PreparationSeconds = 7f;
    internal const float SearchSeconds = 60f;
    private const float ViewRange = 100f;
    private const float HalfViewAngle = 37.5f;
    private const float ChaseSpeedMultiplier = 1.7f;
    private float baseSpeed;
    private const float TagDistance = 2f;
    [SerializeField] private BoxCollider movableCrate;
    [SerializeField] private HideSeekAgent secondRunner;
    private HideSeekAgent visibleTarget;
    private Quaternion crateStartRotation;
    [SerializeField] private BoxCollider secondCrate;
    private BoxCollider selectedCrate;
    private BoxCollider movedCover;
    private float coverSeconds;
    private bool coverRewarded;
    private Vector3 secondStart;
    private Quaternion secondRotation;
    private Vector3 crateStart;
    private int crateAction;
    private const int GrabCrate = 9;
    private const int ToggleCrateLock = 10;
    private const int OrbitFirst = 11;
    private const int DirectionalCarryFirst = 19;
    private const int ActionCount = 27;
    private static readonly Dictionary<BoxCollider, HideSeekAgent> CrateHandlers = new();
    private static readonly Dictionary<BoxCollider, HideSeekAgent> CrateLocks = new();
    private BoxCollider heldCrate;
    private int previousCrateAction;
    private bool lockKeyPending;
    private GameObject lockMarks;
    private Material lockMaterial;
    private const float CrateReach = 1.8f;
    private const float SeekerPushSpeed = 2.5f;
    private const float SeekerPushReach = 0.3f;
    private const int BeamSegments = 32;
    private Mesh beamMesh;
    private Material beamMaterial;
    private MeshRenderer beamRenderer;
    private readonly Vector3[] beamVertices = new Vector3[BeamSegments + 2];

    private static readonly Vector3[] Directions =
    {
        Vector3.forward, Vector3.back, Vector3.left, Vector3.right,
        new Vector3(-1, 0, 1).normalized, new Vector3(1, 0, 1).normalized,
        new Vector3(-1, 0, -1).normalized, new Vector3(1, 0, -1).normalized
    };

    private NavMeshAgent navigation;
    private Vector3 start;
    private Vector3 direction;
    private Vector3 lastSeen;
    private float lastSeenAt;
    private bool hasMemory;
    private float elapsed;
    private float visibleSeconds;
    private int discoveries;
    private bool wasVisible;
    private bool hiddenDuringSearch;
    private bool rediscovered;
    private readonly HashSet<Vector2Int> searchedCells = new();
    private Vector3 spawn;
    private float travelDistance;
    private NavMeshPath trainingPath;
    private Vector3 previousPosition;
    private const int SearchHints = 4;
    private const float RecheckSeconds = 12f;
    private const float SearchCellSize = 10f;
    private readonly List<Vector3[]> surveyCells = new();
    private float[] surveyedAt;
    private float nextSurvey;
    private bool preparationBaselinesCaptured;
    private readonly (Vector3 from, Vector3 to, float length, float startGap, BoxCollider firstWall, BoxCollider secondWall)?[] passageBaselines =
        new (Vector3, Vector3, float, float, BoxCollider, BoxCollider)?[2];
    private int passagesBlocked;
    private readonly HashSet<BoxCollider> rewardedBarrierCrates = new();
    private int passagesMeasured;
    private int preparationCrateMoves;
    private int lastCountedCrateAction;
    private BoxCollider lastCountedCrate;
    private bool preparationScored;
    private bool movedCrateDuringPreparation;
    private Vector3 reportedPosition;
    private float reportedAt = float.NegativeInfinity;
    private NavMeshPath responsePath;
    private Vector3[] responseCorners = System.Array.Empty<Vector3>();
    private int responseCorner;
    private float nextResponsePath;
    private int patrolCell = -1;
    private float nextPatrolAttempt;
    private float patrolProgressAt;
    private Vector3 patrolProgressPosition;
    private readonly Dictionary<int, float> patrolRetryAt = new();

    internal bool IsSearching => (seeker ? elapsed : opponent.elapsed) >= PreparationSeconds;
    private Vector3 Position => navigation.nextPosition;

    public override void Initialize()
    {
        navigation = GetComponent<NavMeshAgent>();
        trainingPath = new NavMeshPath();
        if (seeker)
        {
            baseSpeed = navigation.speed;
            responsePath = new NavMeshPath();
        }
        start = transform.position;
        if (movableCrate)
        {
            crateStart = movableCrate.transform.position;
            crateStartRotation = movableCrate.transform.rotation;
        }
        if (secondCrate)
        {
            secondStart = secondCrate.transform.position;
            secondRotation = secondCrate.transform.rotation;
        }
        spawn = start;
        previousPosition = start;
        if (seeker && !Application.isBatchMode)
            CreateBeam();
    }

    internal bool CanSeeOpponent()
    {
        visibleTarget = CanSee(opponent) ? opponent : null;
        HideSeekAgent otherRunner = seeker ? (isRoundAssistant ? opponent.opponent.secondRunner : secondRunner) : null;
        if (CanSee(otherRunner) && (!visibleTarget ||
            (otherRunner.Position - Position).sqrMagnitude < (visibleTarget.Position - Position).sqrMagnitude))
            visibleTarget = otherRunner;
        return visibleTarget != null;
    }

    private bool CanSee(HideSeekAgent target)
    {
        if (!target || target.navigation == null || !target.navigation.isOnNavMesh || !navigation.isOnNavMesh)
            return false;

        Vector3 offset = target.Position - Position;
        return offset.sqrMagnitude <= ViewRange * ViewRange &&
            (!seeker || Vector3.Angle(transform.forward, offset) <= HalfViewAngle) &&
            !navigation.Raycast(target.Position, out _);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float time = seeker ? elapsed : opponent.elapsed;
        bool canSee = (!seeker || IsSearching) && CanSeeOpponent();
        if (canSee)
        {
            lastSeen = visibleTarget.Position;
            lastSeenAt = time;
            hasMemory = true;
        }

        sensor.AddObservation(seeker ? 1f : 0f);
        sensor.AddObservation(IsSearching ? 1f : 0f);
        sensor.AddObservation(time / (PreparationSeconds + SearchSeconds));
        ObserveXZ(sensor, Position / 50f);
        ObserveXZ(sensor, direction);
        sensor.AddObservation(canSee ? 1f : 0f);
        ObserveXZ(sensor, canSee ? (visibleTarget.Position - Position) / ViewRange : Vector3.zero);
        sensor.AddObservation(hasMemory ? 1f : 0f);
        ObserveXZ(sensor, hasMemory ? (lastSeen - Position) / 80f : Vector3.zero);
        sensor.AddObservation(hasMemory ? Mathf.Clamp01((time - lastSeenAt) / SearchSeconds) : 1f);
        foreach (Vector3 heading in Directions)
            sensor.AddObservation(Clearance(heading, 3f) / 3f);
        ObserveSearchMap(sensor);
        bool canHandleCrate = TryGetCrateHeading(out _);
        BoxCollider observed = canHandleCrate ? selectedCrate : movableCrate;
        if (!canHandleCrate && secondCrate && (!observed ||
            (secondCrate.bounds.center - Position).sqrMagnitude < (observed.bounds.center - Position).sqrMagnitude))
            observed = secondCrate;
        ObserveXZ(sensor, observed ? (observed.bounds.center - Position) / 80f : Vector3.zero);
        sensor.AddObservation(canHandleCrate ? 1f : 0f);
        bool locked = observed && CrateLocks.TryGetValue(observed, out _);
        sensor.AddObservation(locked ? 1f : 0f);
        sensor.AddObservation(locked && CrateLocks[observed] == this ? 1f : 0f);
    }

    private void ObserveSearchMap(VectorSensor sensor)
    {
        HideSeekAgent leader = seeker ? (isRoundAssistant ? opponent.opponent : this) : null;
        HideSeekAgent teammate = leader != null ? (leader == this ? secondSeeker : leader) : null;
        bool hasTeammate = teammate != null && teammate.navigation != null && teammate.navigation.isOnNavMesh;
        ObserveXZ(sensor, hasTeammate ? (teammate.Position - Position) / 80f : Vector3.zero);
        int used = 0;
        if (leader != null && leader.surveyedAt != null && IsSearching && navigation.isOnNavMesh)
        {
            var candidates = new List<(int index, float score)>();
            for (int i = 0; i < leader.surveyCells.Count; i++)
            {
                if (elapsed - leader.surveyedAt[i] < RecheckSeconds)
                    continue;

                Vector3 point = leader.surveyCells[i][0];
                float distance = (point - Position).sqrMagnitude;
                // Prefer points closer to this seeker than to its teammate.
                float score = distance + (hasTeammate && (point - teammate.Position).sqrMagnitude < distance ? 400f : 0f);
                candidates.Add((i, score));
            }
            candidates.Sort((a, b) => a.score.CompareTo(b.score));
            for (int i = 0; i < candidates.Count && i < 12 && used < SearchHints; i++)
            {
                int cell = candidates[i].index;
                Vector3 point = leader.surveyCells[cell][0];
                if (!navigation.CalculatePath(point, trainingPath) || trainingPath.status != NavMeshPathStatus.PathComplete)
                    continue;

                float length = 0f;
                Vector3 previous = Position;
                foreach (Vector3 corner in trainingPath.corners)
                {
                    length += Vector3.Distance(previous, corner);
                    previous = corner;
                }
                ObserveXZ(sensor, (point - Position) / 80f);
                sensor.AddObservation(Mathf.Clamp01(length / 100f));
                sensor.AddObservation(1f);
                sensor.AddObservation((cell + 1f) / leader.surveyCells.Count);
                used++;
            }
        }

        for (; used < SearchHints; used++)
            for (int field = 0; field < 5; field++)
                sensor.AddObservation(0f);
    }

    private void UpdateSurvey()
    {
        if (elapsed < nextSurvey)
            return;

        nextSurvey = elapsed + 0.4f;
        if (surveyedAt == null)
        {
            BuildSearchGrid();
            surveyedAt = new float[surveyCells.Count];
            System.Array.Fill(surveyedAt, float.NegativeInfinity);
        }

        for (int i = 0; i < surveyCells.Count; i++)
        {
            if (elapsed - surveyedAt[i] < RecheckSeconds)
                continue;

            bool allVisible = true;
            foreach (Vector3 point in surveyCells[i])
            {
                if (!CanSurvey(point) && !(secondSeeker && secondSeeker.CanSurvey(point)))
                {
                    allVisible = false;
                    break;
                }
            }
            if (!allVisible)
                continue;

            if (float.IsNegativeInfinity(surveyedAt[i]))
            {
                AddReward(0.015f);
                if (secondSeeker) secondSeeker.AddReward(0.015f);
            }
            surveyedAt[i] = elapsed;
        }
    }

    private void BuildSearchGrid()
    {
        Vector3[] vertices = NavMesh.CalculateTriangulation().vertices;
        if (vertices.Length == 0)
            return;

        var bounds = new Bounds(vertices[0], Vector3.zero);
        foreach (Vector3 vertex in vertices)
            bounds.Encapsulate(vertex);

        var filter = new NavMeshQueryFilter
        {
            agentTypeID = navigation.agentTypeID,
            areaMask = navigation.areaMask
        };
        var ground = GameObject.Find("Ground")?.GetComponent<Collider>();
        float floorY = ground ? ground.bounds.max.y : Position.y - navigation.baseOffset;
        for (float z = bounds.min.z; z < bounds.max.z; z += SearchCellSize)
        for (float x = bounds.min.x; x < bounds.max.x; x += SearchCellSize)
        {
            var samples = new List<Vector3>();
            // Include multiple points so seeing the center alone cannot clear a cell.
            for (int row = 0; row < 3; row++)
            for (int column = 0; column < 3; column++)
            {
                Vector3 sample = new Vector3(x + (column + 0.5f) * SearchCellSize / 3f,
                    floorY, z + (row + 0.5f) * SearchCellSize / 3f);
                if (NavMesh.SamplePosition(sample, out NavMeshHit hit, 2f, filter) &&
                    Mathf.Abs(hit.position.y - floorY) <= 0.5f &&
                    hit.position.x >= x && hit.position.x < x + SearchCellSize &&
                    hit.position.z >= z && hit.position.z < z + SearchCellSize)
                    samples.Add(hit.position);
            }
            if (samples.Count > 0)
                surveyCells.Add(samples.ToArray());
        }
    }

    private bool CanSurvey(Vector3 point)
    {
        if (navigation == null || !navigation.isOnNavMesh)
            return false;

        Vector3 offset = point - Position;
        offset.y = 0f;
        return offset.sqrMagnitude <= ViewRange * ViewRange &&
            Vector3.Angle(transform.forward, offset) <= HalfViewAngle &&
            !navigation.Raycast(point, out _);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.DiscreteActions;
        actions[0] = 0;
#if ENABLE_LEGACY_INPUT_MANAGER
        if (seeker)
            return;
        bool secondPlayer = opponent && opponent.secondRunner == this;
        if (lockKeyPending) { actions[0] = ToggleCrateLock; lockKeyPending = false; return; }
        bool orbit = Input.GetKey(secondPlayer ? KeyCode.Y : KeyCode.T);
        bool grab = Input.GetKey(secondPlayer ? KeyCode.U : KeyCode.Q);

        Vector3 input = new Vector3(
            (Input.GetKey(secondPlayer ? KeyCode.L : KeyCode.D) ? 1f : 0f) -
            (Input.GetKey(secondPlayer ? KeyCode.J : KeyCode.A) ? 1f : 0f), 0f,
            (Input.GetKey(secondPlayer ? KeyCode.I : KeyCode.W) ? 1f : 0f) -
            (Input.GetKey(secondPlayer ? KeyCode.K : KeyCode.S) ? 1f : 0f)).normalized;
        if ((orbit || grab) && input == Vector3.zero)
            input = new Vector3(
                (Input.GetKey(KeyCode.RightArrow) ? 1f : 0f) - (Input.GetKey(KeyCode.LeftArrow) ? 1f : 0f), 0f,
                (Input.GetKey(KeyCode.UpArrow) ? 1f : 0f) - (Input.GetKey(KeyCode.DownArrow) ? 1f : 0f)).normalized;
        actions[0] = grab ? GrabCrate : 0;
        for (int i = 0; i < Directions.Length; i++)
            if (Vector3.Dot(input, Directions[i]) > 0.99f)
                actions[0] = orbit ? OrbitFirst + i : grab ? DirectionalCarryFirst + i : i + 1;
#endif
    }

    private void Update()
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        if (!seeker && Input.GetKeyDown(opponent && opponent.secondRunner == this ? KeyCode.B : KeyCode.V))
            lockKeyPending = true;
#endif
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int action = actions.DiscreteActions[0];
        if (!seeker && action == ToggleCrateLock && previousCrateAction != ToggleCrateLock)
            ToggleLock();
        if (action != lastCountedCrateAction)
        {
            lastCountedCrateAction = action;
            lastCountedCrate = null;
        }
        previousCrateAction = action;
        crateAction = !seeker && action >= GrabCrate && action < ActionCount && action != ToggleCrateLock ? action : 0;
        if (crateAction == 0)
            ReleaseCrate();
        direction = action >= 1 && action <= Directions.Length ? Directions[action - 1] : Vector3.zero;
    }

    public override void WriteDiscreteActionMask(IDiscreteActionMask mask)
    {
        bool canMoveCrate = TryGetCrateHeading(out _);
        bool canToggle = canMoveCrate && (FindOwnLock() || !lockMarks);
        canMoveCrate = canMoveCrate && !CrateLocks.ContainsKey(selectedCrate);
        mask.SetActionEnabled(0, GrabCrate, canMoveCrate);
        for (int i = 0; i < Directions.Length; i++)
            mask.SetActionEnabled(0, OrbitFirst + i, canMoveCrate);
        mask.SetActionEnabled(0, ToggleCrateLock, canToggle);
        for (int i = DirectionalCarryFirst; i < ActionCount; i++)
            mask.SetActionEnabled(0, i, canMoveCrate);
        for (int i = 0; i < Directions.Length; i++)
        {
            Vector3 heading = Directions[i];
            bool canMove = (!seeker || IsSearching) &&
                (Clearance(heading, 0.3f) > 0.04f ||
                 (seeker && FindPushableCrate(heading, SeekerPushReach, 0.04f)));
            mask.SetActionEnabled(0, i + 1, canMove);
        }
    }

    private void FixedUpdate()
    {
        if (!seeker || isRoundAssistant || opponent.navigation == null ||
            !navigation.isOnNavMesh || !opponent.navigation.isOnNavMesh)
            return;

        if (practicePassageStart && practiceLayoutTicks < 2)
        {
            if (practiceLayoutTicks == 0)
                PlacePracticeCrate();
            practiceLayoutTicks++;
            return;
        }

        {
            if (IsSearching && !preparationScored)
                ScorePreparation();
            else if (!IsSearching && !preparationBaselinesCaptured)
                CapturePreparationBaselines();
        }

        float step = Time.fixedDeltaTime;
        opponent.MoveRunner(step);
        if (secondRunner && secondRunner.navigation && secondRunner.navigation.isOnNavMesh)
            secondRunner.MoveRunner(step);
        if (IsSearching)
        {
            bool firstVisible = CanSeeOpponent();
            bool secondReady = secondSeeker && secondSeeker.navigation && secondSeeker.navigation.isOnNavMesh;
            bool secondVisible = secondReady && secondSeeker.CanSeeOpponent();
            if (firstVisible || secondVisible)
            {
                reportedPosition = (firstVisible ? visibleTarget : secondSeeker.visibleTarget).Position;
                reportedAt = elapsed;
            }

            bool caught = SearchStep(step, firstVisible, this);
            if (secondReady)
            {
                secondSeeker.elapsed = elapsed;
                caught |= secondSeeker.SearchStep(step, secondVisible, this);
            }

            if (!firstVisible && !secondVisible)
                UpdateSurvey();

            if (!caught)
            {
                opponent.UpdateCoverReward(this, secondReady ? secondSeeker : null, step);
                if (secondRunner && secondRunner.navigation && secondRunner.navigation.isOnNavMesh)
                    secondRunner.UpdateCoverReward(this, secondReady ? secondSeeker : null, step);
            }

            if (caught)
            {
                AddReward(1f);
                if (secondSeeker) secondSeeker.AddReward(1f);
                opponent.AddReward(-1f);
                if (secondRunner) secondRunner.AddReward(-1f);
                FinishRound(true);
                return;
            }
        }

        elapsed += step;
        if (elapsed + 0.001f >= PreparationSeconds + SearchSeconds)
            FinishRound(false);
    }

    private void PlacePracticeCrate()
    {
        BoxCollider crate = opponent.movableCrate;
        var left = GameObject.Find("MiddleWall")?.GetComponent<BoxCollider>();
        var right = GameObject.Find("MiddleWall (2)")?.GetComponent<BoxCollider>();
        if (!crate || !left || !right)
            return;

        Vector3 center = crate.bounds.center;
        Vector3 size = crate.bounds.extents;
        float gapCenter = (left.bounds.max.x + right.bounds.min.x) * 0.5f;
        float insideEdge = Mathf.Min(left.bounds.min.z, right.bounds.min.z);
        Transform box = crate.transform;
        Vector3 halfSize = Vector3.Scale(crate.size, box.lossyScale) * 0.5f - Vector3.one * 0.04f;
        for (int attempt = 0; attempt < 8; attempt++)
        {
            Vector3 candidate = new Vector3(gapCenter + Random.Range(0.25f, 0.75f), center.y,
                insideEdge - 3f - size.z - Random.Range(0.5f, 1f));
            bool occupied = false;
            foreach (Collider obstacle in Physics.OverlapBox(candidate, halfSize, box.rotation,
                         ~0, QueryTriggerInteraction.Ignore))
            {
                if (obstacle != crate && obstacle.gameObject.name != "Ground")
                {
                    occupied = true;
                    break;
                }
            }
            if (occupied)
                continue;

            box.position += candidate - center;
            Physics.SyncTransforms();
            return;
        }
    }

    private void CapturePreparationBaselines()
    {
        if (secondSeeker && (secondSeeker.navigation == null || !secondSeeker.navigation.isOnNavMesh))
            return;

        if (surveyedAt == null)
        {
            BuildSearchGrid();
            surveyedAt = new float[surveyCells.Count];
            System.Array.Fill(surveyedAt, float.NegativeInfinity);
        }

        passageBaselines[0] = CapturePassageBaseline("MiddleWall", "MiddleWall (2)", gapAxis: 0);
        passageBaselines[1] = CapturePassageBaseline("MiddleWall (3)", "MiddleWall (4)", gapAxis: 2);
        preparationBaselinesCaptured = true;
    }

    private void ScorePreparation()
    {
        preparationScored = true;
        if (!preparationBaselinesCaptured)
            return;

        bool crateMoved = opponent.movedCrateDuringPreparation ||
            (secondRunner && secondRunner.movedCrateDuringPreparation);
        int lockedPassages = MeasurePassages(crateMoved, out float partialReward);
        const float rewardPerPassage = 0.2f;
        const float rewardPerLockedPassage = 0.1f;
        int rewardedPassages = crateMoved ? passagesBlocked : 0;
        int rewardedLocks = crateMoved ? lockedPassages : 0;
        float reward = rewardedPassages * rewardPerPassage + rewardedLocks * rewardPerLockedPassage + partialReward;
        if (reward > 0f)
        {
            opponent.AddReward(reward);
            if (secondRunner)
                secondRunner.AddReward(reward);
        }

        if (Academy.Instance.IsCommunicatorOn)
        {
            var stats = Academy.Instance.StatsRecorder;
            stats.Add("HideSeek/Preparation Block Bonus", rewardedPassages);
            stats.Add("HideSeek/Preparation Locked Block Bonus", rewardedLocks);
            stats.Add("HideSeek/Preparation Partial Block Reward", partialReward);
        }
        Debug.Log($"[준비 수색] 팀 차단 가점 {rewardedPassages}회 | 고정 가점 {rewardedLocks}회 | 틈 줄이기 +{partialReward:F3} | Runner당 +{reward:F2}", this);
    }

    private (Vector3 from, Vector3 to, float length, float startGap, BoxCollider firstWall, BoxCollider secondWall)? CapturePassageBaseline(
        string firstWallName, string secondWallName, int gapAxis)
    {
        var firstWall = GameObject.Find(firstWallName)?.GetComponent<BoxCollider>();
        var secondWall = GameObject.Find(secondWallName)?.GetComponent<BoxCollider>();
        if (!firstWall || !secondWall)
            return null;

        Bounds lower = firstWall.bounds;
        Bounds upper = secondWall.bounds;
        if (lower.center[gapAxis] > upper.center[gapAxis])
            (lower, upper) = (upper, lower);
        if (lower.max[gapAxis] >= upper.min[gapAxis])
            return null;

        int travelAxis = 2 - gapAxis;
        float gapCenter = (lower.max[gapAxis] + upper.min[gapAxis]) * 0.5f;
        float floor = Position.y - navigation.baseOffset;
        var filter = new NavMeshQueryFilter
        {
            agentTypeID = navigation.agentTypeID,
            areaMask = navigation.areaMask
        };
        Vector3 fromPoint = new Vector3(0f, floor, 0f);
        fromPoint[gapAxis] = gapCenter;
        fromPoint[travelAxis] = Mathf.Min(lower.min[travelAxis], upper.min[travelAxis]) - 3f;
        Vector3 toPoint = fromPoint;
        toPoint[travelAxis] = Mathf.Max(lower.max[travelAxis], upper.max[travelAxis]) + 3f;
        if (!NavMesh.SamplePosition(fromPoint, out NavMeshHit from, 1.5f, filter) ||
            !NavMesh.SamplePosition(toPoint, out NavMeshHit to, 1.5f, filter))
            return null;

        float length = PassageLength(from.position, to.position);
        if (length > Vector3.Distance(from.position, to.position) + 1f)
            return null;
        return (from.position, to.position, length, PassageGap(firstWall, secondWall), firstWall, secondWall);
    }

    private float PassageLength(Vector3 from, Vector3 to)
    {
        var filter = new NavMeshQueryFilter
        {
            agentTypeID = navigation.agentTypeID,
            areaMask = navigation.areaMask
        };
        if (!NavMesh.CalculatePath(from, to, filter, trainingPath) ||
            trainingPath.status != NavMeshPathStatus.PathComplete)
            return float.PositiveInfinity;

        float length = 0f;
        Vector3 previous = from;
        foreach (Vector3 corner in trainingPath.corners)
        {
            length += Vector3.Distance(previous, corner);
            previous = corner;
        }
        return length;
    }

    private int MeasurePassages(bool restrictRewardedCrates, out float partialReward)
    {
        partialReward = 0f;
        int lockedPassages = 0;
        for (int i = 0; i < passageBaselines.Length; i++)
        {
            var baseline = passageBaselines[i];
            if (!baseline.HasValue)
            {
                Debug.Log($"[준비 수색] 통로 {i + 1} | 미측정", this);
                continue;
            }

            var (from, to, length, startGap, firstWall, secondWall) = baseline.Value;
            float current = PassageLength(from, to);
            bool locked = CratesSealPassage(firstWall, secondWall, requireLocked: true, restrictRewardedCrates);
            bool blocked = locked ||
                CratesSealPassage(firstWall, secondWall, requireLocked: false, restrictRewardedCrates);
            passagesMeasured++;
            if (blocked)
                passagesBlocked++;
            else if (restrictRewardedCrates)
            {
                float improvement = startGap - PassageGap(firstWall, secondWall);
                if (improvement >= 0.25f)
                    partialReward += Mathf.Min(0.04f, improvement * 0.02f);
            }
            if (locked)
                lockedPassages++;
            string status = float.IsInfinity(current) ? "통과 불가" : $"{current:F1}유닛";
            Debug.Log($"[준비 수색] 통로 {i + 1} | {length:F1}유닛 → {status} | 차단 {(blocked ? 1 : 0)}회", this);
        }
        return lockedPassages;
    }

    private float PassageGap(BoxCollider firstWall, BoxCollider secondWall)
    {
        Vector2[] first = BoxFootprint(firstWall);
        Vector2[] second = BoxFootprint(secondWall);
        float best = FootprintGapSquared(first, second);
        BoxCollider a = opponent.movableCrate;
        BoxCollider b = opponent.secondCrate;
        Vector2[] aFootprint = a && a.enabled && a.gameObject.activeInHierarchy ? BoxFootprint(a) : null;
        Vector2[] bFootprint = b && b.enabled && b.gameObject.activeInHierarchy ? BoxFootprint(b) : null;
        float firstA = aFootprint != null ? FootprintGapSquared(first, aFootprint) : float.PositiveInfinity;
        float aSecond = aFootprint != null ? FootprintGapSquared(aFootprint, second) : float.PositiveInfinity;
        float firstB = bFootprint != null ? FootprintGapSquared(first, bFootprint) : float.PositiveInfinity;
        float bSecond = bFootprint != null ? FootprintGapSquared(bFootprint, second) : float.PositiveInfinity;
        best = Mathf.Min(best, Mathf.Max(firstA, aSecond), Mathf.Max(firstB, bSecond));
        if (aFootprint != null && bFootprint != null)
        {
            float between = FootprintGapSquared(aFootprint, bFootprint);
            best = Mathf.Min(best, Mathf.Max(firstA, between, bSecond), Mathf.Max(firstB, between, aSecond));
        }
        return Mathf.Sqrt(best);
    }

    private bool CratesSealPassage(BoxCollider firstWall, BoxCollider secondWall,
        bool requireLocked, bool restrictRewardedCrates)
    {
        float diameter = 2f * (secondSeeker && secondSeeker.navigation
            ? Mathf.Min(navigation.radius, secondSeeker.navigation.radius)
            : navigation.radius);
        float clearanceSquared = diameter * diameter;
        Vector2[] end = BoxFootprint(secondWall);
        var reachable = new List<(Vector2[] footprint, BoxCollider box, int previous)>
        {
            (BoxFootprint(firstWall), null, -1)
        };
        var remaining = new List<(Vector2[] footprint, BoxCollider box)>();
        foreach (BoxCollider crate in new[] { opponent.movableCrate, opponent.secondCrate })
        {
            if (!crate || !crate.enabled || !crate.gameObject.activeInHierarchy)
                continue;
            if (requireLocked && (!CrateLocks.TryGetValue(crate, out var owner) ||
                (owner != opponent && owner != secondRunner)))
                continue;

            remaining.Add((BoxFootprint(crate), crate));
        }

        // Boxes form a barrier when they link the two walls with no Chaser-width gaps.
        for (int i = 0; i < reachable.Count; i++)
        {
            Vector2[] edge = reachable[i].footprint;
            for (int j = remaining.Count - 1; j >= 0; j--)
            {
                var crate = remaining[j];
                if (FootprintGapSquared(edge, crate.footprint) >= clearanceSquared)
                    continue;

                if (FootprintGapSquared(crate.footprint, end) < clearanceSquared)
                {
                    if (restrictRewardedCrates)
                    {
                        rewardedBarrierCrates.Add(crate.box);
                        for (int link = i; link > 0; link = reachable[link].previous)
                            rewardedBarrierCrates.Add(reachable[link].box);
                    }
                    return true;
                }

                remaining.RemoveAt(j);
                reachable.Add((crate.footprint, crate.box, i));
            }
        }

        return false;
    }

    private static Vector2[] BoxFootprint(BoxCollider box)
    {
        Vector3 x = box.transform.TransformVector(Vector3.right * (box.size.x * 0.5f));
        Vector3 z = box.transform.TransformVector(Vector3.forward * (box.size.z * 0.5f));
        Vector3 center = box.transform.TransformPoint(box.center);
        var c = new Vector2(center.x, center.z);
        var side = new Vector2(x.x, x.z);
        var depth = new Vector2(z.x, z.z);
        return new[] { c - side - depth, c + side - depth, c + side + depth, c - side + depth };
    }

    private static float FootprintGapSquared(Vector2[] first, Vector2[] second)
    {
        bool separated = false;
        float distance = float.PositiveInfinity;
        for (int i = 0; i < 4; i++)
        {
            int next = (i + 1) % 4;
            Vector2 firstEdge = first[next] - first[i];
            Vector2 secondEdge = second[next] - second[i];
            separated |= IsSeparatingAxis(firstEdge, first, second) ||
                IsSeparatingAxis(secondEdge, first, second);
            for (int j = 0; j < 4; j++)
            {
                distance = Mathf.Min(distance, PointEdgeGapSquared(second[j], first[i], firstEdge));
                distance = Mathf.Min(distance, PointEdgeGapSquared(first[j], second[i], secondEdge));
            }
        }
        return separated ? distance : 0f;
    }

    private static bool IsSeparatingAxis(Vector2 edge, Vector2[] first, Vector2[] second)
    {
        var normal = new Vector2(-edge.y, edge.x);
        float firstMin = float.PositiveInfinity, secondMin = float.PositiveInfinity;
        float firstMax = float.NegativeInfinity, secondMax = float.NegativeInfinity;
        for (int i = 0; i < 4; i++)
        {
            float a = Vector2.Dot(first[i], normal);
            float b = Vector2.Dot(second[i], normal);
            firstMin = Mathf.Min(firstMin, a);
            firstMax = Mathf.Max(firstMax, a);
            secondMin = Mathf.Min(secondMin, b);
            secondMax = Mathf.Max(secondMax, b);
        }
        return firstMax < secondMin || secondMax < firstMin;
    }

    private static float PointEdgeGapSquared(Vector2 point, Vector2 start, Vector2 edge)
    {
        float t = Mathf.Clamp01(Vector2.Dot(point - start, edge) / edge.sqrMagnitude);
        return (point - (start + t * edge)).sqrMagnitude;
    }

    private bool SearchStep(float step, bool visible, HideSeekAgent leader)
    {
        navigation.speed = visible ? baseSpeed * ChaseSpeedMultiplier : baseSpeed;
        if (visible)
        {
            StopPatrol();
            nextResponsePath = 0f;
            Move(step, HeadingTo(visibleTarget.Position), true);
        }
        else if (elapsed - leader.reportedAt <= 6f)
        {
            StopPatrol();
            lastSeen = leader.reportedPosition;
            lastSeenAt = leader.reportedAt;
            hasMemory = true;
            RespondToSighting(step, lastSeen);
        }
        else
        {
            nextResponsePath = 0f;
            HideSeekAgent teammate = isRoundAssistant ? leader : leader.secondSeeker;
            PatrolUnsearchedCells(leader, teammate);
            if (teammate && teammate.navigation && teammate.navigation.isOnNavMesh)
            {
                const float separationDistance = 5f;
                float separation = Vector3.Distance(Position, teammate.Position);
                AddReward(-0.02f * step * (1f - Mathf.Clamp01(separation / separationDistance)));
            }
        }
        Vector3 position = Position;
        travelDistance += Vector3.Distance(previousPosition, position);
        previousPosition = position;
        var cell = new Vector2Int(Mathf.FloorToInt(position.x / 4f), Mathf.FloorToInt(position.z / 4f));
        if (leader.searchedCells.Add(cell))
            AddReward(0.035f);

        if (visible && !wasVisible)
        {
            if (discoveries == 0)
                AddReward(0.65f);
            discoveries++;
            rediscovered |= hiddenDuringSearch;
        }
        hiddenDuringSearch |= !visible;
        wasVisible = visible;

        if (visible)
            visibleSeconds += step;
        float reward = (visible ? 1f : -1f) * step / SearchSeconds;
        AddReward(reward);
        opponent.AddReward(-reward);
        if (leader.secondRunner) leader.secondRunner.AddReward(-reward);

        return CanTag(opponent) || CanTag(leader.secondRunner);
    }

    private void PatrolUnsearchedCells(HideSeekAgent leader, HideSeekAgent teammate)
    {
        if (patrolCell >= 0)
        {
            if ((Position - patrolProgressPosition).sqrMagnitude >= 0.04f)
            {
                patrolProgressPosition = Position;
                patrolProgressAt = elapsed;
            }

            bool arrived = !navigation.pathPending && navigation.remainingDistance <=
                Mathf.Max(navigation.stoppingDistance, 0.25f);
            bool blocked = !navigation.pathPending &&
                (!navigation.hasPath || navigation.pathStatus != NavMeshPathStatus.PathComplete);
            if (!arrived && !blocked && elapsed - patrolProgressAt < 2f)
                return;

            patrolRetryAt[patrolCell] = elapsed + (arrived ? RecheckSeconds : 4f);
            StopPatrol();
        }

        if (elapsed < nextPatrolAttempt || leader.surveyedAt == null)
            return;

        nextPatrolAttempt = elapsed + 0.5f;
        var candidates = new List<(int cell, float score)>();
        int teammateCell = teammate ? teammate.patrolCell : -1;
        for (int i = 0; i < leader.surveyCells.Count; i++)
        {
            if (i == teammateCell || (patrolRetryAt.TryGetValue(i, out float retryAt) && elapsed < retryAt))
                continue;

            Vector3 point = leader.surveyCells[i][0];
            float score = Vector3.Distance(Position, point);
            if (teammate && teammate.navigation && teammate.navigation.isOnNavMesh &&
                Vector3.Distance(teammate.Position, point) < score)
                score += 10f;
            if (elapsed - leader.surveyedAt[i] < RecheckSeconds)
                score += 100f;
            candidates.Add((i, score));
        }

        candidates.Sort((a, b) => a.score.CompareTo(b.score));
        foreach (var candidate in candidates)
        {
            foreach (Vector3 point in leader.surveyCells[candidate.cell])
            {
                Vector3 offset = point - Position;
                offset.y = 0f;
                if (offset.sqrMagnitude <= 0.64f ||
                    !navigation.CalculatePath(point, trainingPath) ||
                    trainingPath.status != NavMeshPathStatus.PathComplete ||
                    !navigation.SetPath(trainingPath))
                    continue;

                patrolCell = candidate.cell;
                patrolProgressPosition = Position;
                patrolProgressAt = elapsed;
                return;
            }
        }
    }

    private void StopPatrol()
    {
        if (patrolCell < 0)
            return;

        patrolCell = -1;
        nextPatrolAttempt = 0f;
        navigation.ResetPath();
    }

    private bool CanTag(HideSeekAgent target)
    {
        return target && target.navigation && target.navigation.isOnNavMesh &&
            (target.Position - Position).sqrMagnitude <= TagDistance * TagDistance &&
            !navigation.Raycast(target.Position, out _);
    }

    private void CreateBeam()
    {
        var beam = new GameObject("Chaser 시야");
        beam.transform.SetParent(transform, false);
        beamMesh = new Mesh { name = "Chaser 시야 메시" };
        beamMesh.MarkDynamic();
        beamMesh.vertices = beamVertices;
        var triangles = new int[BeamSegments * 3];
        for (int i = 0; i < BeamSegments; i++)
        {
            triangles[i * 3 + 1] = i + 1;
            triangles[i * 3 + 2] = i + 2;
        }
        beamMesh.triangles = triangles;
        beam.AddComponent<MeshFilter>().sharedMesh = beamMesh;
        beamRenderer = beam.AddComponent<MeshRenderer>();
        beamMaterial = new Material(Shader.Find("Sprites/Default"))
        {
            color = new Color(1f, 0.9f, 0.35f, 0.25f)
        };
        beamRenderer.sharedMaterial = beamMaterial;
    }

    private void LateUpdate()
    {
        if (!beamRenderer)
            return;

        beamRenderer.enabled = IsSearching && navigation.isOnNavMesh;
        if (!beamRenderer.enabled)
            return;

        Vector3 origin = Position;
        Vector3 lift = Vector3.up * 0.08f;
        beamVertices[0] = transform.InverseTransformPoint(origin + lift);
        for (int i = 0; i <= BeamSegments; i++)
        {
            float angle = Mathf.Lerp(-HalfViewAngle, HalfViewAngle, i / (float)BeamSegments);
            Vector3 destination = origin + Quaternion.AngleAxis(angle, Vector3.up) * transform.forward * ViewRange;
            if (navigation.Raycast(destination, out NavMeshHit hit))
                destination = hit.position;
            beamVertices[i + 1] = transform.InverseTransformPoint(destination + lift);
        }
        beamMesh.vertices = beamVertices;
        beamMesh.RecalculateBounds();
    }

    private void OnDestroy()
    {
        if (beamMesh) Destroy(beamMesh);
        if (beamMaterial) Destroy(beamMaterial);
        ReleaseCrate();
        ClearCrateLock(movableCrate);
        ClearCrateLock(secondCrate);
        if (lockMaterial) Destroy(lockMaterial);
    }

    private bool TryGetCrateHeading(out Vector3 heading)
    {
        heading = Vector3.zero;
        selectedCrate = null;
        if (seeker || navigation == null || !navigation.isOnNavMesh)
            return false;

        Vector3 first = CrateOffset(movableCrate);
        Vector3 second = CrateOffset(secondCrate);
        bool useSecond = second != Vector3.zero &&
            (first == Vector3.zero || second.sqrMagnitude < first.sqrMagnitude);
        Vector3 offset = useSecond ? second : first;
        if (offset == Vector3.zero)
            return false;
        selectedCrate = useSecond ? secondCrate : movableCrate;
        heading = offset.normalized;
        return true;
    }

    private Vector3 CrateOffset(BoxCollider box)
    {
        if (!box || !box.enabled || !box.gameObject.activeInHierarchy ||
            (opponent && opponent.rewardedBarrierCrates.Contains(box)) ||
            (CrateHandlers.TryGetValue(box, out var handler) && handler && handler != this) ||
            (CrateLocks.TryGetValue(box, out var owner) && owner && owner != this))
            return Vector3.zero;

        Vector3 nearest = box.ClosestPoint(new Vector3(Position.x, box.bounds.center.y, Position.z));
        Vector3 offset = nearest - Position;
        offset.y = 0f;
        float distance = offset.magnitude;
        if (distance < 0.01f || distance > CrateReach ||
            navigation.Raycast(Position + offset.normalized * Mathf.Max(0f, distance - 0.7f), out _))
            return Vector3.zero;
        return offset;
    }

    private void MoveRunner(float step)
    {
        if (crateAction == 0)
        {
            Move(step);
            return;
        }

        if (!TryGetCrateHeading(out Vector3 toward) || CrateLocks.ContainsKey(selectedCrate))
        {
            ReleaseCrate();
            return;
        }

        if (heldCrate != selectedCrate)
        {
            ReleaseCrate();
            heldCrate = selectedCrate;
            CrateHandlers[heldCrate] = this;
        }

        Transform box = heldCrate.transform;
        Vector3 before = box.position;
        Quaternion rotation = box.rotation;
        MoveCrate(step, toward);
        bool moved = before != box.position || rotation != box.rotation;
        if (moved && !IsSearching)
        {
            movedCrateDuringPreparation = true;
            if (lastCountedCrate != heldCrate)
            {
                preparationCrateMoves++;
                lastCountedCrate = heldCrate;
            }
        }
        if (moved && movedCover != heldCrate)
        {
            movedCover = heldCrate;
            coverSeconds = 0f;
        }
    }

    private void ReleaseCrate()
    {
        if (heldCrate && CrateHandlers.TryGetValue(heldCrate, out var owner) && owner == this)
            CrateHandlers.Remove(heldCrate);
        heldCrate = null;
    }

    private BoxCollider FindOwnLock()
    {
        if (!TryGetCrateHeading(out _))
            return null;
        return CrateLocks.TryGetValue(selectedCrate, out var owner) && owner == this
            ? selectedCrate : null;
    }

    private void ToggleLock()
    {
        if (!TryGetCrateHeading(out _))
            return;
        if (CrateLocks.TryGetValue(selectedCrate, out var owner))
        {
            if (owner == this)
                ClearCrateLock(selectedCrate);
            return;
        }

        if (lockMarks)
            return;

        CrateLocks[selectedCrate] = this;
        ReleaseCrate();
        ShowLockMarks(selectedCrate);
    }

    private void ClearCrateLock(BoxCollider crate)
    {
        if (!crate || !CrateLocks.TryGetValue(crate, out var owner) || owner != this)
            return;
        CrateLocks.Remove(crate);
        if (lockMarks)
        {
            lockMarks.SetActive(false);
            Destroy(lockMarks);
        }
        lockMarks = null;
    }

    private void ShowLockMarks(BoxCollider crate)
    {
        lockMarks = new GameObject("상자 고정 표시");
        lockMarks.transform.SetParent(crate.transform, false);
        Material runnerMaterial = GetComponentInChildren<Renderer>()?.sharedMaterial;
        Color color = runnerMaterial && runnerMaterial.HasProperty("_BaseColor")
            ? runnerMaterial.GetColor("_BaseColor")
            : runnerMaterial && runnerMaterial.HasProperty("_Color") ? runnerMaterial.color : Color.white;
        if (!lockMaterial)
            lockMaterial = new Material(Shader.Find("Sprites/Default"));
        Vector3 half = crate.size * 0.5f;
        foreach (Vector3 normal in new[] { Vector3.right, Vector3.left, Vector3.up, Vector3.down,
                     Vector3.forward, Vector3.back })
        {
            Vector3 right = normal.y == 0 ? Vector3.Cross(Vector3.up, normal) : Vector3.right;
            Vector3 up = Vector3.Cross(normal, right);
            Vector3 center = crate.center + Vector3.Scale(normal, half + Vector3.one * 0.005f);
            Vector3 a = Vector3.Scale(right, half) * 0.65f;
            Vector3 b = Vector3.Scale(up, half) * 0.65f;
            foreach (Vector3 diagonal in new[] { a + b, a - b })
            {
                var stroke = new GameObject("X").AddComponent<LineRenderer>();
                stroke.transform.SetParent(lockMarks.transform, false);
                stroke.useWorldSpace = false;
                stroke.sharedMaterial = lockMaterial;
                stroke.startColor = stroke.endColor = color;
                stroke.widthMultiplier = 0.12f;
                stroke.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                stroke.positionCount = 2;
                stroke.SetPosition(0, center - diagonal);
                stroke.SetPosition(1, center + diagonal);
            }
        }
    }

    private void UpdateCoverReward(HideSeekAgent first, HideSeekAgent second, float step)
    {
        if (coverRewarded || !movedCover || !movedCover.enabled)
            return;

        bool hidden = !first.CanSee(this) && (!second || !second.CanSee(this));
        bool usingCover = hidden && (BlocksViewFrom(first) || (second && BlocksViewFrom(second)));
        coverSeconds = usingCover ? coverSeconds + step : 0f;
        if (coverSeconds < 2f)
            return;

        coverRewarded = true;
        AddReward(0.05f);
        Academy.Instance.StatsRecorder.Add("HideSeek/Crate Cover Bonus", 1f);
        Debug.Log($"[숨바꼭질] {name}가 옮긴 상자 뒤에서 숨기에 성공했어요.", this);
    }

    private bool BlocksViewFrom(HideSeekAgent chaser)
    {
        Vector3 offset = Position - chaser.Position;
        float distance = offset.magnitude;
        if (distance <= TagDistance || distance > ViewRange ||
            Vector3.Angle(chaser.transform.forward, offset) > HalfViewAngle)
            return false;

        // Start above the floor and ignore the seeking capsule at the ray origin.
        Vector3 origin = chaser.Position + Vector3.up * 0.2f;
        var ray = new Ray(origin, offset.normalized);
        if (!movedCover.Raycast(ray, out RaycastHit coverHit, distance))
            return false;

        foreach (RaycastHit hit in Physics.RaycastAll(ray, coverHit.distance, ~0, QueryTriggerInteraction.Ignore))
        {
            Transform obstacle = hit.collider.transform;
            if (hit.collider != movedCover && !obstacle.IsChildOf(chaser.transform) &&
                hit.distance < coverHit.distance - 0.01f)
                return false;
        }
        return true;
    }

    private void RotateCrate(float degrees)
    {
        Transform box = selectedCrate.transform;
        Quaternion rotation = Quaternion.AngleAxis(degrees, Vector3.up) * box.rotation;
        Vector3 pivot = Position;
        pivot.y = box.position.y;
        Vector3 nextPosition = pivot + Quaternion.AngleAxis(degrees, Vector3.up) * (box.position - pivot);
        Vector3 center = nextPosition + rotation * Vector3.Scale(selectedCrate.center, box.lossyScale);
        Vector3 half = Vector3.Scale(selectedCrate.size, box.lossyScale) * 0.5f - Vector3.one * 0.04f;
        foreach (Collider hit in Physics.OverlapBox(center, half, rotation, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit == selectedCrate || hit.gameObject.name == "Ground")
                continue;

            Transform obstacle = hit.transform;
            if (!Physics.ComputePenetration(selectedCrate, nextPosition, rotation,
                hit, obstacle.position, obstacle.rotation, out _, out float nextDepth))
                continue;

            Physics.ComputePenetration(selectedCrate, box.position, box.rotation,
                hit, obstacle.position, obstacle.rotation, out _, out float currentDepth);
            if (nextDepth > currentDepth + 0.001f)
                return;
        }

        box.SetPositionAndRotation(nextPosition, rotation);
        Physics.SyncTransforms();
    }

    private void MoveCrate(float seconds, Vector3 toward)
    {

        if (crateAction >= DirectionalCarryFirst)
        {
            TranslateCrate(seconds, toward, Directions[crateAction - DirectionalCarryFirst]);
            return;
        }

        if (crateAction >= OrbitFirst)
        {
            Vector3 radial = selectedCrate.bounds.center - Position;
            radial.y = 0f;
            float angle = Vector3.SignedAngle(radial, Directions[crateAction - OrbitFirst], Vector3.up);
            float radius = Mathf.Max(radial.magnitude, 0.1f);
            float maxAngle = navigation.speed * seconds / radius * Mathf.Rad2Deg;
            RotateCrate(Mathf.Clamp(angle, -maxAngle, maxAngle));
        }
    }

    private void TranslateCrate(float seconds, Vector3 toward, Vector3 heading)
    {
        float step = navigation.speed * seconds;
        // When moving toward the crate, its carve can lag one physics frame.
        // Sideways/backward movement still requires the Runner's full step to be clear.
        float runnerStep = Clearance(heading, step);
        if (runnerStep < step - 0.001f && Vector3.Dot(heading, toward) < 0.5f)
            return;

        if (!CanSlideCrate(selectedCrate, heading, step))
            return;

        navigation.Move(heading * runnerStep);
        selectedCrate.transform.position += heading * step;
        Physics.SyncTransforms();
        transform.rotation = Quaternion.LookRotation(toward);
    }

    private bool CanSlideCrate(BoxCollider crate, Vector3 heading, float distance)
    {
        Transform box = crate.transform;
        Vector3 extents = Vector3.Scale(crate.size, box.lossyScale) * 0.5f - Vector3.one * 0.04f;
        foreach (RaycastHit hit in Physics.BoxCastAll(box.TransformPoint(crate.center), extents, heading,
                     box.rotation, distance + 0.04f, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit.collider != crate && !hit.collider.transform.IsChildOf(transform) &&
                hit.collider.gameObject.name != "Ground")
                return false;
        }
        return true;
    }

    private BoxCollider FindPushableCrate(Vector3 heading, float reach, float pushDistance)
    {
        if (heading == Vector3.zero || !opponent || !navigation.isOnNavMesh)
            return null;

        Collider nearest = null;
        float nearestDistance = float.PositiveInfinity;
        Vector3 origin = Position + Vector3.up * 0.3f;
        foreach (RaycastHit hit in Physics.SphereCastAll(origin, navigation.radius, heading,
                     reach, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit.distance >= nearestDistance || hit.collider.transform.IsChildOf(transform) ||
                hit.collider.gameObject.name == "Ground")
                continue;
            nearest = hit.collider;
            nearestDistance = hit.distance;
        }

        var crate = nearest as BoxCollider;
        return crate && (crate == opponent.movableCrate || crate == opponent.secondCrate) &&
            !CrateLocks.ContainsKey(crate) && CanSlideCrate(crate, heading, pushDistance) ? crate : null;
    }

    private bool TryPushCrate(Vector3 heading, float seconds)
    {
        float distance = SeekerPushSpeed * seconds;
        BoxCollider crate = FindPushableCrate(heading, SeekerPushReach, distance);
        if (!crate)
            return false;

        crate.transform.position += heading * distance;
        Physics.SyncTransforms();
        transform.rotation = Quaternion.LookRotation(heading);
        navigation.Move(heading * Clearance(heading, distance));
        return true;
    }

    private Vector3 HeadingTo(Vector3 point)
    {
        Vector3 offset = point - Position;
        offset.y = 0f;
        return offset.normalized;
    }

    private void RespondToSighting(float step, Vector3 target)
    {
        Vector3 heading = HeadingTo(target);
        if (TryPushCrate(heading, step))
        {
            nextResponsePath = 0f;
            return;
        }

        if (elapsed >= nextResponsePath)
        {
            nextResponsePath = elapsed + 0.35f;
            responseCorners = navigation.CalculatePath(target, responsePath) &&
                responsePath.status == NavMeshPathStatus.PathComplete
                ? responsePath.corners : System.Array.Empty<Vector3>();
            responseCorner = 0;
        }

        while (responseCorner < responseCorners.Length)
        {
            Vector3 corner = responseCorners[responseCorner];
            Vector3 offset = corner - Position;
            offset.y = 0f;
            if (offset.sqrMagnitude > 0.09f)
            {
                Move(step, HeadingTo(corner), true);
                return;
            }
            responseCorner++;
        }

        Move(step);
    }

    private void Move(float seconds) => Move(seconds, direction, false);

    private void Move(float seconds, Vector3 heading, bool faceBeforeMoving)
    {
        if (heading == Vector3.zero)
            return;

        if (faceBeforeMoving)
            transform.rotation = Quaternion.LookRotation(heading);

        float requested = seconds * navigation.speed;
        float distance = Clearance(heading, requested);
        if (seeker && distance < requested && TryPushCrate(heading, seconds))
            return;
        if (distance <= 0f)
            return;

        navigation.Move(heading * distance);
        if (!faceBeforeMoving)
            transform.rotation = Quaternion.LookRotation(heading);
    }

    private float Clearance(Vector3 heading, float requested)
    {
        if (!navigation.isOnNavMesh)
            return 0f;
        return navigation.Raycast(Position + heading * requested, out NavMeshHit hit)
            ? Mathf.Clamp(Vector3.Dot(hit.position - Position, heading) - 0.03f, 0f, requested)
            : requested;
    }

    private void FinishRound(bool tagged)
    {
        var stats = Academy.Instance.StatsRecorder;
        stats.Add("HideSeek/Visible Fraction", visibleSeconds / SearchSeconds);
        stats.Add("HideSeek/Discovered", discoveries > 0 ? 1f : 0f);
        stats.Add("HideSeek/Rediscovered", rediscovered ? 1f : 0f);
        string result = tagged ? "러너가 잡혔어요" : "수색 시간이 끝났어요";
        Debug.Log($"[숨바꼭질] {result} | 술래 1 발견 {discoveries}회 | 술래 2 발견 {(secondSeeker ? secondSeeker.discoveries : 0)}회 | 시야에 들어온 시간 {visibleSeconds:F2}초", this);
        int moves = opponent.preparationCrateMoves + (secondRunner ? secondRunner.preparationCrateMoves : 0);
        stats.Add("HideSeek/Preparation Crate Operations", moves);
        if (passagesMeasured > 0)
            stats.Add("HideSeek/Preparation Passage Blocked", passagesBlocked);
        string passageSummary = passagesMeasured > 0 ? $"{passagesBlocked}회 (측정 {passagesMeasured}곳)" : "미측정";
        Debug.Log($"[준비 수색] 상자 조작 {moves}회 | 통로 차단 {passageSummary}", this);
        EndEpisode();
        opponent.EndEpisode();
        if (secondRunner) secondRunner.EndEpisode();
        if (secondSeeker) secondSeeker.EndEpisode();
        if (opponent.movableCrate)
        {
            opponent.movableCrate.transform.SetPositionAndRotation(opponent.crateStart, opponent.crateStartRotation);
        }
        if (opponent.secondCrate)
            opponent.secondCrate.transform.SetPositionAndRotation(opponent.secondStart, opponent.secondRotation);
        Physics.SyncTransforms();
        opponent.ResetRound();
        if (secondRunner) secondRunner.ResetRound();

        bool pairedSpawn = secondSeeker && TryPlaceSeekerPair();
        ResetRound(keepPosition: pairedSpawn, jitter: !secondSeeker);
        if (secondSeeker)
            secondSeeker.ResetRound(keepPosition: pairedSpawn, jitter: false);
    }

    private bool TryPlaceSeekerPair()
    {
        var ground = GameObject.Find("Ground")?.GetComponent<Collider>();
        var northWall = GameObject.Find("MiddleWall")?.GetComponent<BoxCollider>();
        var westWall = GameObject.Find("MiddleWall (3)")?.GetComponent<BoxCollider>();
        var arenaWest = GameObject.Find("Wall4")?.GetComponent<Collider>();
        var arenaEast = GameObject.Find("Wall3")?.GetComponent<Collider>();
        var arenaSouth = GameObject.Find("Wall1")?.GetComponent<Collider>();
        var arenaNorth = GameObject.Find("Wall2")?.GetComponent<Collider>();
        if (!ground || !northWall || !westWall || !arenaWest || !arenaEast ||
            !arenaSouth || !arenaNorth || !secondSeeker.navigation)
            return false;

        Bounds arena = ground.bounds;
        Vector3 min = arena.min;
        Vector3 max = arena.max;
        min.x = Mathf.Max(min.x, arenaWest.bounds.max.x);
        max.x = Mathf.Min(max.x, arenaEast.bounds.min.x);
        min.z = Mathf.Max(min.z, arenaSouth.bounds.max.z);
        max.z = Mathf.Min(max.z, arenaNorth.bounds.min.z);
        if (min.x >= max.x || min.z >= max.z)
            return false;
        arena.SetMinMax(min, max);
        float runnerZoneMinX = westWall.bounds.center.x;
        float runnerZoneMaxZ = northWall.bounds.center.z;
        float spacing = Vector2.Distance(new Vector2(start.x, start.z),
            new Vector2(secondSeeker.start.x, secondSeeker.start.z));
        float firstFloor = start.y - navigation.baseOffset;
        float secondFloor = secondSeeker.start.y - secondSeeker.navigation.baseOffset;
        Vector3 previousFirst = Position;

        for (int attempt = 0; attempt < 96; attempt++)
        {
            Vector3 first = new Vector3(Random.Range(arena.min.x, arena.max.x), firstFloor,
                Random.Range(arena.min.z, arena.max.z));
            float angle = Random.Range(0f, 2f * Mathf.PI);
            Vector3 second = new Vector3(first.x + spacing * Mathf.Cos(angle), secondFloor,
                first.z + spacing * Mathf.Sin(angle));

            if (!TrySampleSeekerSpawn(first, navigation, arena, runnerZoneMinX, runnerZoneMaxZ, out Vector3 firstSpawn) ||
                !TrySampleSeekerSpawn(second, secondSeeker.navigation, arena, runnerZoneMinX, runnerZoneMaxZ, out Vector3 secondSpawn) ||
                !navigation.Warp(firstSpawn))
                continue;

            if (secondSeeker.navigation.Warp(secondSpawn))
                return true;

            navigation.Warp(previousFirst);
        }
        return false;
    }

    private static bool TrySampleSeekerSpawn(Vector3 point, NavMeshAgent agent, Bounds arena,
        float runnerZoneMinX, float runnerZoneMaxZ, out Vector3 spawnPoint)
    {
        spawnPoint = default;
        var body = agent.GetComponent<CapsuleCollider>();
        float bodyRadius = body ? Mathf.Max(body.bounds.extents.x, body.bounds.extents.z) : agent.radius;
        float clearance = Mathf.Max(bodyRadius, agent.radius) + 0.2f;
        if (point.x < arena.min.x + clearance || point.x > arena.max.x - clearance ||
            point.z < arena.min.z + clearance || point.z > arena.max.z - clearance ||
            (point.x >= runnerZoneMinX - clearance && point.z <= runnerZoneMaxZ + clearance))
            return false;

        var filter = new NavMeshQueryFilter { agentTypeID = agent.agentTypeID, areaMask = agent.areaMask };
        if (!NavMesh.SamplePosition(point, out NavMeshHit hit, 1.5f, filter))
            return false;

        // Keep the pair's sampled XZ positions and use NavMesh sampling only for floor height.
        Vector3 offset = hit.position - point;
        if (offset.x * offset.x + offset.z * offset.z > 0.000001f)
            return false;

        Vector3 floor = new Vector3(point.x, hit.position.y, point.z);
        float bodyHeight = body ? Mathf.Max(body.bounds.size.y, agent.height) : agent.height;
        Vector3 lower = floor + Vector3.up * clearance;
        Vector3 upper = floor + Vector3.up * Mathf.Max(clearance, bodyHeight - clearance);
        foreach (Collider obstacle in Physics.OverlapCapsule(lower, upper, clearance, ~0, QueryTriggerInteraction.Ignore))
        {
            if (obstacle.gameObject.name != "Ground" && !obstacle.GetComponentInParent<HideSeekAgent>())
                return false;
        }

        spawnPoint = floor;
        return true;
    }

    private void ResetRound(bool keepPosition = false, bool jitter = true)
    {
        if (!keepPosition)
        {
            Vector3 candidate = jitter
                ? start + new Vector3(Random.Range(-2f, 2f), 0f, Random.Range(-2f, 2f))
                : start;
            var filter = new NavMeshQueryFilter
            {
                agentTypeID = navigation.agentTypeID,
                areaMask = navigation.areaMask
            };
            bool found = NavMesh.SamplePosition(candidate, out NavMeshHit hit, 1.5f, filter);
            if (!navigation.Warp(found ? hit.position : start))
                navigation.Warp(start);
        }
        spawn = Position;
        previousPosition = spawn;
        travelDistance = 0f;
        searchedCells.Clear();
        nextSurvey = 0f;
        preparationBaselinesCaptured = false;
        practiceLayoutTicks = 0;
        System.Array.Clear(passageBaselines, 0, passageBaselines.Length);
        passagesBlocked = 0;
        rewardedBarrierCrates.Clear();
        passagesMeasured = 0;
        preparationCrateMoves = 0;
        lastCountedCrateAction = 0;
        lastCountedCrate = null;
        preparationScored = false;
        movedCrateDuringPreparation = false;
        if (surveyedAt != null)
            System.Array.Fill(surveyedAt, float.NegativeInfinity);
        patrolCell = -1;
        nextPatrolAttempt = 0f;
        patrolRetryAt.Clear();
        navigation.ResetPath();
        elapsed = 0f;
        direction = Vector3.zero;
        crateAction = 0;
        previousCrateAction = 0;
        lockKeyPending = false;
        ReleaseCrate();
        ClearCrateLock(movableCrate);
        ClearCrateLock(secondCrate);
        movedCover = null;
        coverSeconds = 0f;
        coverRewarded = false;
        hasMemory = false;
        reportedAt = float.NegativeInfinity;
        nextResponsePath = 0f;
        responseCorners = System.Array.Empty<Vector3>();
        responseCorner = 0;
        if (seeker)
            navigation.speed = baseSpeed;
        discoveries = 0;
        visibleSeconds = 0f;
        wasVisible = false;
        hiddenDuringSearch = false;
        rediscovered = false;
    }

    private static void ObserveXZ(VectorSensor sensor, Vector3 value)
    {
        sensor.AddObservation(Mathf.Clamp(value.x, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(value.z, -1f, 1f));
    }
}
