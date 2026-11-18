using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.AI;


[RequireComponent(typeof(NavMeshAgent))]
public class BoxEscapeRunner : Agent
{
    [SerializeField] private NavMeshAgent chaser;
    [SerializeField] private Transform crate;
    [SerializeField] private Transform shortcutExit;
    [SerializeField] private Transform detourExit;
    [SerializeField] private Transform pushStart;
    [SerializeField] private Transform pullStart;
    [SerializeField] private Transform crateBlocked;

    [SerializeField, Min(0.1f)] private float pushRange = 0.6f;
    [SerializeField, Min(0.1f)] private float catchRange = 0.8f;
    [SerializeField, Min(1f)] private float roundSeconds = 40f;

    private NavMeshAgent runner;
    private Vector3 runnerStart;
    private Vector3 chaserStart;
    private Vector3 crateStart;
    private bool blocked;
    private const int MaxCrateRoundTrips = 2;
    private const float StarIntervalSeconds = 10f;
    private const int MaxHealth = 100;
    private const int HealthDrain = 25;
    private const int HealthRecovery = 50;
    private const float HealthTickSeconds = 5f;
    private int health;
    private int chaserHealth;
    private float nextHealthTick;
    private float nextStarDrop;
    private GameObject brownStar;
    private GameObject greenStar;
    private GameObject secondGreenStar;
    private int healthStarsCollected;
    private float nextTacticalSample;
    private float previousThreatDistance;
    private float previousHealthStarDistance;
    private Mesh starMesh;
    private Material brownMaterial;
    private Material greenMaterial;
    private NavMeshPath pickupPath;
    private int crateRoundTrips;
    private Vector3 idleAnchor;
    private float idleSince;
    private float prolongedIdleSeconds;
    private bool CanBlockShortcut => !blocked && crateRoundTrips < MaxCrateRoundTrips;
    private bool approachingCrate;
    private bool approachingToPull;
    private bool movingCrate;
    private bool blockingCrate;
    private Vector3 crateMoveGoal;
    private Vector3 pullApproachPoint;
    private Vector3 pullRunnerStart;
    private Vector3 pullCrateStart;
    private Vector3 pullOffset;
    private float pullProgress;
    private float elapsed;
    private const float DirectionStep = 4f;
    private float secondReopenTime;
    private Vector3 moveDirection;
    private const int PushAction = 9;
    private const int PullAction = 10;
    private static readonly Vector3[] MoveDirections =
    {
        Vector3.forward, Vector3.back, Vector3.left, Vector3.right,
        new Vector3(-1f, 0f, 1f).normalized,
        new Vector3(1f, 0f, 1f).normalized,
        new Vector3(-1f, 0f, -1f).normalized,
        new Vector3(1f, 0f, -1f).normalized
    };
    private float nextRepath;
    private int manualAction;

    public override void Initialize()
    {
        runner = GetComponent<NavMeshAgent>();
        pickupPath = new NavMeshPath();
        runnerStart = runner.transform.position;
        chaserStart = chaser.transform.position;
        crateStart = crate.position;
    }

    public override void OnEpisodeBegin()
    {
        crate.position = crateStart;
        ResetPosition(runner, runnerStart);
        ResetPosition(chaser, chaserStart);
        blocked = false;
        crateRoundTrips = 0;
        health = MaxHealth;
        chaserHealth = MaxHealth;
        nextHealthTick = HealthTickSeconds;
        nextStarDrop = StarIntervalSeconds;
        if (brownStar) brownStar.SetActive(false);
        if (greenStar) greenStar.SetActive(false);
        if (secondGreenStar) secondGreenStar.SetActive(false);
        healthStarsCollected = 0;
        nextTacticalSample = 0f;
        previousThreatDistance = float.PositiveInfinity;
        previousHealthStarDistance = float.PositiveInfinity;
        approachingCrate = false;
        approachingToPull = false;
        movingCrate = false;
        elapsed = 0f;
        moveDirection = Vector3.zero;
        secondReopenTime = -1f;
        idleAnchor = transform.position;
        idleSince = 0f;
        prolongedIdleSeconds = 0f;
        nextRepath = 0f;
        manualAction = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        ObserveOffset(sensor, chaser.transform.position);
        ObserveOffset(sensor, crate.position);
        ObserveOffset(sensor, pushStart.position);
        sensor.AddObservation(blocked ? 1f : 0f);
        sensor.AddObservation((float)crateRoundTrips / MaxCrateRoundTrips);

        ObserveOffset(sensor, pullStart.position);
        sensor.AddObservation(chaser.velocity.x / Mathf.Max(chaser.speed, 0.1f));
        sensor.AddObservation(chaser.velocity.z / Mathf.Max(chaser.speed, 0.1f));

        foreach (Vector3 direction in MoveDirections)
            sensor.AddObservation(GetDirectionalClearance(direction) / DirectionStep);

        sensor.AddObservation(moveDirection.x);
        sensor.AddObservation(moveDirection.z);
        foreach (Vector3 direction in MoveDirections)
            sensor.AddObservation(GetDirectionalClearance(direction, 12f) / 12f);

        sensor.AddObservation(Mathf.Clamp01(elapsed / roundSeconds));
        sensor.AddObservation(health / (float)MaxHealth);
        sensor.AddObservation(Mathf.Clamp01((nextHealthTick - elapsed) / HealthTickSeconds));
        sensor.AddObservation(Mathf.Clamp01((nextStarDrop - elapsed) / StarIntervalSeconds));
        ObserveStar(sensor, brownStar);
        ObserveStar(sensor, greenStar);
        ObserveStar(sensor, secondGreenStar);
        sensor.AddObservation(chaserHealth / (float)MaxHealth);
        sensor.AddObservation(Mathf.Clamp01((elapsed - idleSince) / 2f));
    }

    private float GetDirectionalClearance(Vector3 direction, float range = DirectionStep)
    {
        if (!runner.isOnNavMesh)
            return 0f;

        Vector3 origin = runner.nextPosition;
        if (runner.Raycast(origin + direction * range, out NavMeshHit hit))
        {
            float clearance = Vector3.Dot(hit.position - origin, direction);
            return Mathf.Clamp(clearance - 0.05f, 0f, range);
        }

        return range;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (approachingCrate || approachingToPull || movingCrate || !runner.isOnNavMesh)
            return;

        int action = actions.DiscreteActions[0];
        if (action >= 1 && action <= MoveDirections.Length)
        {
            if (runner.hasPath || runner.pathPending)
                runner.ResetPath();
            Vector3 direction = MoveDirections[action - 1];
            if (elapsed < 5f && Vector3.Dot(moveDirection, direction) < -0.5f)
                AddReward(-0.015f);
            moveDirection = direction;
            return;
        }

        moveDirection = Vector3.zero;
        switch (action)
        {
            case PushAction:
                if (CanBlockShortcut)
                    approachingCrate = runner.SetDestination(pushStart.position);
                break;
            case PullAction:
                if (blocked)
                    BeginCratePull();
                break;
        }
    }

    private void MoveRunner()
    {
        if (moveDirection == Vector3.zero || approachingCrate || approachingToPull || movingCrate)
            return;

        float step = runner.speed * Time.fixedDeltaTime;
        Vector3 movement = moveDirection * GetDirectionalClearance(moveDirection, step);
        if (movement.sqrMagnitude > 0f)
        {
            runner.Move(movement);
            transform.rotation = Quaternion.LookRotation(moveDirection);
        }
    }

    public override void WriteDiscreteActionMask(IDiscreteActionMask actionMask)
    {
        for (int index = 0; index < MoveDirections.Length; index++)
            actionMask.SetActionEnabled(0, index + 1,
                GetDirectionalClearance(MoveDirections[index], 0.2f) > 0.025f);
        actionMask.SetActionEnabled(0, PushAction, CanBlockShortcut);
        actionMask.SetActionEnabled(0, PullAction, blocked);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActions = actionsOut.DiscreteActions;
        discreteActions[0] = manualAction;
        manualAction = 0;

#if ENABLE_LEGACY_INPUT_MANAGER
        if (discreteActions[0] != 0)
            return;

        bool north = Input.GetKey(KeyCode.W);
        bool south = Input.GetKey(KeyCode.S);
        bool west = Input.GetKey(KeyCode.A);
        bool east = Input.GetKey(KeyCode.D);

        if (north && !south)
            discreteActions[0] = west == east ? 1 : west ? 5 : 6;
        else if (south && !north)
            discreteActions[0] = west == east ? 2 : west ? 7 : 8;
        else if (west != east)
            discreteActions[0] = west ? 3 : 4;
#endif
    }

    private void Update()
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        if (Input.GetKeyDown(KeyCode.Alpha3)) manualAction = PushAction;
        if (Input.GetKeyDown(KeyCode.Alpha4)) manualAction = PullAction;
#endif
    }

    private void FixedUpdate()
    {
        if (runner == null || !runner.isOnNavMesh || !chaser.isOnNavMesh)
            return;

        if (approachingCrate)
        {
            if (FlatDistance(transform.position, pushStart.position) <= pushRange)
            {
                runner.ResetPath();
                approachingCrate = false;
                if (CanBlockShortcut)
                {
                    BeginCrateMove(new Vector3(
                        crateBlocked.position.x, crateStart.y, crateBlocked.position.z), true);
                }
            }
            else if (!runner.pathPending &&
                     (!runner.hasPath || runner.pathStatus != NavMeshPathStatus.PathComplete))
            {
                runner.ResetPath();
                approachingCrate = false;
            }
        }

        MoveRunner();
        UpdateCratePull();

        elapsed += Time.fixedDeltaTime;
        UpdatePickups();
        if (health <= 0)
        {
            FinishRound(-1f, "체력 소진");
            return;
        }

        if (chaserHealth <= 0)
        {
            FinishRound(1f, "Chaser 체력 소진 · Runner 승리");
            return;
        }

        if (movingCrate || FlatDistance(transform.position, idleAnchor) >= 1.5f)
        {
            idleAnchor = transform.position;
            idleSince = elapsed;
        }

        bool stayingInPlace = elapsed - idleSince >= 2f;
        if (stayingInPlace)
            prolongedIdleSeconds += Time.fixedDeltaTime;

        float survivalRewardRate = stayingInPlace ? -0.08f : 0.02f;
        AddReward(survivalRewardRate * Time.fixedDeltaTime);
        UpdateTacticalRewards();
        if (elapsed >= nextRepath)
        {
            chaser.SetDestination(ChooseChaserDestination());
            var corners = chaser.path.corners;
            for (int i = 1; i < corners.Length; i++)
            {
                Debug.DrawLine(
                    corners[i - 1] + Vector3.up * 0.3f,
                    corners[i] + Vector3.up * 0.3f,
                    Color.magenta,
                    0.25f);
            }
            nextRepath = elapsed + 0.2f;
        }

        float contactRange = Mathf.Max(catchRange, runner.radius + chaser.radius + 0.1f);
        if (FlatDistance(transform.position, chaser.transform.position) <= contactRange &&
            !NavMesh.Raycast(chaser.nextPosition, runner.nextPosition,
                out _, chaser.areaMask))
        {
            FinishRound(-1f, "잡힘");
        }
        else if (elapsed >= roundSeconds)
        {
            FinishRound(1f, "제한 시간 도달");
        }
    }

    private void BeginCratePull()
    {
        if (pullStart == null)
        {
            Debug.LogWarning("[BoxEscape] PullStart를 연결해 주세요.", this);
            return;
        }

        var filter = new NavMeshQueryFilter
        {
            agentTypeID = runner.agentTypeID,
            areaMask = runner.areaMask
        };

        if (!NavMesh.SamplePosition(pullStart.position, out NavMeshHit hit, 0.75f, filter))
        {
            Debug.LogWarning("[BoxEscape] PullStart 주변에서 걸을 수 있는 위치를 찾지 못했어요.", this);
            return;
        }

        pullApproachPoint = hit.position;
        approachingToPull = runner.SetDestination(pullApproachPoint);
    }

    private void UpdateCratePull()
    {
        if (approachingToPull)
        {
            if (FlatDistance(transform.position, pullApproachPoint) <= pushRange)
            {
                approachingToPull = false;
                runner.ResetPath();
                BeginCrateMove(crateStart, false);
            }
            else if (!runner.pathPending &&
                     (!runner.hasPath || runner.pathStatus != NavMeshPathStatus.PathComplete))
            {
                runner.ResetPath();
                approachingToPull = false;
            }
        }

        AdvanceCrateMove();
    }

    private void BeginCrateMove(Vector3 goal, bool blockShortcut)
    {
        pullRunnerStart = runner.nextPosition;
        pullCrateStart = crate.position;
        crateMoveGoal = goal;
        pullOffset = goal - pullCrateStart;
        pullProgress = 0f;
        blockingCrate = blockShortcut;
        movingCrate = pullOffset.sqrMagnitude > 0.0001f;
    }

    private void AdvanceCrateMove()
    {
        if (!movingCrate)
            return;

        // Face the crate while pushing or pulling.
        Vector3 towardCrate = crate.position - transform.position;
        towardCrate.y = 0f;
        if (towardCrate.sqrMagnitude > 0.0001f)
            transform.rotation = Quaternion.LookRotation(towardCrate);

        float nextProgress = Mathf.MoveTowards(
            pullProgress, 1f, 1.5f * Time.fixedDeltaTime / pullOffset.magnitude);
        Vector3 nextPosition = pullRunnerStart + pullOffset * nextProgress;
        if (runner.Raycast(nextPosition, out NavMeshHit hit))
        {
            string action = blockingCrate ? "3 밀기" : "4 끌기";
            Debug.LogWarning(
                $"[BoxEscape] 상자 이동 경로 차단 | {action} | {elapsed:F2}초 | 진행 {pullProgress:P0} " +
                $"| Runner 출발 {pullRunnerStart:F2} | 현재 {runner.nextPosition:F2} " +
                $"| 다음 위치 {nextPosition:F2} | NavMesh 경계 {hit.position:F2} " +
                $"| 상자 현재 {crate.position:F2} | 상자 목표 {crateMoveGoal:F2}", this);
            crate.position = pullCrateStart;
            movingCrate = false;
            return;
        }

        runner.Move(nextPosition - runner.nextPosition);
        crate.position = Vector3.Lerp(pullCrateStart, crateMoveGoal, nextProgress);
        pullProgress = nextProgress;
        if (pullProgress >= 1f)
        {
            movingCrate = false;
            blocked = blockingCrate;
            if (!blockingCrate && ++crateRoundTrips == MaxCrateRoundTrips)
            {
                if (secondReopenTime < 0f)
                    secondReopenTime = elapsed;
            }
        }
    }

    private void FinishRound(float reward, string outcome)
    {
        string crateState = blocked ? "막힘" : "열림";
        Debug.Log($"[BoxEscape] {outcome} | 경과 {elapsed:F2}초 | 초록 별 {healthStarsCollected}개 | 상자 {crateState} | 왕복 {crateRoundTrips}/{MaxCrateRoundTrips}", this);
        var stats = Academy.Instance.StatsRecorder;
        stats.Add("BoxEscape/Round Duration Seconds", elapsed);
        stats.Add("BoxEscape/Health Stars Collected", healthStarsCollected);
        stats.Add("BoxEscape/Prolonged Idle Seconds", prolongedIdleSeconds);
        stats.Add("BoxEscape/Reached Time Limit", outcome == "제한 시간 도달" ? 1f : 0f);
        stats.Add("BoxEscape/Completed Two Round Trips", secondReopenTime >= 0f ? 1f : 0f);
        if (secondReopenTime >= 0f)
        {
            float postReopenSeconds = elapsed - secondReopenTime;
            stats.Add("BoxEscape/Survival After Second Reopen", postReopenSeconds);
            Debug.Log($"[BoxEscape] 두 번째로 길을 연 뒤 경과 {postReopenSeconds:F2}초", this);
        }
        AddReward(reward);
        EndEpisode();
    }

    private void UpdateTacticalRewards()
    {
        if (elapsed < nextTacticalSample)
            return;

        nextTacticalSample = elapsed + 0.4f;
        float threatDistance = FlatDistance(runner.nextPosition, chaser.nextPosition);
        if (threatDistance < 8f && previousThreatDistance < 8f)
            AddReward(0.025f * Mathf.Clamp(threatDistance - previousThreatDistance, -1f, 1f));
        previousThreatDistance = threatDistance;

        float starDistance = health <= 50 && threatDistance >= 6f
            ? Mathf.Min(GetStarPathDistance(runner, greenStar), GetStarPathDistance(runner, secondGreenStar))
            : float.PositiveInfinity;
        if (!float.IsInfinity(starDistance) && !float.IsInfinity(previousHealthStarDistance))
            AddReward(0.015f * Mathf.Clamp(previousHealthStarDistance - starDistance, -1f, 1f));
        previousHealthStarDistance = starDistance;
    }

    private float GetStarPathDistance(NavMeshAgent agent, GameObject star)
    {
        if (!star || !star.activeSelf)
            return float.PositiveInfinity;

        return PathDistance(agent, star.transform.position - Vector3.up * 0.12f);
    }

    private void UpdatePickups()
    {
        if (elapsed + 0.001f >= nextStarDrop)
        {
            nextStarDrop += StarIntervalSeconds;
            previousHealthStarDistance = float.PositiveInfinity;
            SpawnStar(ref brownStar, ref brownMaterial, "갈색 상자 별", new Color(0.62f, 0.32f, 0.10f));
            var green = new Color(0.23f, 0.95f, 0.25f);
            SpawnStar(ref greenStar, ref greenMaterial, "초록 체력 별 1", green, preferFarFromChaser: true);
            SpawnStar(ref secondGreenStar, ref greenMaterial, "초록 체력 별 2", green, preferFarFromChaser: true);
        }

        if (crateRoundTrips == MaxCrateRoundTrips && CanCollect(brownStar))
        {
            crateRoundTrips = 0;
            brownStar.SetActive(false);
        }

        CollectHealthStar(greenStar);
        CollectHealthStar(secondGreenStar);
        CollectChaserHealthStar(greenStar);
        CollectChaserHealthStar(secondGreenStar);

        if (elapsed + 0.001f >= nextHealthTick)
        {
            nextHealthTick += HealthTickSeconds;
            health = Mathf.Max(0, health - HealthDrain);
            chaserHealth = Mathf.Max(0, chaserHealth - HealthDrain);
        }
    }

    private void CollectHealthStar(GameObject star)
    {
        if (health >= MaxHealth || !CanCollect(star))
            return;

        int recovered = Mathf.Min(MaxHealth - health, HealthRecovery);
        health += recovered;
        healthStarsCollected++;
        AddReward(0.30f * recovered / HealthRecovery);
        previousHealthStarDistance = float.PositiveInfinity;
        star.SetActive(false);
    }

    private Vector3 ChooseChaserDestination()
    {
        if (chaserHealth > 50)
            return transform.position;

        float firstDistance = GetStarPathDistance(chaser, greenStar);
        float secondDistance = GetStarPathDistance(chaser, secondGreenStar);
        if (float.IsPositiveInfinity(Mathf.Min(firstDistance, secondDistance)))
            return transform.position;

        GameObject target = firstDistance <= secondDistance ? greenStar : secondGreenStar;
        return target.transform.position - Vector3.up * 0.12f;
    }

    private void CollectChaserHealthStar(GameObject star)
    {
        if (chaserHealth >= MaxHealth || !CanCollect(star, chaser))
            return;

        chaserHealth = Mathf.Min(MaxHealth, chaserHealth + HealthRecovery);
        star.SetActive(false);
        previousHealthStarDistance = float.PositiveInfinity;
    }

    private bool CanCollect(GameObject star) => CanCollect(star, runner);

    private static bool CanCollect(GameObject star, NavMeshAgent agent)
    {
        if (!star || !star.activeSelf)
            return false;

        Vector3 position = star.transform.position;
        position.y = agent.nextPosition.y;
        return FlatDistance(position, agent.nextPosition) <= 0.85f &&
            !agent.Raycast(position, out _);
    }

    private void ObserveStar(VectorSensor sensor, GameObject star)
    {
        bool available = star && star.activeSelf;
        sensor.AddObservation(available ? 1f : 0f);
        if (available)
            ObserveOffset(sensor, star.transform.position);
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
    }

    private void SpawnStar(ref GameObject star, ref Material material, string starName, Color color, bool preferFarFromChaser = false)
    {
        var filter = new NavMeshQueryFilter
        {
            agentTypeID = runner.agentTypeID,
            areaMask = runner.areaMask
        };
        if (star) star.SetActive(false);
        Vector3 bestPosition = default;
        float bestDistance = -1f;
        for (int attempt = 0; attempt < 48; attempt++)
        {
            // Sample the current arena around the original spawn, then verify reachability.
            Vector2 offset = Random.insideUnitCircle * 34f;
            Vector3 candidate = runnerStart + new Vector3(offset.x, 0f, offset.y);
            if (!NavMesh.SamplePosition(candidate, out NavMeshHit hit, 3f, filter) ||
                FlatDistance(hit.position, runner.nextPosition) < (preferFarFromChaser ? 7f : 3f) ||
                FlatDistance(hit.position, chaser.nextPosition) < 2f ||
                IsNearStar(brownStar, hit.position) || IsNearStar(greenStar, hit.position) ||
                IsNearStar(secondGreenStar, hit.position))
                continue;

            float runnerDistance = PathDistance(runner, hit.position);
            if (float.IsInfinity(runnerDistance) || (preferFarFromChaser && runnerDistance > 24f))
                continue;

            float chaserDistance = preferFarFromChaser ? PathDistance(chaser, hit.position) : 0f;
            if (float.IsInfinity(chaserDistance))
                chaserDistance = FlatDistance(chaser.nextPosition, hit.position);
            float distance = preferFarFromChaser ? chaserDistance + 0.5f * runnerDistance : 0f;
            if (distance <= bestDistance)
                continue;

            bestDistance = distance;
            bestPosition = hit.position;
            if (!preferFarFromChaser)
                break;
        }

        if (bestDistance < 0f)
            return;

        if (!star)
        {
            star = new GameObject(starName);
            star.AddComponent<MeshFilter>().sharedMesh = GetStarMesh();
            if (!material)
            {
                Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
                material = new Material(shader) { color = color };
            }
            star.AddComponent<MeshRenderer>().sharedMaterial = material;
        }
        star.transform.position = bestPosition + Vector3.up * 0.12f;
        star.SetActive(true);
    }

    private float PathDistance(NavMeshAgent agent, Vector3 destination)
    {
        if (!agent.isOnNavMesh || !agent.CalculatePath(destination, pickupPath) ||
            pickupPath.status != NavMeshPathStatus.PathComplete)
            return float.PositiveInfinity;

        float distance = 0f;
        Vector3 previous = agent.nextPosition;
        foreach (Vector3 corner in pickupPath.corners)
        {
            distance += FlatDistance(previous, corner);
            previous = corner;
        }
        return distance;
    }

    private static bool IsNearStar(GameObject star, Vector3 position)
    {
        return star && star.activeSelf && FlatDistance(star.transform.position, position) < 2f;
    }

    private Mesh GetStarMesh()
    {
        if (starMesh)
            return starMesh;

        var vertices = new Vector3[11];
        var triangles = new int[30];
        for (int i = 0; i < 10; i++)
        {
            float angle = i * Mathf.PI / 5f;
            float radius = i % 2 == 0 ? 0.85f : 0.36f;
            vertices[i + 1] = new Vector3(Mathf.Sin(angle) * radius, 0f, Mathf.Cos(angle) * radius);
            triangles[i * 3 + 1] = i + 1;
            triangles[i * 3 + 2] = (i + 1) % 10 + 1;
        }

        starMesh = new Mesh { name = "Pickup Star", vertices = vertices, triangles = triangles };
        starMesh.RecalculateNormals();
        return starMesh;
    }

    private void OnGUI()
    {
        if (!Application.isPlaying || Application.isBatchMode || !runner)
            return;

        string crateStatus = crateRoundTrips == MaxCrateRoundTrips
            ? "갈색 별 필요" : $"상자 왕복 {crateRoundTrips}/{MaxCrateRoundTrips}";
        GUI.Box(new Rect(12, 12, 220, 70), $"Runner 체력 {health}/{MaxHealth}\nChaser 체력 {chaserHealth}/{MaxHealth}\n{crateStatus}");
    }

    protected override void OnDisable()
    {
        base.OnDisable();
        if (brownStar) Destroy(brownStar);
        if (greenStar) Destroy(greenStar);
        if (secondGreenStar) Destroy(secondGreenStar);
        if (starMesh) Destroy(starMesh);
        if (brownMaterial) Destroy(brownMaterial);
        if (greenMaterial) Destroy(greenMaterial);
    }

    private void ObserveOffset(VectorSensor sensor, Vector3 point)
    {
        Vector3 offset = point - transform.position;
        sensor.AddObservation(Mathf.Clamp(offset.x / 20f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(offset.z / 20f, -1f, 1f));
    }

    private static float FlatDistance(Vector3 a, Vector3 b)
    {
        return Vector2.Distance(new Vector2(a.x, a.z), new Vector2(b.x, b.z));
    }

    private static void ResetPosition(NavMeshAgent agent, Vector3 position)
    {
        if (agent.Warp(position))
            agent.ResetPath();
        else
            Debug.LogError($"{agent.name}의 시작 위치가 NavMesh 밖에 있어요.", agent);
    }
}