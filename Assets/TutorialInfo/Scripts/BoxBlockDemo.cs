using System.Collections;
using UnityEngine;

public class BoxBlockDemo : MonoBehaviour
{
    [Header("Scene objects")]
    [SerializeField] private Transform runner;
    [SerializeField] private Transform chaser;
    [SerializeField] private Transform crate;

    [Header("Runner")]
    [SerializeField] private Transform[] runnerToPush;
    [SerializeField] private Transform crateBlockedPosition;
    [SerializeField] private Transform runnerEscapePoint;

    [Header("Chaser")]
    [SerializeField] private Transform[] shortcut;
    [SerializeField] private Transform[] detour;
    [SerializeField] private Transform finishPoint;

    [Header("Test settings")]
    [SerializeField] private bool blockShortcut = true;
    [SerializeField, Min(0.1f)] private float moveSpeed = 3f;
    [SerializeField, Min(0.1f)] private float pushSpeed = 1.5f;

    private IEnumerator Start()
    {
        if (!runner || !chaser || !crate || !crateBlockedPosition ||
            !runnerEscapePoint || !finishPoint ||
            !HasWaypoints(runnerToPush) ||
            !HasWaypoints(shortcut) || !HasWaypoints(detour))
        {
            Debug.LogError("BoxBlockDemo: Inspector의 참조와 경로를 확인해 주세요.", this);
            yield break;
        }

        yield return Follow(runner, runnerToPush);

        if (blockShortcut)
        {
            Vector3 crateGoal = AtCurrentHeight(crate, crateBlockedPosition.position);
            Vector3 pushOffset = runner.position - crate.position;

            while (crate.position != crateGoal)
            {
                crate.position = Vector3.MoveTowards(
                    crate.position, crateGoal, pushSpeed * Time.deltaTime);
                runner.position = crate.position + pushOffset;
                yield return null;
            }
        }

        // 두 조건 모두 Runner가 같은 지점에 도착하면 Chaser를 출발시킵니다.
        yield return Move(runner, runnerEscapePoint.position);

        float startedAt = Time.time;
        yield return Follow(chaser, blockShortcut ? detour : shortcut);
        yield return Move(chaser, finishPoint.position);

        string route = blockShortcut ? "우회로" : "지름길";
        Debug.Log($"Chaser {route} 도착 — {Time.time - startedAt:F2}초", this);
    }

    private IEnumerator Follow(Transform actor, Transform[] waypoints)
    {
        foreach (Transform waypoint in waypoints)
            yield return Move(actor, waypoint.position);
    }

    private IEnumerator Move(Transform actor, Vector3 destination)
    {
        Vector3 goal = AtCurrentHeight(actor, destination);

        while (actor.position != goal)
        {
            actor.position = Vector3.MoveTowards(
                actor.position, goal, moveSpeed * Time.deltaTime);
            yield return null;
        }
    }

    private static Vector3 AtCurrentHeight(Transform actor, Vector3 position)
    {
        position.y = actor.position.y;
        return position;
    }

    private static bool HasWaypoints(Transform[] waypoints)
    {
        if (waypoints == null || waypoints.Length == 0)
            return false;

        foreach (Transform waypoint in waypoints)
        {
            if (!waypoint)
                return false;
        }

        return true;
    }
}