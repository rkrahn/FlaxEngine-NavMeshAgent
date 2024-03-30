using FlaxEngine;

namespace Game;

public enum AgentType
{
    Steering,
    ExactPath
}

/// <summary>
/// NavMeshAgent Script.
/// </summary>
public class NavMeshAgent : Script
{
    public CharacterController characterController;
    public AgentType agentType;

    /// <summary>
    /// The offset applied to the actor position on moving it.
    /// </summary>
    public Vector3 Offset = new Vector3(0, 100, 0);

    /// <summary>
    /// Agent movement speed (units/second).
    /// </summary>
    public float Speed = 500.0f;

    /// <summary>
    /// Agent steering speed.
    /// </summary>
    public float SteeringSpeed = 5.0f;

    /// <summary>
    /// Determines when/if to move forward/backward. 
    /// Values equal to or greater than 0 will make the agent only move forward.
    /// Values are clamped between 1 and -1.
    /// </summary>
    public float AlignmentFactor = 0f;

    /// <summary>
    /// If true, stops the agent's update loop.
    /// </summary>
    public bool IsStopped = false;

    /// <summary>
    /// Distance from waypoint to be considered reached.
    /// </summary>
    public float StopDistance = 20f;

    private Vector3 _destination;
    private Vector3 _targetPos;
    private Vector3[] _path;
    private float _pathLength;
    private float _pathPosition;

    private int _waypoint;
    private Vector3 _currentPos;

    /// <inheritdoc />
    public override void OnAwake()
    {
        SetDestination(Actor.Position);
    }

    /// <inheritdoc />
    public override void OnUpdate()
    {
        if (IsStopped)
        {
            return;
        }

        _currentPos = Actor.Position;

        // Check if reached target location
        if (Vector3.Distance(ref _currentPos, ref _destination) < StopDistance)
            return;

        // Check if need to build a new path
        if (_destination != _targetPos)
        {
            BuildNewPath();
        }

        // Skip if has no path
        if (_path == null)
        {
            return;
        }

        switch (agentType)
        {
            case AgentType.ExactPath:
                ForceMovePath();
                break;

            case AgentType.Steering:
                SteerPath();
                break;
        }
    }

    private void BuildNewPath()
    {
        _targetPos = _destination;
        _pathPosition = 0;
        _pathLength = 0;
        _waypoint = 1;

        if (!Navigation.TestPath(_currentPos, _targetPos))
        {
            Debug.LogWarning("Failed to find path to the target.");
            _path = null;
            return;
        }
        Navigation.FindPath(_currentPos, _targetPos, out _path);

        // Move the start/end points to navmesh floor
        if (_path.Length != 0)
        {
            Navigation.FindClosestPoint(_path[0], out _path[0]);
        }

        if (_path.Length > 1)
        {
            Navigation.FindClosestPoint(_path[_path.Length - 1], out _path[_path.Length - 1]);
        }

        // Compute path length
        for (int i = 1; i < _path.Length; i++)
        {
            _pathLength += Vector3.Distance(ref _path[i - 1], ref _path[i]);
        }
    }

    public void SetDestination(Vector3 destination)
    {
        _destination = destination;
    }

    private void SteerPath()
    {
        // Skip if reached destination
        if (_waypoint >= _path.Length)
        {
            Debug.Log($"Path length end: {_path.Length}");
            return;
        }

        // Calculate direction to next waypoint
        Vector3 direction = _path[_waypoint] - _currentPos;
        direction.Y = 0;
        direction.Normalize();

        // Flatten forward vector
        Vector3 forward = Transform.Forward;
        forward.Y = 0;
        forward.Normalize();

        // Calculate forward/backward movement
        float alignment = Vector3.Dot(ref forward, ref direction);
        if (alignment > AlignmentFactor)
        {
            alignment = Mathf.Clamp(alignment, -1f, 1f);
        }
        else
        {
            alignment = 0;
        }

        // Calculate movement speed to prevent overshooting
        float distanceToWaypoint = Vector3.Distance(ref _currentPos, ref _path[_waypoint]);
        var forwardSpeed = Speed;
        if (distanceToWaypoint < Speed * Time.DeltaTime)
        {
            forwardSpeed = distanceToWaypoint;
        }

        characterController.SimpleMove(forward * forwardSpeed * alignment);

        // Calculate rotation
        var rotation = Quaternion.GetRotationFromTo(forward, direction, Vector3.Up);

        float scale = 1f;
        if (Mathf.RadiansToDegrees * rotation.Angle > SteeringSpeed)
        {
            var ratio = Mathf.RadiansToDegrees * rotation.Angle / SteeringSpeed;
            scale = 1 / ratio;
        }

        characterController.Orientation = Quaternion.Lerp(
            characterController.Orientation,
            characterController.Orientation * rotation,
            scale);

        if (distanceToWaypoint < StopDistance)
        {
            _waypoint++;
        }
    }

    private void ForceMovePath()
    {
        // Move
        var pathProgress = Mathf.Min(_pathLength * _pathPosition + Time.DeltaTime * Speed, _pathLength);
        _pathPosition = pathProgress / _pathLength;

        // Calculate position on path
        float segmentsSum = 0;
        for (int i = 0; i < _path.Length - 1; i++)
        {
            var segmentLength = Vector3.Distance(ref _path[i], ref _path[i + 1]);
            if (segmentsSum <= pathProgress && segmentsSum + segmentLength >= pathProgress)
            {
                float t = (pathProgress - segmentsSum) / segmentLength;
                var targetPos = Vector3.Lerp(_path[i], _path[i + 1], t) + Offset;
                characterController.Move(targetPos - _currentPos);

                // Rotate
                Vector3 direction = _path[i + 1] - _currentPos;
                direction.Y = 0;
                direction = Vector3.Normalize(direction);
                var forward = Transform.Forward;
                forward.Y = 0;

                var rotation = Quaternion.GetRotationFromTo(forward, direction, Vector3.Up);
                Actor.Orientation *= rotation;
                break;
            }

            segmentsSum += segmentLength;
        }
    }

    public override void OnDebugDraw()
    {
        if (_path == null)
            return;

        for (int i = 0; i < _path.Length - 1; i++)
        {
            DebugDraw.DrawLine(_path[i], _path[i + 1], Color.Gold);
        }
    }
}
