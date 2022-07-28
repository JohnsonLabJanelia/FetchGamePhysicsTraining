using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// A trainable agent for the fetch game:
/// <seealso href="https://github.com/JohnsonLabJanelia/FetchGamePhysics/tree/main/FetchArenaProject"/>
/// </summary>
public class FetchGamePhysicsTrainingAgent : Janelia.EasyMLAgentGrounded
{
    public override string BehaviorName { get; protected set; } = "FetchGamePhysics";

    public override int VectorObservationSize { get; protected set; } = 0;

#if false
    // Overriding this virtual property would be an alternative to setting its value in `Setup`, but
    // the latter seems simpler, and `Setup` is needed for a few other purposes anyway.
    public override List<string> ChildSensorForwardDetectableTags { get; protected set; } = new List<string>()
    {
        FetchGamePhysicsTrainingArena.TAG_BOUNDARY,
        FetchGamePhysicsTrainingArena.TAG_RAMP
    };
#endif

    /// <summary>
    /// Supports adjusting the force direction to fake sliding along the obstacle.
    /// </summary>
    public override Vector3 MoveForwardDirection
    {
        get { return (_moveForwardDirection != Vector3.zero) ? _moveForwardDirection : transform.forward; }
    }
    private Vector3 _moveForwardDirection;

    /// <summary>
    /// The number of discrete action branches. 
    /// This agent only has jumping as discrete action, with two choices: either to jump or not to jump.
    /// </summary>
    public override int[] DiscreteBranchSize { get; protected set; } = new int[] { 2 };

    // This value cannot be too large or the agent flies off the arena.
    [Tooltip("Force to apply when jumping")]
    public float jumpForce;

    protected GameObject _ball;
    protected Rigidbody _ballRigidBody;
    public float fieldOfViewDegree { get; protected set; }
    protected bool _isGrounded;
    protected Vector3 _lastPosition;
    protected float _episodePathLength;
    protected float _shortestPathLength;
    protected FetchGamePhysicsTrainingArena.TaskType _taskType;
    protected GameObject _targetContainer;

    /// <summary>
    /// Called after the Setup function for <see cref="FetchGamePhysicsTrainingArena"/>).
    /// </summary>
    /// <param name="helper"></param>
    public override void Setup(Janelia.IEasyMLSetupHelper helper)
    {
        // Before calling `EasyMLAgentGrounded.Setup` (as `base.Setup`) override some parameters
        // it will use.
        UseChildSensorForward = false;
        ChildSensorForwardRayLength = GetTurfDiameter();
        BodyColor = "#4b3c39";

        base.Setup(helper);

        gameObject.name = "AgentJump";

        // Use the ceiling camera for observation.
        CameraSensorComponent cameraSensor = gameObject.GetComponent<CameraSensorComponent>();
        if (cameraSensor == null)
        {
            cameraSensor = gameObject.AddComponent<CameraSensorComponent>();
        }
        if (cameraSensor.Camera == null)
        {
            GameObject ceilingCamera = GameObject.Find("AgentCamera");
            cameraSensor.Camera = ceilingCamera.GetComponent<Camera>();
            cameraSensor.Grayscale = true;
            cameraSensor.Width = 64;
            cameraSensor.Height = 64;
            cameraSensor.ObservationStacks = 2; // Use two stacked observations; have tried 4 but it was too slow.
        }

        // Set the field of view degree (single direction) from the MaxRayDegree used by the raySensor.
        GameObject raySensor = GameObject.Find("RaysForward");
        fieldOfViewDegree = raySensor != null ? raySensor.GetComponent<RayPerceptionSensorComponent3D>().MaxRayDegrees : 30;
        
        // Make the agent camera view be consistent with the actual field of view set.
        Camera agentCamera = GameObject.Find("AgentCamera").GetComponent<Camera>();
        agentCamera.fieldOfView = fieldOfViewDegree / agentCamera.aspect * 2;
        
        // Freeze the x and z rotation of the agent so that it doesn't flip over.
        _agentRigidbody = GetComponent<Rigidbody>();
        _agentRigidbody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        moveForce = 7.0f;
        jumpForce = 2.0f;
    }

    /// <summary>
    /// Called every time the Agent receives an action to take. Receives the action chosen by the Agent. 
    /// It is also common to assign a reward in this method.
    /// </summary>
    /// <param name="actions">Action to take</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // The `EasyMLAgentGround` base class handles everything except the assignment of rewards.
        base.OnActionReceived(actions);

        // Make sure the agent only jumps when it is grounded and only jump once.
        if (actions.DiscreteActions[0] == 1 && CheckGrounded())
        {
            _agentRigidbody.AddForce(transform.up * jumpForce, ForceMode.Impulse);
            _isGrounded = false;
        }

        // Count the distance traveled since the last timestep and update the last position.
        _episodePathLength += Vector3.Distance(transform.position, _lastPosition);
        _lastPosition = transform.position;

        // https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Learning-Environment-Design-Agents.md#rewards-summary--best-practices
        // "If you want the agent to finish a task quickly, it is often helpful to provide a small penalty every step
        // (-0.05) that the agent does not complete the task. In this case completion of the task should also coincide
        // with the end of the episode by calling EndEpisode() on the agent when it has accomplished its goal."
        // Also, the total penalty over all the steps should not exceed 1.

        float pathLengthPenaltyProportion = Academy.Instance.EnvironmentParameters.GetWithDefault("path_length_penalty", 0.8f);
        float perStepPenalty = -1.0f * (1 - pathLengthPenaltyProportion) / (float)MaxStep;
        AddReward(perStepPenalty);

        if (_ball != null)
        {
            float ballFetchedThreshold = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_fetched_threshold", 0.0f);
            float thresholdDistance = ballFetchedThreshold * GetTurfDiameter();
            float distanceToBall = Vector3.Distance(transform.position, _ball.transform.position);

            if (distanceToBall < thresholdDistance)
            {
                Debug.Log(transform.parent.name + " distance " + distanceToBall + " is within ball_fetched_threshold distance " + thresholdDistance);
                AddFetchedReward();
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        base.Heuristic(actionsOut);
        int jump = 0;

        if (Input.GetKey(KeyCode.Space))
        {
            jump = 1;
        }

        ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
        discreteActions[0] = jump;
    }

    /// <summary>
    /// Collects vector observations from the environment.
    /// </summary>
    /// <param name="sensor">The vector sensor</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        if (VectorObservationSize == 0)
        {
            return;
        }
        
        float ballObserved = IsBallObservable() ? 1 : 0;
        if ((_ball == null) || (_ballRigidBody == null) || (ballObserved == 0))
        {
            sensor.AddObservation(new float[VectorObservationSize]);
            return;
        }

        // Indicator that encodes if the agent can see the ball with 1 indicating yes and 0 if not.
        sensor.AddObservation(ballObserved);

        // Normalize observations to [0, 1] or [-1, 1]
        // https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Learning-Environment-Design-Agents.md#normalization

        Vector3 toBall = _ball.transform.position - transform.position;
        float angleForward = SignedAngleNormalized(transform.forward, toBall);
        // One observation
        sensor.AddObservation(angleForward);

        // One observation
        // Dividing makes it a relative distance
        float turfDiameter = GetTurfDiameter();
        sensor.AddObservation(toBall.magnitude / turfDiameter);

        Vector3 ballVelocity = _ballRigidBody.velocity;

        // Angle will be 0 if the ball is moving directly away from the agent,
        // or if the ball is not moving.  Angle will be negative if the ball is moving
        // to the left of the agent's forward direction, and positive for the right.
        // Angle will be 1 (or maybe -1) if the ball is moving directly towards the agent.
        float angleVelocity = SignedAngleNormalized(transform.forward, ballVelocity);
        // One observation
        sensor.AddObservation(angleVelocity);

        // Normalize
        float agentSpeed = _agentRigidbody.velocity.magnitude / turfDiameter;
        float ballSpeed = ballVelocity.magnitude / turfDiameter;

        // But this normalization may make the values very small, so use a heuristic to increase them
        float speedScale = 4.0f;
        agentSpeed = Mathf.Clamp01(agentSpeed * speedScale);
        ballSpeed = Mathf.Clamp01(ballSpeed * speedScale);

        // One observation
        sensor.AddObservation(agentSpeed);
        // One observation
        sensor.AddObservation(ballSpeed);
    }

    /// <summary>
    /// Called on the frame when a script is enabled just before any of the Update methods 
    /// are called the first time.
    /// </summary>
    private void Start()
    {
        Debug.Log(transform.parent.name + " FetchAgent.Start");

        GameObject arena = transform.parent.gameObject;
        _ball = Janelia.EasyMLRuntimeUtils.FindChildWithTag(arena, FetchGamePhysicsTrainingArena.TAG_BALL);

        _ballRigidBody = _ball.GetComponent<Rigidbody>();
        
        // Set the field of view degree (single direction) from the MaxRayDegree used by the raySensor.
        GameObject raySensor = GameObject.Find("RaysForward");
        fieldOfViewDegree = raySensor != null ? raySensor.GetComponent<RayPerceptionSensorComponent3D>().MaxRayDegrees : 30;
    }

    /// <summary>
    /// Called when the agent collides with another scene object.
    /// </summary>
    /// <param name="collision">The collision info</param>
    private void OnCollisionEnter(Collision collision)
    {
        Collider c = collision.collider;

        if (c.CompareTag(FetchGamePhysicsTrainingArena.TAG_GROUND) || c.CompareTag(FetchGamePhysicsTrainingArena.TAG_RAMP) || c.CompareTag(FetchGamePhysicsTrainingArena.TAG_OBSTACLE))
        {
            // Agent is grounded.
            _isGrounded = true;
        }

        if (c.CompareTag(FetchGamePhysicsTrainingArena.TAG_BALL))
        {
            // Disable any collision response that might put the ball or agent in a bad position.
            _ballRigidBody.Sleep();
            _agentRigidbody.Sleep();

            if (trainingMode)
            {
                AddFetchedReward();
            }
        }

        // Penalize the agent for colliding with the wrong container.
        if (_taskType == FetchGamePhysicsTrainingArena.TaskType.containment)
        {
            if (c.CompareTag(FetchGamePhysicsTrainingArena.TAG_CONTAINER) && c.gameObject != _targetContainer)
            {
                SetReward(-1.0f);
                Debug.Log(transform.parent.name + " FetchAgent.OnCollisionEnter: Penalize for colliding with wrong container and end episode.");
                EndEpisode();
            }
        }

        // TODO: There is no penalty for a collision with the obstacle, to keep training from being
        // too difficult.  Is this approach right?  Should there be no penalty for any collision?
        if (c.CompareTag(FetchGamePhysicsTrainingArena.TAG_BOUNDARY) || c.CompareTag(FetchGamePhysicsTrainingArena.TAG_RAMP))
        {
            if (trainingMode)
            {
                Debug.Log(transform.parent.name + " adding collision penalty");
                AddReward(-0.5f);
            }
        }
    }

    // private void OnCollisionStay(Collision collision)
    // {
    //     if (collision.collider.CompareTag(FetchGamePhysicsTrainingArena.TAG_OBSTACLE))
    //     {
    //         // When colliding with the obstacle, hack the movement force direction to make
    //         // the agent slide along the obstacle instead of getting stuck on it.  The algorithm
    //         // assumes the obstacle is a plane.
    //         Vector3 v = Vector3.ProjectOnPlane(transform.forward, collision.gameObject.transform.right);
    //         v.Normalize();
    //         _moveForwardDirection = v;
    //     }
    // }

    // private void OnCollisionExit(Collision collision)
    // {
    //     if (collision.collider.CompareTag(FetchGamePhysicsTrainingArena.TAG_OBSTACLE))
    //     {
    //         // When there is no more collison with the obstacle, go back to the standard 
    //         // movement force direction.
    //         _moveForwardDirection = Vector3.zero;
    //     }
    // }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        _isGrounded = true;

        // Stop agent's movement.
        _agentRigidbody.velocity = Vector3.zero;
        _agentRigidbody.angularVelocity = Vector3.zero;

        // Record the last position of the agent and reset the path length record.
        _lastPosition = transform.position;
        _episodePathLength = 0;

        // Get current task type.
        _taskType = GetTaskType();

        // Get the target container if applicable.
        FetchGamePhysicsTrainingArena fetchArena = GetComponentInParent<FetchGamePhysicsTrainingArena>();
        if (_taskType == FetchGamePhysicsTrainingArena.TaskType.containment)
        {
            _targetContainer = fetchArena.targetContainer;
        }

        _shortestPathLength = fetchArena.GetShortestPathLength();
    }

    private void AddFetchedReward()
    {
        Vector3 toBall = (_ball.transform.position - transform.position);
        float speedBonusProportion = Academy.Instance.EnvironmentParameters.GetWithDefault("speed_bonus", 0.8f);
        float orientationBonus = 0.5f * (1 - speedBonusProportion) * Mathf.Clamp01(Vector3.Dot(transform.forward.normalized, toBall.normalized));
        float speedBonus = 0.5f * speedBonusProportion * (1 - StepCount / MaxStep);
        AddReward(0.5f + orientationBonus + speedBonus);
        
        // Penalize for the length of the path taken.
        float pathLengthPenaltyProportion = Academy.Instance.EnvironmentParameters.GetWithDefault("path_length_penalty", 0.8f);
        float standardizedPathLength = Mathf.Clamp((_episodePathLength - _shortestPathLength) / GetTurfDiameter(), 0, 1);
        AddReward(-pathLengthPenaltyProportion * standardizedPathLength);
        
        Debug.Log(transform.parent.name + " successfully complete task with reward: " + GetCumulativeReward() + " and path length: " + _episodePathLength + " with a shortest path length " + _shortestPathLength);
        EndEpisode();
    }

    private float GetTurfDiameter()
    {
        FetchGamePhysicsTrainingArena fetchArena = GetComponentInParent<FetchGamePhysicsTrainingArena>();
        return (fetchArena != null) ? 2.0f * fetchArena.TurfRadius : 1.0f;
    }

    private FetchGamePhysicsTrainingArena.TaskType GetTaskType()
    {
        FetchGamePhysicsTrainingArena fetchArena = GetComponentInParent<FetchGamePhysicsTrainingArena>();
        return fetchArena.taskType;
    }

    private float SignedAngleNormalized(Vector3 a, Vector3 b)
    {
        return Vector3.SignedAngle(a, b, transform.up) / 180;
        // // [0, 180] for left or right
        // float angleBetween = Vector3.Angle(a.normalized, b.normalized);
        // // Negative for left
        // Vector3 cross = Vector3.Cross(a, b);
        // float sign = Mathf.Sign(Vector3.Dot(cross, transform.up));
        // return sign * angleBetween / 180;
    }

    private bool IsBallObservable()
    {
        Vector3 rayInit = transform.TransformPoint(BodyScale / 3);
        Vector3 toBall = _ball.transform.position - rayInit;
        float distance = toBall.magnitude - _ball.transform.localScale.x;
        float angleToBall = Vector3.SignedAngle(transform.forward, toBall, transform.up);
        bool atFront = angleToBall < fieldOfViewDegree && angleToBall > -fieldOfViewDegree;
        RaycastHit hit;
        bool blocked = Physics.Raycast(rayInit, toBall, out hit, distance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
        if (blocked) Debug.Log("Raycast hit " + hit.collider);
        return atFront && !blocked;
    }

    public bool CheckGrounded()
    {
        return _isGrounded;
    }
}
