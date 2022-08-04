using Unity.MLAgents;
using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// An arena for training an agent in the fetch game:
/// <seealso href="https://github.com/JohnsonLabJanelia/FetchGamePhysics/tree/main/FetchArenaProject"/>
/// </summary>
public class FetchGamePhysicsTrainingArena : Janelia.EasyMLArena
{
    public static readonly string TAG_BALL = "Ball";
    public static readonly string TAG_BOUNDARY = "Boundary";
    public static readonly string TAG_RAMP = "Ramp";
    public static readonly string TAG_GROUND = "Ground";
    public static readonly string TAG_OBSTACLE = "Obstacle";
    public static readonly string TAG_CONTAINER = "Container";

    public float TurfRadius
    {
        get { return _turfRadius; }
    }
    private float _turfRadius = 0.0f;
    private float _turfY = 0.0f;
    private float _turfThickness = 0.0f;

    private Vector3 _rampSize = Vector3.zero;

    private float _ballRadius = 0.0f;
    private float _obstacleHeight = 0.0f;
    private Vector3 _agentScale;

    public static readonly float RAMP_ANGLE_WIGGLE_DEGS = 10.0f;
    public static readonly float AGENT_EASY_CASE_PROBABILITY = 0.0f;

    public static readonly float OBSTACLE_ANGLE_WIGGLE_DEGS = 25.0f;

    public bool ballOnRamp;
    public bool ballInitVel = false;
    public float maxBallInitVelMagnitude = 1.5f;
    public float minBallInitVelMagnitude = 0.5f;
    public int numBalls = 1;
    public enum TaskType { obstacleOneBall, efficientForaging, containment };
    public TaskType taskType { get; protected set; } = TaskType.obstacleOneBall;
    protected System.Random _rng = new System.Random();
    protected bool _useContainer = false;
    protected int numContainers = 1;
    public GameObject targetContainer { get; protected set; }
    // These convertion factors for the hollowCyl
    // are manually measured from the Unity editor.
    protected float _hollowCylInnerRadius = 0.1f / 1.5f;
    protected float _hollowCylOuterRadius = 0.1f / 1.1f;
    protected float _hollowCylHeight = 0.1f / 0.75f;

    /// <summary>
    /// Performs the initial setup of the objects involved in training (except for
    /// <see cref="FetchGamePhysicsTrainingAgent"/> which has its own Setup function,
    /// called after this one is called).
    /// </summary>
    /// <param name="helper">A class with helper functions for tasks like adding tags or 
    /// creating materials</param>
    public override void Setup(Janelia.IEasyMLSetupHelper helper)
    {
        base.Setup(helper);

        helper.CreateTag(TAG_BALL);
        helper.CreateTag(TAG_BOUNDARY);
        helper.CreateTag(TAG_RAMP);
        helper.CreateTag(TAG_GROUND);
        helper.CreateTag(TAG_OBSTACLE);
        helper.CreateTag(TAG_CONTAINER);
        
        Reparent();
        FindTurfMetrics();
        FindRampMetrics();
        FindBallMetrics();
        FindAgentMetrics();
        UpdateContainerMetrics(GameObject.FindGameObjectWithTag(TAG_CONTAINER));

        string title = "FetchGamePhysicsTraining Setup";
        string message;
        // message = "Use an obstacle during training?";
        // bool useObstacle = helper.DisplayDialog(title, message, "Yes", "No");
        // if (useObstacle)
        // {
            // if ((Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_OBSTACLE) == null)) CreateObstacle();
        // }
        // else
        // {
        //     DestroyObstacle();
        // }

        if ((Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_OBSTACLE) == null)) CreateObstacle();
        
        message = "Give the ball a random horizontal velocity?";
        ballInitVel = helper.DisplayDialog(title, message, "Yes", "No");

        if (!name.StartsWith("TrainingArena"))
        {
            name = "TrainingArena";
        }

        // Disable shadows from the light source.
        GameObject.Find("Directional Light").GetComponent<Light>().shadows = LightShadows.None;
    }

    private void Reparent()
    {
        GameObject simpleArena = Reparent("SimpleArena");
        if (simpleArena != null)
        {
            Transform wallTransform = simpleArena.transform.Find("Wall");
            if (wallTransform != null)
            {
                wallTransform.gameObject.tag = TAG_BOUNDARY;
            }
            Transform turfTransform = simpleArena.transform.Find("Turf");
            if (turfTransform != null)
            {
                turfTransform.gameObject.tag = TAG_GROUND;
            }
        }

        GameObject ramp = Reparent("ramp_unit");
        if (ramp != null)
        {
            ramp.tag = TAG_RAMP;
        }

        Reparent("Table");
        
        GameObject hollowCyl = Reparent("HollowCyl");
        if (hollowCyl != null)
        {
            hollowCyl.tag = TAG_CONTAINER;
        }

        GameObject sphere = Reparent("Sphere");
        if (sphere != null)
        {
            sphere.tag = TAG_BALL;
        }

        Reparent("Floor");

        Reparent("Bumblebee");
        Reparent("Grimlock");
        Reparent("Optimus");

        // The "FetchGamPhysics" project does have this mispelling in it.
        Reparent("ExprimenterView");

        CleanupUnused();

        FixMeshCollider(TAG_GROUND);
        FixMeshCollider(TAG_BOUNDARY);
    }

    private GameObject Reparent(string childName)
    {
        Transform childTransform = transform.Find(childName);
        if (childTransform != null)
        {
            return childTransform.gameObject;
        }
        GameObject toReparent = GameObject.Find(childName);
        if (toReparent != null)
        {
            toReparent.transform.parent = transform;
            return toReparent;
        }
        return null;
    }

    private void CleanupUnused()
    {
        Deactivate("Cube");
        Deactivate("Goal");
        Deactivate("Publisher");
        Deactivate("Canvas");
        Deactivate("EventSystem");
        Deactivate("Bumblebee");
        Deactivate("Grimlock");
        Deactivate("Optimus");
        Deactivate("ExprimenterView");
        Deactivate("Table");
        Deactivate("VentionFrame");
    }

    private void Deactivate(string name)
    {
        GameObject obj = GameObject.Find(name);
        if (obj != null)
        {
            obj.SetActive(false);
        }
    }

    private void FixMeshCollider(string tag)
    {
        GameObject obj = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, tag);
        if (obj != null)
        {
            MeshCollider collider = obj.GetComponent<MeshCollider>() as MeshCollider;
            collider.enabled = true;

            // The ball passes through the boundary if the boundary collider is marked convex,
            // perhaps because the ball starts out inside that convex region, Unity ignores
            // internal collisions.
            collider.convex = false;
        }
    }

    public override void PlaceRandomly()
    {
        FindTurfMetrics();
        FindRampMetrics();
        FindBallMetrics();
        FindAgentMetrics();

        // // Set the task type randomly at the beginning of each trial.
        // _taskType = (TaskType) Random.Range(0, System.Enum.GetValues(typeof(TaskType)).Length);
        
        // Set the task type deterministically at the beginning of each trial.
        taskType = TaskType.containment;

        Debug.Log("Setting up task type: " + taskType.ToString());
        UpdateParameters();

        PlaceRamp();
        PlaceAgent();
        PlaceBall();
        PlaceContainer();
        PlaceObstacle();
    }

    private void UpdateParameters()
    {
        switch (taskType)
        {
            case TaskType.obstacleOneBall:
                numBalls = 1;
                ballOnRamp = true;
                _obstacleHeight = _rampSize.y;
                _useContainer = false;
                break;
            case TaskType.efficientForaging:
                numBalls = 2;
                ballOnRamp = false;
                _obstacleHeight = 2.1f * _ballRadius;
                _useContainer = false;
                break;
            case TaskType.containment:
                numBalls = 1;
                ballOnRamp = false;
                _obstacleHeight = 0f;
                _useContainer = true;
                numContainers = (int) Academy.Instance.EnvironmentParameters.GetWithDefault("num_containers", 3);
                break;
        }

        // Disable the components of ramp if not used. The gameobject is not
        // deactivated because it's difficult to retrieve for future use.
        GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);
        ramp.GetComponent<MeshCollider>().enabled = ballOnRamp;
        ramp.GetComponent<MeshRenderer>().enabled = ballOnRamp;

        // Disable the parts of the container if not used.
        List<GameObject> containers = Janelia.EasyMLRuntimeUtils.FindChildrenWithTag(gameObject, TAG_CONTAINER);
        foreach (GameObject container in containers)
        {
            foreach(Transform part in container.transform.GetComponentsInChildren<Transform>(includeInactive: true))
            {
                if (part.gameObject != container)
                {
                    part.gameObject.SetActive(_useContainer);
                }
            }
        }

        bool _useObstacle = _obstacleHeight != 0;
        GameObject obstacle = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_OBSTACLE);
        if (obstacle != null)
        {
            obstacle.GetComponent<BoxCollider>().enabled = _useObstacle;
            obstacle.GetComponent<MeshRenderer>().enabled = _useObstacle;
        }
    }

    private void FindTurfMetrics()
    {
        GameObject turf = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_GROUND);
        if (turf != null)
        {
            _turfY = turf.transform.localPosition.y;

            MeshFilter turfMeshFilter = turf.GetComponent<MeshFilter>() as MeshFilter;
            if ((turfMeshFilter != null) && (turfMeshFilter.sharedMesh != null))
            {
                Vector3 turfSize = turfMeshFilter.sharedMesh.bounds.size;
                _turfRadius = turfSize.x / 2.0f;
                // When placed in the scene, the turf object is rotated -90 around x, so
                // its thickness is the original z dimension, which becomes the y dimension.
                _turfThickness = turfSize.z;
            }
        }
    }

    private void FindRampMetrics()
    {
        GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);
        if (ramp != null)
        {
            MeshFilter rampMeshFilter = ramp.GetComponent<MeshFilter>() as MeshFilter;
            if ((rampMeshFilter != null) && (rampMeshFilter.sharedMesh != null))
            {
                Vector3 rampScale = ramp.transform.localScale;
                _rampSize = Vector3.Scale(rampMeshFilter.sharedMesh.bounds.size, rampScale);
            }
        }
    }

    private void FindBallMetrics()
    {
        GameObject ball = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_BALL);
        if (ball != null)
        {
            _ballRadius = ball.transform.localScale.x / 2;
        }
    }

    private void FindAgentMetrics()
    {
        GameObject agent = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, Janelia.EasyMLAgent.TAG_AGENT);
        Transform agentBody = agent.transform.Find(agent.GetComponent<FetchGamePhysicsTrainingAgent>().BodyName);
        _agentScale = (agentBody != null) ? agentBody.localScale : Vector3.one;
    }

    private void PlaceRamp()
    {
        GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);
        if (ramp != null)
        {
            float angle = UnityEngine.Random.Range(0, 360);
            float radius = UnityEngine.Random.Range(_turfRadius / 3, _turfRadius);

            Vector3 p = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)).MultiplyVector(Vector3.forward);
            p *= radius;
            p.y = ramp.transform.localPosition.y;
            ramp.transform.localPosition = p;

            float angleLocalY = angle;
            angleLocalY += UnityEngine.Random.Range(-RAMP_ANGLE_WIGGLE_DEGS, RAMP_ANGLE_WIGGLE_DEGS);
            if (angleLocalY > 360)
            {
                angleLocalY -= 360;
            }
            ramp.transform.localEulerAngles = new Vector3(0, angleLocalY, 0);
        }
    }

    private void PlaceAgent()
    {
        GameObject agent = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, Janelia.EasyMLAgent.TAG_AGENT);
        if (agent != null)
        {
            // Reset the agent's velocity.
            Rigidbody agentRigidbody = agent.GetComponent<Rigidbody>() as Rigidbody;
            if (agentRigidbody != null)
            {
                agentRigidbody.velocity = Vector3.zero;
                agentRigidbody.angularVelocity = Vector3.zero;
            }
            
            Transform agentBody = agent.transform.Find("Body");
            GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);

            bool safe = false;
            float angle = 0;
            int attempts = 0;
            Transform agentTransform = agent.transform;
            
            while (!safe && (attempts++ < 100))
            {
                angle = UnityEngine.Random.Range(0, 360);
                Vector3 p = Vector3.zero;
                float padding = Mathf.Max(_agentScale.x, _agentScale.z);
                if ((ramp != null) && (UnityEngine.Random.value < AGENT_EASY_CASE_PROBABILITY))
                {
                    p = ramp.transform.localPosition;
                    // The range of agent position.
                    float d = 0f;
                    switch (taskType)
                    {
                        case TaskType.obstacleOneBall:
                            d = UnityEngine.Random.Range(_rampSize.z, p.magnitude + _turfRadius - padding);
                            break;
                        default:
                            d = _turfRadius - padding;
                            break;
                    } 
                    // Minus because `ramp.transform.forward` points out from the turf center.
                    Vector3 q = -ramp.transform.forward;
                    p += d * q;
                }
                else
                {
                    p = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)).MultiplyVector(Vector3.forward);
                    p.y = ramp.transform.localPosition.y;
                    float radius = 0f;
                    switch (taskType)
                    {
                        case TaskType.obstacleOneBall:
                            radius = UnityEngine.Random.Range(0, _turfRadius - padding);
                            break;
                        default:
                            radius = _turfRadius - padding;
                            break;
                    }
                    p *= radius;
                }
                p.y = _turfY + _turfThickness / 2;
                agentTransform.localPosition = p;

                // Continue trying placements until there is a safe configuration, with the agent's
                // body not overlapping the ramp.
                float rampPadding = Mathf.Max(_rampSize.x, _rampSize.z);
                
                // Use OverlapBox to check if it's safe to place the agent.
                // Collider[] colliders = CheckOverlap(agentTransform.position, agent.GetComponent<FetchGamePhysicsTrainingAgent>().BodyScale, Quaternion.identity);
                // safe = colliders.Length == 1; // 1 because the turf is always detected.

                // // Use distance to check if it's safe to place the agent.
                safe = Vector3.Distance(ramp.transform.localPosition, p) > padding + rampPadding;
            }

            // Make the agent face the center of the arena.
            float angleLocalY = angle + 180;
            if (angleLocalY > 360)
            {
                angleLocalY -= 360;
            }
            agent.transform.localEulerAngles = new Vector3(0, angleLocalY, 0);
        }
    }

    private void PlaceBall()
    {
        // If the number of balls doesn't match the number of balls, create or destroy balls.
        List<GameObject> balls = Janelia.EasyMLRuntimeUtils.FindChildrenWithTag(gameObject, TAG_BALL);
        if (balls.Count > numBalls)
        {
            while (balls.Count > numBalls && balls.Count > 1)
            {
                GameObject ballToRemove = balls[balls.Count - 1];
                balls.RemoveAt(balls.Count - 1);
                #if UNITY_EDITOR
                DestroyImmediate(ballToRemove);
                #else
                Destroy(ballToRemove);
                #endif
            }
        }else if (balls.Count < numBalls)
        {
            GameObject ballOriginal = balls[0];
            while (balls.Count < numBalls)
            {
                GameObject ball = GameObject.Instantiate(ballOriginal);
                ball.transform.parent = transform;
                ball.tag = TAG_BALL;
                balls.Add(ball);
            }
        }
        if (balls != null)
        {
            foreach (GameObject ball in balls)
            {
                Rigidbody ballRigidbody = ball.GetComponent<Rigidbody>();

                if (ballRigidbody != null)
                {
                    // Reset the ball's velocity.
                    ballRigidbody.velocity = Vector3.zero;
                    ballRigidbody.angularVelocity = Vector3.zero;

                    float maxVel = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_max_angular_velocity", 7.0f);
                    ballRigidbody.maxAngularVelocity = maxVel;

                    if (ballInitVel)
                    {
                        float angle = UnityEngine.Random.Range(0, 360);
                        float speed = UnityEngine.Random.Range(minBallInitVelMagnitude, maxBallInitVelMagnitude);
                        Vector3 v = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)).MultiplyVector(Vector3.forward);
                        v *= speed;
                        ballRigidbody.velocity = v;
                    }
                }

                Transform ballTransform = ball.transform;
                GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);
                Vector3 p;
                switch (taskType)
                {
                    case TaskType.obstacleOneBall:
                        Vector3 r = ramp.transform.localEulerAngles;
                        ballTransform.localEulerAngles = new Vector3(0, r.y, 0);
                        p = ramp.transform.localPosition;
                        // Move ball to the center of the ramp.   
                        p.y += _rampSize.y + _ballRadius;
                        p -= ramp.transform.forward * 4 * _ballRadius;
                        // Minus because `ramp.transform.forward` points out from the turf center.
                        ballTransform.localPosition = p;
                        break;
                    default:
                        // float angle = UnityEngine.Random.Range(0, 360);
                        // float distance = UnityEngine.Random.Range(0, _turfRadius * 0.95f);
                        // ballTransform.localEulerAngles = new Vector3(0, angle, 0);
                        // ballTransform.localPosition = new Vector3(0, ramp.transform.localPosition.y + _rampSize.y + 2 * _ballRadius, distance);
                        GameObject agent = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, Janelia.EasyMLAgent.TAG_AGENT);
                        Transform agentTransform = agent.transform;
                        float fov = agent.GetComponent<FetchGamePhysicsTrainingAgent>().fieldOfViewDegree;
                        float ballFetchedThreshold = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_fetched_threshold", 0.02f);
                        float thresholdDistance = ballFetchedThreshold * TurfRadius * 2.0f;
                        float angle = 0;
                        float radius = 0;
                        int attempts = 0;
                        bool safe = false;
                        while (attempts++ < 1000 && !safe)
                        { 
                            angle = UnityEngine.Random.Range(0, 360);
                            radius = UnityEngine.Random.Range(0, TurfRadius * 0.8f);
                            p = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)).MultiplyVector(Vector3.forward);
                            p *= radius;
                            p.y = ramp.transform.localPosition.y + 5 * _ballRadius;
                            ballTransform.localPosition = p;
                            Vector3 agentToBall = ballTransform.position - agentTransform.position;
                            agentToBall.y = 0;
                            Collider[] colliders = CheckOverlap(ballTransform.position, ballTransform.localScale, Quaternion.identity);
                            // Make sure the all is placed far away enough from the agent and within the view of the agent so that the agent sees the ball.
                            safe = Vector3.Distance(agentTransform.localPosition, new Vector3(ballTransform.localPosition.x, agentTransform.localPosition.y, ballTransform.localPosition.z)) > thresholdDistance;
                            safe = safe && (Vector3.Angle(agentTransform.forward, agentToBall) < fov);
                            safe = safe && (colliders.Length == 0); // TODO: use distance comparison instead of overlap to check for collision with other balls.
                        }
                        break;
                }
            }
        }
    }

    private void PlaceContainer()
    {
        // If the number of containers doesn't match the number of balls, create or destroy containers.
        List<GameObject> containers = Janelia.EasyMLRuntimeUtils.FindChildrenWithTag(gameObject, TAG_CONTAINER);
        if (containers.Count > numContainers)
        {
            while (containers.Count > numContainers && containers.Count > 1)
            {
                GameObject containerToRemove = containers[containers.Count - 1];
                containers.RemoveAt(containers.Count - 1);
                #if UNITY_EDITOR
                DestroyImmediate(containerToRemove);
                #else
                Destroy(containerToRemove);
                #endif
            }
        }else if (containers.Count < numContainers)
        {
            GameObject containerOriginal = containers[0];
            while (containers.Count < numContainers)
            {
                GameObject container = GameObject.Instantiate(containerOriginal);
                container.transform.parent = transform;
                container.tag = TAG_CONTAINER;
                containers.Add(container);
            }
        }

        if (containers != null)
        {
            List<GameObject> placedContainers = new List<GameObject>();

            // Randomly choose a container to contain the ball.
            int index = _rng.Next(containers.Count);
            targetContainer = containers[index];
            UpdateContainerMetrics(targetContainer);
            GameObject ball = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_BALL);
            Vector3 p = ball.transform.localPosition;
            p.y = targetContainer.transform.localPosition.y;
            targetContainer.transform.localPosition = p;
            containers.RemoveAt(index);
            placedContainers.Add(targetContainer);

            foreach (GameObject container in containers)
            {
                UpdateContainerMetrics(container);
                Transform containerTransform = container.transform;
                float padding = Mathf.Max(containerTransform.localScale.x, containerTransform.localScale.z) * 2f * _hollowCylOuterRadius;
                float angle = 0;
                float radius = 0;
                int attempts = 0;
                bool safe = false;
                while(attempts++ < 1000 && !safe)
                {
                    angle = UnityEngine.Random.Range(0, 360);
                    radius = UnityEngine.Random.Range(0, TurfRadius * 0.8f);
                    p = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)).MultiplyVector(Vector3.forward);
                    p *= radius;
                    p.y = containerTransform.localPosition.y;
                    containerTransform.localPosition = p;

                    // // Physics.OverlapBox based collision detection but this is not good because of the update issue.
                    // Collider[] colliders = CheckOverlap(containerTransform.position, containerTransform.localScale * _hollowCylOuterRadius, Quaternion.identity);
                    // safe = colliders.Length == 1; // 1 because the turf is always detected.

                    // Distance based safe check.
                    safe = true;
                    foreach (GameObject placedContainer in placedContainers)
                    {
                        safe = safe && Vector3.Distance(placedContainer.transform.localPosition, containerTransform.localPosition) > padding;
                    }
                }
                placedContainers.Add(container);
            }
        }
    }

    private void PlaceObstacle()
    {
        // If the number of obstacles doesn't match the number of balls, create or destroy obstacles.
        GameObject obstacle = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_OBSTACLE);
        if (obstacle == null)
        {
            return;
        }

        GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);
        GameObject agent = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, Janelia.EasyMLAgent.TAG_AGENT);
        if ((ramp == null) || (agent == null))
        {
            return;
        }
        
        Vector3 scale = obstacle.transform.localScale;
        float scaleFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("obstacle_scale_vs_ramp", 0.67f);
        scale.z = scaleFactor * _rampSize.z;
        scale.y = _obstacleHeight;
        obstacle.transform.localScale = scale;


        switch (taskType)
        {
            case TaskType.obstacleOneBall:
                float angleLocalY = ramp.transform.localEulerAngles.y;
                /* TODO: Disabled pending detection that the ball path won't be interrupted.
                angleLocalY += UnityEngine.Random.Range(-OBSTACLE_ANGLE_WIGGLE_DEGS, OBSTACLE_ANGLE_WIGGLE_DEGS);
                if (angleLocalY > 360)
                {
                    angleLocalY -= 360;
                }
                */
                obstacle.transform.localEulerAngles = new Vector3(0, angleLocalY, 0);

                // To position the obstacle, start at the ramp's position.
                Vector3 p = ramp.transform.localPosition;
                p.y = obstacle.transform.localPosition.y;
                // Push along the ramp's forward direction until the obstacle is just touching the ramp's front edge.
                // Minus because `ramp.transform.forward` points out from the turf center.
                p -= ramp.transform.forward * (_rampSize.z + obstacle.transform.localScale.z / 2);
                // Push along that forward direction a bit further.
                float d = UnityEngine.Random.Range(0.2f, 0.4f) * _rampSize.z;
                p -= ramp.transform.forward * d;

                // Now push partway along the vector to the agent to get the obstacle out of the way of 
                // the movement of the ball but still in the way of the agent's view of the ball.
                Vector3 toAgent = agent.transform.localPosition - p;
                toAgent.y = 0;
                Vector3 toAgentRight = Vector3.Project(toAgent, ramp.transform.right);
                float min = obstacle.transform.localScale.x * 2;
                min += obstacle.transform.localScale.x;
                if (toAgentRight.magnitude > min)
                {
                    Vector3 offset = UnityEngine.Random.Range(0.25f, 0.75f) * toAgentRight;
                    p += offset;
                }
                else
                {
                    // If there is no room along the vector to the agent, give up on having the obstacle
                    // block the agent's view of the ball.
                    Transform agentBody = agent.transform.Find("Body");

                    Vector3 offset = toAgentRight;
                    offset += toAgentRight.normalized * (obstacle.transform.localScale.x + _agentScale.x);
                    p += offset;
                }

                obstacle.transform.localPosition = p;
                
                float angle = 0;
                int attempts = 0;
                while (attempts++ < 100)
                {
                    angle = UnityEngine.Random.Range(0, 360);
                    Quaternion q = Quaternion.Euler(0, angle + obstacle.transform.localRotation.y, 0);
                    Collider[] colliders = CheckOverlap(obstacle.transform.position, obstacle.transform.localScale, q);
                    if (colliders.Length == 1) // 1 because the turf is always detected.
                    {
                        break;
                    }
                }
                obstacle.transform.Rotate(0, angle, 0);
                break;
            case TaskType.efficientForaging:
                // Make the obstacle less wide so that the other ball is not blocked.
                scale.z = scale.x * 3;
                obstacle.transform.localScale = scale;

                // Randomly choose a ball to block the view of.
                List<GameObject> balls = Janelia.EasyMLRuntimeUtils.FindChildrenWithTag(gameObject, TAG_BALL);
                int index = _rng.Next(balls.Count);
                GameObject ball = balls[index];
                Vector3 agentToBall = ball.transform.localPosition - agent.transform.localPosition;
                agentToBall.y = 0;
                Vector3 obstaclePosition = ball.transform.localPosition - (_ballRadius + obstacle.transform.localScale.x + _agentScale.z) * agentToBall / agentToBall.magnitude;
                obstaclePosition.y = _turfY + _turfThickness / 2 + _obstacleHeight / 2;
                obstacle.transform.localPosition = obstaclePosition;

                // Rotate the obstacle to be perpendicular to the agent's view of the ball.
                angle = -Vector3.SignedAngle(agentToBall, Vector3.forward, Vector3.up) + 90f;
                obstacle.transform.localEulerAngles = new Vector3(0, angle, 0);
                break;
            case TaskType.containment:
                break;
        }
            
    }

    private GameObject CreateObstacle()
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = "Obstacle";
        obstacle.tag = TAG_OBSTACLE;
        obstacle.transform.parent = transform;

        obstacle.transform.localScale = new Vector3(_rampSize.x * 0.167f, _obstacleHeight, _rampSize.z * 0.67f);

        float y = _turfY + _turfThickness / 2 + obstacle.transform.localScale.y / 2;
        obstacle.transform.localPosition = new Vector3(0, y, 0);

        return obstacle;
    }

    private void DestroyObstacle()
    {
        GameObject tagged = GameObject.FindGameObjectWithTag(TAG_OBSTACLE);
        if (tagged != null)
        {
            DestroyImmediate(tagged);
        }
    }

    private void UpdateContainerMetrics(GameObject hollowCyl)
    {
        if (hollowCyl != null)
        {
            Vector3 scale = Vector3.Max(_agentScale, _ballRadius * Vector3.one);
            scale /= _hollowCylInnerRadius;
            scale.y = scale.y * _hollowCylInnerRadius / _hollowCylHeight;
            hollowCyl.transform.localScale = scale *  1.6f; // Multiplied by a factor to make the container easier to jump in.
            hollowCyl.transform.localPosition = new Vector3(0, _turfY + _turfThickness / 2 + 0.5f * _hollowCylHeight * scale.y, 0);

            foreach(Transform part in hollowCyl.transform.GetComponentsInChildren<Transform>(includeInactive: true))
            {
                if (part.gameObject != hollowCyl && part.gameObject.GetComponent<MeshCollider>() == null)
                {
                    part.gameObject.AddComponent<MeshCollider>();
                }
            }
        }
    }

    // TODO: Rewrite everything that uses this method to use distance based overlap checks.
    // This approach of checking for overlap doesn't work because it doesn't reflect the updated position
    // of the game objects. Updating the positions is possible through Physics.Simulate but it doesn't work here
    // because this is in a Physics callback.
    private Collider[] CheckOverlap(Vector3 p, Vector3 s, Quaternion q)
    {
        // Physics.autoSimulation = false; // Required by Physics.Simulate().
        // Physics.Simulate(Time.fixedDeltaTime); // Required to update the new positions.
        Collider[] colliders = Physics.OverlapBox(p, s / 2, q); // Half-extent is used for Physics.OverlapBox.
        // Physics.autoSimulation = true;
        return colliders;
    }

    // Get the shortest distance between the agent and any ball.
    // Used for calculating the path length penalty.
    public float GetShortestPathLength()
    {
        float shortest = 0;
        GameObject agent = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, Janelia.EasyMLAgent.TAG_AGENT);
        List<GameObject> balls = Janelia.EasyMLRuntimeUtils.FindChildrenWithTag(gameObject, TAG_BALL);
        foreach (GameObject ball in balls)
        {
            Vector3 agentToBall = ball.transform.localPosition - agent.transform.localPosition;
            agentToBall.y = 0;
            float distance = agentToBall.magnitude;
            if (distance < shortest || shortest == 0)
            {
                shortest = distance;
            }
        }
        return shortest;
    }
}
