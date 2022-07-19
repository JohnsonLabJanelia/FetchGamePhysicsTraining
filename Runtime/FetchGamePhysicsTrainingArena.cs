using Unity.MLAgents;
using UnityEngine;

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

    public float TurfRadius
    {
        get { return _turfRadius; }
    }
    private float _turfRadius = 0.0f;
    private float _turfY = 0.0f;
    private float _turfThickness = 0.0f;

    private Vector3 _rampSize = Vector3.zero;

    private float _ballRadius = 0.0f;

    public static readonly float RAMP_ANGLE_WIGGLE_DEGS = 10.0f;
    public static readonly float AGENT_EASY_CASE_PROBABILITY = 0.0f;

    public static readonly float OBSTACLE_ANGLE_WIGGLE_DEGS = 25.0f;

    public bool ballOnRamp = true;
    public bool ballInitVel = false;
    public float maxBallInitVelMagnitude = 2.5f;

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
        
        Reparent();
        FindTurfMetrics();
        FindRampMetrics();
        FindBallMetrics();

        string title = "FetchGamePhysicsTraining Setup";
        string message = "Use an obstacle during training?";
        bool useObstacle = helper.DisplayDialog(title, message, "Yes", "No");
        if (useObstacle)
        {
            if ((Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_OBSTACLE) == null)) CreateObstacle();
        }
        else
        {
            DestroyObstacle();
        }

        message = "Spawn the ball on ramp?";
        ballOnRamp = helper.DisplayDialog(title, message, "Yes", "No");

        message = "Give the ball a random horizontal velocity?";
        ballInitVel = helper.DisplayDialog(title, message, "Yes", "No");

        if (!name.StartsWith("TrainingArena"))
        {
            name = "TrainingArena";
        }
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
        Reparent("HollowCyl");
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
        Deactivate("HollowCyl");
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
        PlaceRamp();
        PlaceBall();
        PlaceAgent();
        PlaceObstacle();
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

    private void PlaceBall()
    {
        GameObject ball = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_BALL);
        if (ball != null)
        {
            Rigidbody ballRigidbody = ball.GetComponent<Rigidbody>();
            if (ballRigidbody != null)
            {
                float maxVel = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_max_angular_velocity", 7.0f);
                ballRigidbody.maxAngularVelocity = maxVel;

                if (ballInitVel)
                {
                    float angle = UnityEngine.Random.Range(0, 360);
                    float speed = UnityEngine.Random.Range(1.0f, maxBallInitVelMagnitude);
                    Vector3 v = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)).MultiplyVector(Vector3.forward);
                    v *= speed;
                    ballRigidbody.velocity = v;
                }
            }

            GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);
            if ((ramp != null) && (ballOnRamp))
            {
                Vector3 r = ramp.transform.localEulerAngles;
                ball.transform.localEulerAngles = new Vector3(0, r.y, 0);

                Vector3 p = ramp.transform.localPosition;
                p.y += _rampSize.y + _ballRadius;
                // Minus because `ramp.transform.forward` points out from the turf center.
                p -= ramp.transform.forward * 4 * _ballRadius;
                ball.transform.localPosition = p;
            } else
            {
                float angle = UnityEngine.Random.Range(0, 360);
                float distance = UnityEngine.Random.Range(0, _turfRadius * 0.95f);
                ball.transform.localEulerAngles = new Vector3(0, angle, 0);
                ball.transform.localPosition = new Vector3(0, ramp.transform.localPosition.y + _rampSize.y + 2 * _ballRadius, distance);
            }
        }
    }

    private void PlaceAgent()
    {
        GameObject agent = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, Janelia.EasyMLAgent.TAG_AGENT);
        if (agent != null)
        {
            Transform agentBody = agent.transform.Find("Body");
            Vector3 agentScale = (agentBody != null) ? agentBody.localScale : Vector3.one;
            GameObject ramp = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_RAMP);

            bool safe = false;
            float angle = 0;
            int attempts = 0;
            GameObject ball = Janelia.EasyMLRuntimeUtils.FindChildWithTag(gameObject, TAG_BALL);
            float ballFetchedThreshold = Academy.Instance.EnvironmentParameters.GetWithDefault("ball_fetched_threshold", 0.02f);
            float thresholdDistance = ballFetchedThreshold * TurfRadius * 2.0f;
            
            while (!safe && (attempts++ < 100))
            {
                angle = UnityEngine.Random.Range(0, 360);
                Vector3 p = Vector3.zero;
                float padding = Mathf.Max(agentScale.x, agentScale.z);
                if ((ramp != null) && (UnityEngine.Random.value < AGENT_EASY_CASE_PROBABILITY))
                {
                    p = ramp.transform.localPosition;
                    float d = UnityEngine.Random.Range(_rampSize.z, p.magnitude + _turfRadius - padding);
                    // Minus because `ramp.transform.forward` points out from the turf center.
                    Vector3 q = -ramp.transform.forward;
                    p += d * q;
                }
                else
                {
                    p = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)).MultiplyVector(Vector3.forward);
                    p.y = ramp.transform.localPosition.y;
                    float radius = UnityEngine.Random.Range(0, _turfRadius - padding);
                    p *= radius;
                }
                p.y = _turfY + _turfThickness / 2;
                agent.transform.localPosition = p;

                // Continue trying placements until there is a safe configuration, with the agent's
                // body not overlapping the ramp.
                float rampPadding = Mathf.Max(_rampSize.x, _rampSize.z);
                float distanceToBall = Vector3.Distance(agent.transform.position, ball.transform.position);
                safe = (Vector3.Distance(ramp.transform.localPosition, p) > padding + rampPadding) & (distanceToBall > thresholdDistance);
            }

            float angleLocalY = angle + 180;
            if (angleLocalY > 360)
            {
                angleLocalY -= 360;
            }
            agent.transform.localEulerAngles = new Vector3(0, angleLocalY, 0);
        }
    }

    private void PlaceObstacle()
    {
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
        obstacle.transform.localScale = scale;

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
            Vector3 agentScale = (agentBody != null) ? agentBody.localScale : Vector3.one;

            Vector3 offset = toAgentRight;
            offset += toAgentRight.normalized * (obstacle.transform.localScale.x + agentScale.x);
            p += offset;
        }

        obstacle.transform.localPosition = p;
        
        float angle = 0;
        int attempts = 0;
        while (attempts++ < 100)
        {
            angle = UnityEngine.Random.Range(0, 360);
            Quaternion q = Quaternion.Euler(0, angle + obstacle.transform.localRotation.y, 0);
            if (Physics.OverlapBox(obstacle.transform.position, obstacle.transform.localScale / 2, q, Physics.DefaultRaycastLayers).Length == 0)
            {
                
                break;
            }
        }
        obstacle.transform.Rotate(0, angle, 0);
    }

    private GameObject CreateObstacle()
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = "Obstacle";
        obstacle.tag = TAG_OBSTACLE;
        obstacle.transform.parent = transform;

        obstacle.transform.localScale = new Vector3(_rampSize.x * 0.167f, _rampSize.y, _rampSize.z * 0.67f);

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
}
