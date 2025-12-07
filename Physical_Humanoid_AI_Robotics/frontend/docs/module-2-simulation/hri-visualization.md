---
sidebar_position: 4
title: 'Unity for HRI Visualization'
---

# Unity for HRI Visualization

This section covers how to use Unity for Human-Robot Interaction (HRI) visualization, providing immersive and interactive interfaces for robotics applications.

## Learning Objectives

After completing this section, you will be able to:
- Set up Unity for robotics visualization applications
- Create interactive 3D environments for robot simulation
- Implement Human-Robot Interaction interfaces in Unity
- Integrate Unity with ROS 2 for real-time data visualization
- Design intuitive user interfaces for robot control and monitoring

## Introduction to Unity for Robotics

Unity is a powerful 3D development platform that has become increasingly popular for robotics visualization due to its real-time rendering capabilities, extensive asset library, and flexible scripting environment.

### Unity Robotics Ecosystem

Unity provides several tools specifically for robotics:
- **Unity Robotics Hub**: Centralized access to robotics packages
- **Unity Robotics Package (URP)**: Core robotics integration
- **Unity ML-Agents**: For training AI agents in simulation
- **ROS# (ROS Sharp)**: Bridge between Unity and ROS
- **Unity Perception**: Tools for generating synthetic training data

### Setting Up Unity for Robotics

To get started with Unity for robotics applications:

1. Download Unity Hub from the Unity website
2. Install Unity version 2021.3 LTS or newer (recommended for robotics)
3. Install required packages through Unity Package Manager
4. Set up the Unity-Rosbridge connection

## Creating Interactive 3D Environments

### Basic Scene Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotEnvironment : MonoBehaviour
{
    [SerializeField] private GameObject robotPrefab;
    [SerializeField] private Transform spawnPoint;

    private RosConnection ros;

    void Start()
    {
        // Initialize ROS connection
        ros = RosConnection.GetOrCreateInstance();
        ros.RegisteredTopicListUpdated.AddListener(OnTopicsUpdated);

        // Spawn robot in environment
        SpawnRobot();
    }

    void SpawnRobot()
    {
        if (robotPrefab != null && spawnPoint != null)
        {
            Instantiate(robotPrefab, spawnPoint.position, spawnPoint.rotation);
        }
    }

    void OnTopicsUpdated(string[] topics)
    {
        Debug.Log($"ROS Topics updated: {string.Join(", ", topics)}");
    }
}
```

### Environment Design Principles

#### Realistic Lighting
```csharp
using UnityEngine;

public class EnvironmentLighting : MonoBehaviour
{
    [Header("Lighting Configuration")]
    [Range(0, 2)] public float ambientIntensity = 0.5f;
    [Range(0, 5)] public float directionalLightIntensity = 1.0f;

    private Light directionalLight;
    private RenderSettings renderSettings;

    void Start()
    {
        SetupLighting();
    }

    void SetupLighting()
    {
        // Configure directional light (sun)
        directionalLight = FindObjectOfType<Light>();
        if (directionalLight != null)
        {
            directionalLight.intensity = directionalLightIntensity;
            directionalLight.color = Color.white;
        }

        // Configure ambient lighting
        RenderSettings.ambientIntensity = ambientIntensity;
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
    }
}
```

#### Physics Configuration
```csharp
using UnityEngine;

public class PhysicsConfiguration : MonoBehaviour
{
    [Header("Physics Settings")]
    public float gravity = -9.81f;
    public int solverIterations = 6;
    public int solverVelocityIterations = 1;

    void Start()
    {
        ConfigurePhysics();
    }

    void ConfigurePhysics()
    {
        Physics.gravity = new Vector3(0, gravity, 0);
        Physics.defaultSolverIterations = solverIterations;
        Physics.defaultSolverVelocityIterations = solverVelocityIterations;

        // Enable auto-simulation for real-time physics
        Physics.autoSimulation = true;
        Physics.autoSyncTransforms = true;
    }
}
```

## Human-Robot Interaction Interfaces

### Basic Robot Control Interface

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotControlUI : MonoBehaviour
{
    [Header("UI Elements")]
    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button moveButton;
    public Button stopButton;

    [Header("ROS Settings")]
    public string cmdVelTopic = "/cmd_vel";

    private RosConnection ros;
    private bool isMoving = false;

    void Start()
    {
        ros = RosConnection.GetOrCreateInstance();

        // Set up UI event listeners
        moveButton.onClick.AddListener(MoveRobot);
        stopButton.onClick.AddListener(StopRobot);

        // Initialize sliders
        linearVelocitySlider.minValue = 0f;
        linearVelocitySlider.maxValue = 2.0f;
        linearVelocitySlider.value = 0.5f;

        angularVelocitySlider.minValue = -2.0f;
        angularVelocitySlider.maxValue = 2.0f;
        angularVelocitySlider.value = 0f;
    }

    void MoveRobot()
    {
        if (ros == null) return;

        var twist = new TwistMsg();
        twist.linear = new Vector3Msg(linearVelocitySlider.value, 0, 0);
        twist.angular = new Vector3Msg(0, 0, angularVelocitySlider.value);

        ros.Publish(cmdVelTopic, twist);
        isMoving = true;
    }

    void StopRobot()
    {
        if (ros == null) return;

        var twist = new TwistMsg();
        twist.linear = new Vector3Msg(0, 0, 0);
        twist.angular = new Vector3Msg(0, 0, 0);

        ros.Publish(cmdVelTopic, twist);
        isMoving = false;

        // Reset sliders
        linearVelocitySlider.value = 0f;
        angularVelocitySlider.value = 0f;
    }
}
```

### Sensor Data Visualization

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class SensorVisualization : MonoBehaviour
{
    [Header("UI Elements")]
    public Text lidarDistanceText;
    public Text imuOrientationText;
    public Text cameraImageText;

    [Header("Visualization")]
    public GameObject lidarPointCloud;
    public GameObject robotOrientationIndicator;

    private RosConnection ros;

    void Start()
    {
        ros = RosConnection.GetOrCreateInstance();

        // Subscribe to sensor topics
        ros.Subscribe<LaserScanMsg>("/scan", OnLidarDataReceived);
        ros.Subscribe<ImuMsg>("/imu/data", OnImuDataReceived);
        ros.Subscribe<ImageMsg>("/camera/image_raw", OnCameraDataReceived);
    }

    void OnLidarDataReceived(LaserScanMsg scan)
    {
        if (lidarDistanceText != null)
        {
            // Show minimum distance
            float minDistance = float.MaxValue;
            foreach (float range in scan.ranges)
            {
                if (range > scan.range_min && range < scan.range_max && range < minDistance)
                {
                    minDistance = range;
                }
            }

            lidarDistanceText.text = $"Min Distance: {minDistance:F2}m";
        }
    }

    void OnImuDataReceived(ImuMsg imu)
    {
        if (imuOrientationText != null)
        {
            // Convert quaternion to Euler angles
            Quaternion quat = new Quaternion(
                (float)imu.orientation.x,
                (float)imu.orientation.y,
                (float)imu.orientation.z,
                (float)imu.orientation.w
            );

            Vector3 eulerAngles = quat.eulerAngles;
            imuOrientationText.text = $"Orientation: {eulerAngles.x:F1}°, {eulerAngles.y:F1}°, {eulerAngles.z:F1}°";
        }

        if (robotOrientationIndicator != null)
        {
            // Update visual indicator based on IMU data
            Quaternion quat = new Quaternion(
                (float)imu.orientation.x,
                (float)imu.orientation.y,
                (float)imu.orientation.z,
                (float)imu.orientation.w
            );
            robotOrientationIndicator.transform.rotation = quat;
        }
    }

    void OnCameraDataReceived(ImageMsg image)
    {
        if (cameraImageText != null)
        {
            cameraImageText.text = $"Camera: {image.width}x{image.height}, Format: {image.encoding}";
        }
    }
}
```

## ROS Integration with Unity

### Setting Up ROS Bridge

The Unity-Rosbridge integration enables real-time communication between Unity and ROS systems:

```csharp
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ROSBridgeManager : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    public bool autoConnect = true;

    [Header("Topics to Monitor")]
    public string[] topicsToMonitor;

    private RosConnection ros;

    void Start()
    {
        if (autoConnect)
        {
            ConnectToROS();
        }
    }

    public void ConnectToROS()
    {
        ros = RosConnection.GetOrCreateInstance();

        // Configure connection
        ros.Initialize(rosIPAddress, rosPort);

        Debug.Log($"Connecting to ROS at {rosIPAddress}:{rosPort}");
    }

    public void SubscribeToTopic<T>(string topicName) where T : Message
    {
        if (ros != null)
        {
            ros.Subscribe<T>(topicName, (msg) =>
            {
                Debug.Log($"Received message on {topicName}: {msg}");
            });
        }
    }

    public void PublishToTopic<T>(string topicName, T message) where T : Message
    {
        if (ros != null)
        {
            ros.Publish(topicName, message);
        }
    }
}
```

### Real-time Data Synchronization

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotStateSynchronizer : MonoBehaviour
{
    [Header("ROS Topics")]
    public string jointStatesTopic = "/joint_states";
    public string tfTopic = "/tf";

    [Header("Robot Configuration")]
    public Transform[] jointTransforms;
    public string[] jointNames;

    private RosConnection ros;
    private bool isConnected = false;

    void Start()
    {
        ros = RosConnection.GetOrCreateInstance();

        // Subscribe to joint states
        ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStatesReceived);

        Debug.Log("Robot State Synchronizer initialized");
    }

    void OnJointStatesReceived(JointStateMsg jointState)
    {
        if (jointState.name.Count != jointState.position.Count)
        {
            Debug.LogError("Joint names and positions count mismatch");
            return;
        }

        // Update each joint based on received positions
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            double jointPosition = jointState.position[i];

            // Find corresponding transform in Unity
            for (int j = 0; j < jointNames.Length; j++)
            {
                if (jointNames[j] == jointName && jointTransforms[j] != null)
                {
                    // Update joint rotation (assuming it's a revolute joint)
                    jointTransforms[j].localRotation = Quaternion.Euler(0, (float)jointPosition * Mathf.Rad2Deg, 0);
                    break;
                }
            }
        }
    }
}
```

## Advanced HRI Features

### Voice Command Interface

```csharp
using UnityEngine;
using UnityEngine.Windows.Speech;
using System.Collections.Generic;

public class VoiceCommandHandler : MonoBehaviour
{
    [Header("Voice Commands")]
    public string[] moveCommands = { "move forward", "move backward", "turn left", "turn right", "stop" };

    [Header("ROS Integration")]
    public string cmdVelTopic = "/cmd_vel";

    private KeywordRecognizer keywordRecognizer;
    private Dictionary<string, System.Action> keywordActions;
    private RosConnection ros;

    void Start()
    {
        SetupVoiceCommands();
        ros = RosConnection.GetOrCreateInstance();
    }

    void SetupVoiceCommands()
    {
        keywordActions = new Dictionary<string, System.Action>();

        keywordActions["move forward"] = () => SendVelocityCommand(0.5f, 0f);
        keywordActions["move backward"] = () => SendVelocityCommand(-0.5f, 0f);
        keywordActions["turn left"] = () => SendVelocityCommand(0f, 0.5f);
        keywordActions["turn right"] = () => SendVelocityCommand(0f, -0.5f);
        keywordActions["stop"] = () => SendVelocityCommand(0f, 0f);

        keywordRecognizer = new KeywordRecognizer(moveCommands);
        keywordRecognizer.OnPhraseRecognized += OnPhraseRecognized;
        keywordRecognizer.Start();
    }

    void OnPhraseRecognized(PhraseRecognizedEventArgs args)
    {
        string command = args.text.ToLower();
        Debug.Log($"Voice command recognized: {command}");

        if (keywordActions.ContainsKey(command))
        {
            keywordActions[command]();
        }
    }

    void SendVelocityCommand(float linearX, float angularZ)
    {
        if (ros == null) return;

        var twist = new RosMessageTypes.Geometry.TwistMsg();
        twist.linear = new RosMessageTypes.Geometry.Vector3Msg(linearX, 0, 0);
        twist.angular = new RosMessageTypes.Geometry.Vector3Msg(0, 0, angularZ);

        ros.Publish(cmdVelTopic, twist);
    }
}
```

### Gesture Recognition Interface

```csharp
using UnityEngine;
using UnityEngine.UI;

public class GestureControlHandler : MonoBehaviour
{
    [Header("UI Elements")]
    public Camera mainCamera;
    public RawImage gestureInputImage;

    [Header("Gesture Mapping")]
    public float minSwipeDistance = 50f;
    public float maxSwipeTime = 1f;

    private Vector2 startPos;
    private float startTime;
    private bool isTrackingSwipe = false;

    void Update()
    {
        HandleTouchInput();
    }

    void HandleTouchInput()
    {
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);

            switch (touch.phase)
            {
                case TouchPhase.Began:
                    startPos = touch.position;
                    startTime = Time.time;
                    isTrackingSwipe = true;
                    break;

                case TouchPhase.Ended:
                    if (isTrackingSwipe)
                    {
                        ProcessSwipe(touch.position);
                        isTrackingSwipe = false;
                    }
                    break;
            }
        }

        // Mouse simulation for testing
        if (Input.GetMouseButtonDown(0))
        {
            startPos = Input.mousePosition;
            startTime = Time.time;
            isTrackingSwipe = true;
        }
        else if (Input.GetMouseButtonUp(0))
        {
            if (isTrackingSwipe)
            {
                ProcessSwipe(Input.mousePosition);
                isTrackingSwipe = false;
            }
        }
    }

    void ProcessSwipe(Vector2 endPos)
    {
        float swipeTime = Time.time - startTime;
        Vector2 swipeVector = endPos - startPos;
        float swipeDistance = swipeVector.magnitude;

        if (swipeDistance >= minSwipeDistance && swipeTime <= maxSwipeTime)
        {
            Vector2 direction = swipeVector.normalized;

            // Determine swipe direction
            if (Mathf.Abs(direction.x) > Mathf.Abs(direction.y))
            {
                // Horizontal swipe
                if (direction.x > 0.5f)
                {
                    ExecuteGestureCommand("right");
                }
                else if (direction.x < -0.5f)
                {
                    ExecuteGestureCommand("left");
                }
            }
            else
            {
                // Vertical swipe
                if (direction.y > 0.5f)
                {
                    ExecuteGestureCommand("up");
                }
                else if (direction.y < -0.5f)
                {
                    ExecuteGestureCommand("down");
                }
            }
        }
    }

    void ExecuteGestureCommand(string gesture)
    {
        Debug.Log($"Gesture recognized: {gesture}");

        // Map gesture to robot action
        switch (gesture)
        {
            case "up":
                SendVelocityCommand(0.5f, 0f); // Move forward
                break;
            case "down":
                SendVelocityCommand(-0.5f, 0f); // Move backward
                break;
            case "left":
                SendVelocityCommand(0f, 0.5f); // Turn left
                break;
            case "right":
                SendVelocityCommand(0f, -0.5f); // Turn right
                break;
        }
    }

    void SendVelocityCommand(float linearX, float angularZ)
    {
        // Implementation for sending command to robot
        Debug.Log($"Sending velocity command: linear={linearX}, angular={angularZ}");
    }
}
```

## Lab Activity: Unity HRI Interface

Create a complete Unity-based HRI interface with:
1. 3D visualization of a robot in an environment
2. Multiple control methods (UI sliders, voice commands, gestures)
3. Real-time sensor data visualization
4. Integration with ROS for robot control

### Steps:
1. Set up Unity project with robotics packages
2. Create 3D environment with robot model
3. Implement UI controls for robot movement
4. Add voice command recognition
5. Include gesture recognition for mobile devices
6. Integrate with ROS bridge for real communication
7. Test all interaction methods

### Expected Outcome:
- Interactive Unity application with robot visualization
- Multiple control methods working correctly
- Real-time sensor data display
- Successful ROS integration

## Best Practices

1. **Performance Optimization**: Optimize scenes for real-time performance
2. **User Experience**: Design intuitive and responsive interfaces
3. **Safety**: Include safety checks and emergency stop functionality
4. **Testing**: Test interfaces with real users for usability
5. **Documentation**: Clearly document all interaction methods
6. **Accessibility**: Consider users with different abilities

## Checklist

- [ ] Unity environment setup completed
- [ ] 3D visualization implemented
- [ ] Multiple interaction methods created
- [ ] ROS integration established
- [ ] Sensor data visualization working
- [ ] Advanced HRI features implemented
- [ ] Lab activity completed successfully

## Next Steps

In the next module, we'll explore NVIDIA Isaac, which provides a comprehensive AI framework for robotics, building on the simulation and visualization foundations we've established.