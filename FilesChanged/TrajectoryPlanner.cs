using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Ur3Moveit;
using RosMessageTypes.MSI;

using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;


using System.Net;
using System.Net.Sockets;
using System;


using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

public class TrajectoryPlanner : MonoBehaviour 
{
    // ROS Connector
    private ROSConnection ros;

    // Hardcoded variables 
    private int numRobotJoints = 6;
    private readonly float jointAssignmentWait = 2.06f; // 0.06f;
    private readonly float poseAssignmentWait = 2.5f; //0.5f;
    private readonly float gripperAngle = 14f;
    // Offsets to ensure gripper is above grasp points
    private readonly Vector3 pickPoseOffset = new Vector3(0, 0.255f, 0);
    private readonly Vector3 placePoseOffset = new Vector3(0, 0.275f, 0);
    // Multipliers correspond to the URDF mimic tag for each joint
    private float[] multipliers = new float[] { -1f, -1f, -1f, 1f, 1f, 1f };
    // Orientation is hardcoded for this example so the gripper is always directly above the placement object
    private readonly Quaternion pickOrientation = new Quaternion(-0.5f, -0.5f, 0.5f, -0.5f);


    public string rosServiceName = "ur3_moveit";
    public string m_TopicName = "ur3_joints";
    public string GripperTopic = "gripper";
    private const int isBigEndian = 0;
    private const int step = 4;

    public GameObject robot;
    public GameObject target;
    public Transform goal;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    ArticulationBody[] articulationChain;
    private List<ArticulationBody> gripperJoints;

    // UI elements
    private Button InitializeButton;
    private Button RandomizeButton;
    private Button ServiceButton;
    private Text ActualPos;
    private Text ActualRot;
    private Text EstimatedPos;
    private Text EstimatedRot;

    private RenderTexture renderTexture;

    public PoseEstimationScenario scenario;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place,
        PostPlace
    };


    static Socket listener;
    private CancellationTokenSource source;
    public ManualResetEvent allDone;
    public Renderer objectRenderer;
    private Color matColor;

    public static readonly int PORT = 1755;
    public static readonly int WAITTIME = 1;


    private Socket client;
    [SerializeField]
    private float[] dataOut, dataIn; //debugging

    /// <summary>
    /// Helper function for sending and receiving.
    /// </summary>
    /// <param name="dataOut">Data to send</param>
    /// <returns></returns>
    /*protected float[] ServerRequest(float[] dataOut)
    {
        //print("request");
        this.dataOut = dataOut; //debugging
        this.dataIn = SendAndReceive(dataOut); //debugging
        return this.dataIn;
    }
    private float[] SendAndReceive(float[] dataOut)
    {
        //initialize socket
        float[] floatsReceived;
        client = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        client.Connect(ip, port);
        
        if (!client.Connected)
        {
            Debug.LogError("Connection Failed");
            return null;
        }
        Debug.Log("connected");
        //convert floats to bytes, send to port
        var byteArray = new byte[dataOut.Length * 4];
        Buffer.BlockCopy(dataOut, 0, byteArray, 0, byteArray.Length);
        client.Send(byteArray);

        //allocate and receive bytes
        byte[] bytes = new byte[4000];
        int idxUsedBytes = client.Receive(bytes);
        //print(idxUsedBytes + " new bytes received.");

        //convert bytes to floats
        floatsReceived = new float[idxUsedBytes / 4];
        Buffer.BlockCopy(bytes, 0, floatsReceived, 0, idxUsedBytes);

        client.Close();
        return floatsReceived;
    }
    */
    /// <summary>
    ///     Opens and closes the attached gripper tool based on a gripping angle.
    /// </summary>
    /// <param name="toClose"></param>
    /// <returns></returns>
    public IEnumerator IterateToGrip(bool toClose)
    {
        var grippingAngle = toClose ? gripperAngle : 0f;
        for (int i = 0; i < gripperJoints.Count; i++)
        {
            var curXDrive = gripperJoints[i].xDrive;
            curXDrive.target = multipliers[i] * grippingAngle;
            gripperJoints[i].xDrive = curXDrive;
        }
        yield return new WaitForSeconds(jointAssignmentWait);
    }

    /// <summary>
    ///     Button callback for setting the robot to default position
    /// </summary>
    public void Initialize()
    {
        StartCoroutine(MoveToInitialPosition());
    }

    /// <summary>
    ///     Button callback for the Cube Randomization
    /// </summary>
    public void RandomizeCube()
    {
        scenario.Move();
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
    }

    /// <summary>
    ///     Button callback for the Pose Estimation
    /// </summary>
    public void PoseEstimation()
    {
        Debug.Log("Capturing image ...");

        InitializeButton.interactable = false;
        RandomizeButton.interactable = false;
        ServiceButton.interactable = false;

        EstimatedPos.text = "-";

        EstimatedRot.text = "-";
        float[] obj_id = new float[2];
        //InvokePoseEstimationServiceOve6D(obj_id);
        source = new CancellationTokenSource();
        allDone = new ManualResetEvent(false);
        Task.Run(() => ListenEvents(source.Token));
        //var estimatedRotation = Camera.main.transform.rotation * response.estimated_pose.orientation.From<RUF>();
        //Vector3 estimatedPosition = new Vector3(float.Parse("-0.37", CultureInfo.InvariantCulture.NumberFormat), float.Parse("0.8", CultureInfo.InvariantCulture.NumberFormat), float.Parse("0.18", CultureInfo.InvariantCulture.NumberFormat));
        //PublishJoints(estimatedPosition, estimatedRotation);

    }

    private void ListenEvents(CancellationToken token)
    {


        IPHostEntry ipHostInfo = Dns.GetHostEntry(Dns.GetHostName());
        IPAddress ipAddress = ipHostInfo.AddressList.FirstOrDefault(ip => ip.AddressFamily == AddressFamily.InterNetwork);
        print(ipAddress.MapToIPv4().ToString());
        IPEndPoint localEndPoint = new IPEndPoint(ipAddress, PORT);


        listener = new Socket(ipAddress.AddressFamily, SocketType.Stream, ProtocolType.Tcp);


        try
        {
            listener.Bind(localEndPoint);
            listener.Listen(10);


            while (!token.IsCancellationRequested)
            {
                allDone.Reset();

                print("Waiting for a connection... host :" + ipAddress.MapToIPv4().ToString() + " port : " + PORT);
                listener.BeginAccept(new AsyncCallback(AcceptCallback), listener);

                while (!token.IsCancellationRequested)
                {
                    if (allDone.WaitOne(WAITTIME))
                    {
                        break;
                    }
                }

            }

        }
        catch (Exception e)
        {
            print(e.ToString());
        }
    }

    void AcceptCallback(IAsyncResult ar)
    {
        Socket listener = (Socket)ar.AsyncState;
        Socket handler = listener.EndAccept(ar);

        allDone.Set();

        StateObject state = new StateObject();
        state.workSocket = handler;
        handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0, new AsyncCallback(ReadCallback), state);
    }

    void ReadCallback(IAsyncResult ar)
    {
        StateObject state = (StateObject)ar.AsyncState;
        Socket handler = state.workSocket;

        int read = handler.EndReceive(ar);

        if (read > 0)
        {
            state.colorCode.Append(Encoding.ASCII.GetString(state.buffer, 0, read));
            handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0, new AsyncCallback(ReadCallback), state);
        }
        else
        {
            if (state.colorCode.Length > 1)
            {
                string content = state.colorCode.ToString();
                print($"Read {content.Length} bytes from socket.\n Data : {content}");
                SetColors(content);
            }
            handler.Close();
        }
    }

    //Set color to the Material
    private void SetColors(string data)
    {
        string[] colors = data.Split(',');
        print(data);

    }

    private void OnDestroy()
    {
        source.Cancel();
    }

    public class StateObject
    {
        public Socket workSocket = null;
        public const int BufferSize = 1024;
        public byte[] buffer = new byte[BufferSize];
        public StringBuilder colorCode = new StringBuilder();
    }



private IEnumerator MoveToInitialPosition()
    {
        bool isRotationFinished = false;
        while (!isRotationFinished)
        {
            isRotationFinished = ResetRobotToDefaultPosition();
            yield return new WaitForSeconds(jointAssignmentWait);
        }
        ServiceButton.interactable = true;
    }

    private bool ResetRobotToDefaultPosition()
    {
        bool isRotationFinished = true;
        var rotationSpeed = 180f;

        for (int i = 0; i < numRobotJoints; i++)
        {
            var tempXDrive = jointArticulationBodies[i].xDrive;
            float currentRotation = tempXDrive.target;

            float rotationChange = rotationSpeed * Time.fixedDeltaTime;

            if (currentRotation > 0f) rotationChange *= -1;

            if (Mathf.Abs(currentRotation) < rotationChange)
                rotationChange = 0;
            else
                isRotationFinished = false;

            // the new xDrive target is the currentRotation summed with the desired change
            float rotationGoal = currentRotation + rotationChange;
            tempXDrive.target = rotationGoal;
            jointArticulationBodies[i].xDrive = tempXDrive;
        }

        MoverServiceRequest request2 = new MoverServiceRequest();
        request2.joints_input = CurrentJointConfig();

        MoverServiceRequest letter = new MoverServiceRequest();
        UR3MoveitJoints myjoints = new UR3MoveitJoints();
        myjoints = request2.joints_input;


        myjoints.joint_02 = myjoints.joint_02 - 1.5708f;

        letter.joints_input = myjoints;


        ros.Send("ur3_joints", letter);

        return isRotationFinished;
    }

    /// <summary>
    ///     Create a new PoseEstimationServiceRequest with the captured screenshot as bytes and instantiates 
    ///     a sensor_msgs/image.
    ///
    ///     Call the PoseEstimationService using the ROSConnection and calls PoseEstimationCallback on the 
    ///     PoseEstimationServiceResponse.
    /// </summary>
    /// <param name="imageData"></param>

    /*private void InvokePoseEstimationServiceOve6D(float[] obj_id)

    {

        float[] prediction = ServerRequest(obj_id);
        Vector3 estimatedPosition = new Vector3(prediction[9], prediction[10], prediction[11]);
        var rotationMatrix = new Matrix4x4();
        int idx = 0;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                idx += 1;
                rotationMatrix[i, j] = prediction[idx];
            }
        }
        rotationMatrix[3, 3] = 1f;

        Quaternion estimatedRotation = Quaternion.LookRotation(rotationMatrix.GetColumn(2), rotationMatrix.GetColumn(1));



        // Quaternion estimatedRotation = new Quaternion(prediction[0], prediction[1], prediction[2], prediction[3]);
        PublishJoints(estimatedPosition, estimatedRotation);
        EstimatedPos.text = estimatedPosition.ToString();
        EstimatedRot.text = estimatedRotation.eulerAngles.ToString();
        //RosMessageTypes.MSI.Int32 poseServiceRequest = new RosMessageTypes.MSI.Int32(obj_id);

        //ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_ove6d_estimation_service", poseServiceRequest, PoseEstimationCallbackOve6D);
    }*/


    private void InvokeGripper(float pos, float speed, float force)

    {
        //RosMessageTypes.MSI.Int32 ros_obj_id = new RosMessageTypes.MSI.Int32(obj_id);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////RosMessageTypes.MSI.RobotiqGripperCommand GripperRequest = new RosMessageTypes.MSI.RobotiqGripperCommand(false, 0, false, pos,speed,force);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////ros.Send("gripper", GripperRequest);
    }

    /*void PoseEstimationCallbackOve6D(PoseEstimationServiceResponse response)
    {
        if (response != null)
        {
            // The position output by the model is the position of the cube relative to the camera so we need to extract its global position 
            var estimatedPosition = response.estimated_pose.position.From<RUF>();
            var estimatedRotation = response.estimated_pose.orientation.From<RUF>();

            PublishJoints(estimatedPosition, estimatedRotation);

            EstimatedPos.text = estimatedPosition.ToString();
            EstimatedRot.text = estimatedRotation.eulerAngles.ToString();
        }
        else
        {
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
        }
    }
    */

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>UR3MoveitJoints</returns>
    UR3MoveitJoints CurrentJointConfig()
    {
        UR3MoveitJoints joints = new UR3MoveitJoints();

        joints.joint_00 = jointArticulationBodies[0].xDrive.target;
        joints.joint_01 = jointArticulationBodies[1].xDrive.target;
        joints.joint_02 = jointArticulationBodies[2].xDrive.target;
        joints.joint_03 = jointArticulationBodies[3].xDrive.target;
        joints.joint_04 = jointArticulationBodies[4].xDrive.target;
        joints.joint_05 = jointArticulationBodies[5].xDrive.target;

        return joints;
    }

    public void PublishJoints(Vector3 targetPos, Quaternion targetRot)
    {
        MoverServiceRequest request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (targetPos + pickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(90, targetRot.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (goal.position + placePoseOffset).To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };
        ////////////////////////////////////////////////////////////////////////////////////////////ros.Send("ur3_joints", request);
        ros.SendServiceMessage<MoverServiceResponse>(rosServiceName, request, TrajectoryResponse);
        request.joints_input.joint_01 -= 1.5708f;
        ros.Send(m_TopicName, request.joints_input);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories != null && response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
            ServiceButton.interactable = true;
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///         PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// 
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from ur3_moveit mover service running in ROS</param>
    /// <returns></returns>
    private IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (int poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();


                    MoverServiceRequest letter1 = new MoverServiceRequest();
                    UR3MoveitJoints myjoints = new UR3MoveitJoints();

                    myjoints.joint_00 = result[0];
                    myjoints.joint_01 = result[1];
                    myjoints.joint_02 = result[2] - 1.5708f;
                    myjoints.joint_03 = result[3];
                    myjoints.joint_04 = result[4];
                    myjoints.joint_05 = result[5];
                    letter1.joints_input = myjoints;
                    ros.Send("ur3_joints", letter1);
                    yield return new WaitForSeconds(jointAssignmentWait);


                    // Set the joint values for every joint
                    for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = jointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        jointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(jointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    StartCoroutine(IterateToGrip(true));

                    ///////////////////////////////////////////////InvokeGripper(0.0f, 0.0f, 0.0f);


                    yield return new WaitForSeconds(jointAssignmentWait);
                }
                else if (poseIndex == (int)Poses.Place)
                {
                    yield return new WaitForSeconds(poseAssignmentWait);
                    // Open the gripper to place the target cube

                    /////////////////////////////////////////////////////InvokeGripper(1.0f, 0.0f, 0.0f);

                    StartCoroutine(IterateToGrip(false));
                }
                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(poseAssignmentWait);
            }

            // Re-enable buttons
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
            yield return new WaitForSeconds(jointAssignmentWait);
        }
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find all gripper joints and assign them to their respective articulation body objects.
    /// </summary>
    void Awake()
    {
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = robot.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string arm_link = shoulder_link + "/upper_arm_link";
        jointArticulationBodies[1] = robot.transform.Find(arm_link).GetComponent<ArticulationBody>();

        string elbow_link = arm_link + "/forearm_link";
        jointArticulationBodies[2] = robot.transform.Find(elbow_link).GetComponent<ArticulationBody>();

        string forearm_link = elbow_link + "/wrist_1_link";
        jointArticulationBodies[3] = robot.transform.Find(forearm_link).GetComponent<ArticulationBody>();

        string wrist_link = forearm_link + "/wrist_2_link";
        jointArticulationBodies[4] = robot.transform.Find(wrist_link).GetComponent<ArticulationBody>();

        string hand_link = wrist_link + "/wrist_3_link";
        jointArticulationBodies[5] = robot.transform.Find(hand_link).GetComponent<ArticulationBody>();

        articulationChain = robot.GetComponent<RosSharp.Control.Controller>().GetComponentsInChildren<ArticulationBody>();

        var gripperJointNames = new string[] { "right_outer_knuckle", "right_inner_finger", "right_inner_knuckle", "left_outer_knuckle", "left_inner_finger", "left_inner_knuckle" };
        gripperJoints = new List<ArticulationBody>();

        foreach (ArticulationBody articulationBody in robot.GetComponentsInChildren<ArticulationBody>())
        {
            if (gripperJointNames.Contains(articulationBody.name))
            {
                gripperJoints.Add(articulationBody);
            }
        }
    }

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;
        ros.RegisterPublisher("ur3_joints", "ur3_moveit/UR3MoveitJoints");
        // Assign UI elements
        InitializeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/DefaultButton").GetComponent<Button>();
        RandomizeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/RandomButton").GetComponent<Button>();
        ServiceButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/ServiceButton").GetComponent<Button>();

        ActualPos = GameObject.Find("ROSObjects/Canvas/PositionPanel/ActualPosField").GetComponent<Text>();
        ActualRot = GameObject.Find("ROSObjects/Canvas/PositionPanel/ActualRotField").GetComponent<Text>();
        EstimatedPos = GameObject.Find("ROSObjects/Canvas/PositionPanel/EstPosField").GetComponent<Text>();
        EstimatedRot = GameObject.Find("ROSObjects/Canvas/PositionPanel/EstRotField").GetComponent<Text>();

        // Initialize UI element values
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
        EstimatedPos.text = "-";
        EstimatedRot.text = "-";

        // Render texture 
        renderTexture = new RenderTexture(Camera.main.pixelWidth, Camera.main.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
    }
}