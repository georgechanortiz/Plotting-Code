import os
from tkinter import Tk, filedialog
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from scipy.spatial.transform import Rotation as R
from collections import defaultdict
import numpy as np

def parseQuadBag(trialName=None, namespace=""):
    def load_mcap(trial_name=None):
        # Default to quad_log_current
        if trial_name is None:
            trial_name = "quad_log_current"

        # Construct expected path
        filepath = f"../bags/{trial_name}.mcap"

        # If file does not exist, open file chooser
        if not os.path.exists(filepath):
            print(f"{filepath} does not exist, choose a file manually...")

            root = Tk()
            root.withdraw()  
            filepath = filedialog.askopenfilename(
                title="Select log file",
                filetypes=[("MCAP files", "*.mcap")]
            )
            root.destroy()

            if not filepath:
                raise FileNotFoundError("No log file selected.")

            # Extract trial name from selected file
            trial_name = os.path.basename(filepath).replace(".mcap", "")

        print(f"Loading: {filepath}...")

        # Open the MCAP file for reading
        f = open(filepath, "rb")
        reader = make_reader(f, decoder_factories=[DecoderFactory()])

        return reader, filepath, trial_name

    reader, filepath, trial_name = load_mcap()
    print(filepath)

    timestamps = []
    topic_data = defaultdict(list)
    topic_timestamps = defaultdict(list)


    with open(filepath, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/robot_1/state/estimate" , "/robot_1/state/ground_truth" , "/robot_1/state/trajectory" , "/robot_1/control/grfs" , "/robot_1/state/grfs" , "/robot_1/local_plan"]):
            topic = channel.topic
            timestamp = message.log_time / 1e9 #convert nanosec to sec
            topic_data[topic].append(ros_msg)
            topic_timestamps[topic].append(timestamp)


    # Read state estimate data
    stateEstimateData = topic_data.get("/robot_1/state/estimate")
    stateEstimate = {}

    if stateEstimateData == None:
        print('No data on state estimate topic')
    else:
        # stateEstimate["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateEstimateData])
        stateEstimate["time"] = np.array(topic_timestamps.get("/robot_1/state/estimate"))
        stateEstimate["position"] = np.array([[m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z] for m in stateEstimateData])
        stateEstimate["velocity"] = np.array([[m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z] for m in stateEstimateData])
        stateEstimate["orientationQuat"] = np.array([[m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z] for m in stateEstimateData])
        stateEstimate["orientationRPY"] = np.array([R.from_quat([m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z, m.body.pose.orientation.w]).as_euler("xyz") for m in stateEstimateData])
        stateEstimate["angularVelocity"] = np.array([[m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z] for m in stateEstimateData])
        stateEstimate["jointPosition"] = np.array([np.array(m.joints.position) for m in stateEstimateData])
        stateEstimate["jointVelocity"] = np.array([np.array(m.joints.velocity) for m in stateEstimateData])
        stateEstimate["jointEffort"] = np.array([np.array(m.joints.effort) for m in stateEstimateData])

    # Read ground truth data
    stateGroundTruthData = topic_data.get("/robot_1/state/ground_truth")
    stateGroundTruth = {}

    if stateGroundTruthData == None:
        print('No data on ground truth state topic')
    else:
        stateGroundTruth["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateGroundTruthData])
        stateGroundTruth["position"] = np.array([[m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z] for m in stateGroundTruthData])
        stateGroundTruth["velocity"] = np.array([[m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z] for m in stateGroundTruthData])
        stateGroundTruth["orientationQuat"] = np.array([[m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z] for m in stateGroundTruthData])
        stateGroundTruth["orientationRPY"] = np.array([R.from_quat([m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z, m.body.pose.orientation.w]).as_euler("xyz") for m in stateGroundTruthData])
        stateGroundTruth["angularVelocity"] = np.array([[m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z] for m in stateGroundTruthData])
        stateGroundTruth["jointPosition"] = np.vstack([np.array(m.joints.position) for m in stateGroundTruthData])
        stateGroundTruth["jointVelocity"] = np.vstack([np.array(m.joints.velocity) for m in stateGroundTruthData])
        stateGroundTruth["jointEffort"] = np.vstack([np.array(m.joints.effort) for m in stateGroundTruthData])
        num_feet = len(stateGroundTruthData[0].feet.feet)
        stateGroundTruth["footPosition"] = []
        stateGroundTruth["footVelocity"] = []
        for i in range(num_feet):
            stateGroundTruth["footPosition"].append(np.array([[m.feet.feet[i].position.x, m.feet.feet[i].position.y, m.feet.feet[i].position.z] for m in stateGroundTruthData]))
            stateGroundTruth["footVelocity"].append(np.array([[m.feet.feet[i].velocity.x, m.feet.feet[i].velocity.y, m.feet.feet[i].velocity.z] for m in stateGroundTruthData]))

    # Read trajectory data
    stateTrajectoryData = topic_data.get("/robot_1/state/trajectory")
    stateTrajectory = {}
        
    if stateTrajectoryData == None:
        print('No data on trajectory topic')
    else:
        stateTrajectory["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateTrajectoryData])
        stateTrajectory["position"] = np.array([[m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z] for m in stateTrajectoryData])
        stateTrajectory["velocity"] = np.array([[m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z] for m in stateTrajectoryData])
        stateTrajectory["orientationQuat"] = np.array([[m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z] for m in stateTrajectoryData])
        stateTrajectory["orientationRPY"] = np.array([R.from_quat([m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z, m.body.pose.orientation.w]).as_euler("xyz") for m in stateTrajectoryData])
        stateTrajectory["angularVelocity"] = np.array([[m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z] for m in stateTrajectoryData])
        # Omit joint and foot data, as it is not included in the reference trajectory
        # stateTrajectory['jointPosition'] = np.array([m.joints.position for m in stateTrajectoryData])
        # stateTrajectory['jointPosition'].fill(np.nan)
        # stateTrajectory['jointVelocity'] = np.array([m.joints.velocity for m in stateTrajectoryData])
        # stateTrajectory['jointVelocity'].fill(np.nan)
        # stateTrajectory['jointEffort'] = np.array([m.joints.effort for m in stateTrajectoryData])
        # stateTrajectory['jointEffort'].fill(np.nan)

        # num_feet = len(stateTrajectoryData[0].feet.feet)
        # stateTrajectory['footPosition'] = [None] * num_feet
        # stateTrajectory['footVelocity'] = [None] * num_feet
        # for i in range(num_feet):
        #     stateTrajectory['footPosition'][i] = np.array([[m.feet.feet[i].position.x, m.feet.feet[i].position.y, m.feet.feet[i].position.z] for m in stateTrajectoryData])
        #     stateTrajectory['footVelocity'][i] = np.array([[m.feet.feet[i].velocity.x, m.feet.feet[i].velocity.y, m.feet.feet[i].velocity.z] for m in stateTrajectoryData])
        #     stateTrajectory['footPosition'][i].fill(np.nan)
        #     stateTrajectory['footVelocity'][i].fill(np.nan)



    # Read control GRF data
    controlGRFsData = topic_data.get("/robot_1/control/grfs")
    controlGRFs = {}
    controlGRFs["vectors"] = []
    controlGRFs["points"] = []
    controlGRFs["contactStates"] = []

    if controlGRFsData == None:
        print('No data on grf control topic')
    else:
        controlGRFs["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in controlGRFsData])
        num_feet = 4
        for i in range(num_feet):
            try:
                controlGRFs["vectors"].append(np.array([[m.vectors[i].x, m.vectors[i].y, m.vectors[i].z] for m in controlGRFsData]))
                controlGRFs["points"].append(np.array([[m.points[i].x, m.points[i].y, m.points[i].z] for m in controlGRFsData]))
                controlGRFs["contactStates"].append(np.array([[m.contact_states[i], m.contact_states[i], m.contact_states[i]] for m in controlGRFsData]))
            except:
                controlGRFs["vectors"].append(np.array([[0, 0, 0] for m in controlGRFsData]))
                controlGRFs["points"].append(np.array([[0, 0, 0] for m in controlGRFsData]))
                controlGRFs["contactStates"].append(np.array([[0, 0, 0] for m in controlGRFsData]))
        
    # Read state GRF data
    stateGRFsData = topic_data.get("/robot_1/state/grfs")
    stateGRFs = {}
    stateGRFs["vectors"] = []
    stateGRFs["points"] = []
    stateGRFs["contactStates"] = []

    if len(stateGRFsData) == 0:
        print('No data on grf state topic')
    else:
        stateGRFs["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateGRFsData])
        num_feet = 4
        for i in range(num_feet):
            try:
                stateGRFs["vectors"].append(np.array([[m.vectors[i].x, m.vectors[i].y, m.vectors[i].z] for m in stateGRFsData]))
                stateGRFs["points"].append(np.array([[m.points[i].x, m.points[i].y, m.points[i].z] for m in stateGRFsData]))
                stateGRFs["contactStates"].append(np.array([[m.contact_states[i], m.contact_states[i], m.contact_states[i]] for m in stateGRFsData]))
            except:
                stateGRFs["vectors"].append(np.array([[0, 0, 0] for m in stateGRFsData]))
                stateGRFs["points"].append(np.array([[0, 0, 0] for m in stateGRFsData]))
                stateGRFs["contactStates"].append(np.array([[0, 0, 0] for m in stateGRFsData]))
        


    # Read local plan data
    localPlanData = topic_data.get("/robot_1/local_plan")
    localPlan = {}

    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/local_plan"]):
        localPlanData.append(ros_msg)

    if len(localPlanData) == 0:
        print('No data on local plan topic')
    else:
        localPlan["time"] = np.array([float(m.state_timestamp.sec) + float(m.state_timestamp.nanosec) * 1e-9 for m in localPlanData])
        localPlan["elementTimes"] = [np.array(m.diagnostics.element_times, dtype=float) for m in localPlanData]
        localPlan["solveTime"] = np.array([m.diagnostics.compute_time for m in localPlanData])
        localPlan["cost"] = np.array([m.diagnostics.cost for m in localPlanData])
        localPlan["iterations"] = np.array([m.diagnostics.iterations for m in localPlanData])
        localPlan["horizonLength"] = np.array([m.diagnostics.horizon_length for m in localPlanData])
        localPlan["complexitySchedule"] = [np.array(m.diagnostics.complexity_schedule, dtype=float) for m in localPlanData]





    # Localize time to the first message
    startTime = stateGroundTruth["time"][0]
    data = {}

    # Update time of existing messages and pack them into the dictionary
    if len(stateEstimate.keys()) != 0:
        stateEstimate["time"] = stateEstimate["time"] - startTime
        data["stateEstimate"] = stateEstimate
    else:
        data["stateEstimate"] = []
        
    if len(stateGroundTruth.keys()) != 0:
        stateGroundTruth["time"] = stateGroundTruth["time"] - startTime
        data["stateGroundTruth"] = stateGroundTruth
    else:
        data["stateGroundTruth"] = []

    if len(stateTrajectory.keys()) != 0:
        stateTrajectory["time"] = stateTrajectory["time"] - startTime
        data["stateTrajectory"] = stateTrajectory
    else:
        data["stateTrajectory"] = []
        
    if len(controlGRFs.keys()) != 0:
        controlGRFs["time"] = controlGRFs["time"] - startTime
        data["controlGRFs"] = controlGRFs
    else:
        data["controlGRFs"] = []

    if len(stateGRFs.keys()) != 0:
        stateGRFs["time"] = stateGRFs["time"] - startTime
        data["stateGRFs"] = stateGRFs
    else:
        data["stateGRFs"] = []

    if len(localPlan.keys()) != 0:
        localPlan["time"] = localPlan["time"] - startTime
        for i in range(len(localPlan["elementTimes"])):
            shiftedElementTimes = localPlan["elementTimes"][i]+localPlan["time"][i]
            localPlan["elementTimes"][i] = shiftedElementTimes
        data["localPlan"] = localPlan
    else:
        data["localPlan"] = []

    return [data, trial_name]


