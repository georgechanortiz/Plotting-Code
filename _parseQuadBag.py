# Specify file path
import os
from tkinter import Tk, filedialog
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from scipy.spatial.transform import Rotation as R

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

    # Read state estimate data
    stateEstimateData = []
    stateEstimate = {}

    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/state/estimate"]):
        stateEstimateData.append(ros_msg)

    if len(stateEstimateData) == 0:
        print('No data on state estimate topic')
    else:
        stateEstimate["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateEstimateData])

        stateEstimate["position"] = np.array([
            [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z] for m in stateEstimateData])
        stateEstimate["velocity"] = np.array([
            [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z] for m in stateEstimateData])
        stateEstimate["orientationQuat"] = np.array([
            [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z] for m in stateEstimateData])
        stateEstimate["orientationRPY"] = np.array([R.from_quat(
            [m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z, m.body.pose.orientation.w]).as_euler("xyz") for m in stateEstimateData])
        stateEstimate["angularVelocity"] = np.array([
            [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z] for m in stateEstimateData])

        stateEstimate["jointPosition"] = np.array([np.array(m.joints.position) for m in stateEstimateData])
        stateEstimate["jointVelocity"] = np.array([np.array(m.joints.velocity) for m in stateEstimateData])
        stateEstimate["jointEffort"] = np.array([np.array(m.joints.effort) for m in stateEstimateData])


    # Read ground truth data
    stateGroundTruthData = []
    stateGroundTruth = {}

    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/state/ground_truth"]):
        stateGroundTruthData.append(ros_msg)

    if len(stateGroundTruthData) == 0:
        print('No data on ground truth state topic')
    else:
        stateGroundTruth["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateGroundTruthData])
        stateGroundTruth["position"] = np.array([
            [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z] for m in stateGroundTruthData])
        stateGroundTruth["velocity"] = np.array([
            [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z] for m in stateGroundTruthData])
        stateGroundTruth["orientationQuat"] = np.array([
            [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z] for m in stateGroundTruthData])
        stateGroundTruth["orientationRPY"] = np.array([R.from_quat(
            [m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z, m.body.pose.orientation.w]).as_euler("xyz") for m in stateGroundTruthData])
        stateGroundTruth["angularVelocity"] = np.array([
            [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z] for m in stateGroundTruthData])

        stateGroundTruth["jointPosition"] = np.vstack([np.array(m.joints.position) for m in stateGroundTruthData])
        stateGroundTruth["jointVelocity"] = np.vstack([np.array(m.joints.velocity) for m in stateGroundTruthData])
        stateGroundTruth["jointEffort"] = np.vstack([np.array(m.joints.effort) for m in stateGroundTruthData])
        
        num_feet = len(stateGroundTruthData[0].feet.feet)
        stateGroundTruth["footPosition"] = []
        stateGroundTruth["footVelocity"] = []

        for i in range(num_feet):
            stateGroundTruth["footPosition"].append(
                np.array([[m.feet.feet[i].position.x, m.feet.feet[i].position.y, m.feet.feet[i].position.z] for m in stateGroundTruthData]))
            stateGroundTruth["footVelocity"].append(
                np.array([[m.feet.feet[i].velocity.x, m.feet.feet[i].velocity.y, m.feet.feet[i].velocity.z] for m in stateGroundTruthData]))



    # Read the Ground truth Body Frame Data
    # stateGroundTruthBodyFrameData = []
    # stateGroundTruthBodyFrame = {}

    # for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/state/ground_truth_body_frame"]):
    #     stateGroundTruthBodyFrameData.append(ros_msg)

    # if len(stateGroundTruthData) == 0:
    #     print('No data on ground truth body frame state topic')
    # else:
    #     stateGroundTruthBodyFrame["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateGroundTruthBodyFrameData])
    #     stateGroundTruthBodyFrame["position"] = np.array([
    #         [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z] for m in stateGroundTruthBodyFrameData])
    #     stateGroundTruthBodyFrame["velocity"] = np.array([
    #         [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z] for m in stateGroundTruthBodyFrameData])
    #     stateGroundTruthBodyFrame["orientationQuat"] = np.array([
    #         [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z] for m in stateGroundTruthBodyFrameData])
    #     stateGroundTruthBodyFrame["orientationRPY"] = np.array([R.from_quat(
    #         [m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z, m.body.pose.orientation.w]).as_euler("xyz") for m in stateGroundTruthBodyFrameData])
    #     stateGroundTruthBodyFrame["angularVelocity"] = np.array([
    #         [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z] for m in stateGroundTruthBodyFrameData])

    #     stateGroundTruthBodyFrame["jointPosition"] = np.vstack([np.array(m.joints.position) for m in stateGroundTruthBodyFrameData])
    #     stateGroundTruthBodyFrame["jointVelocity"] = np.vstack([np.array(m.joints.velocity) for m in stateGroundTruthBodyFrameData])
    #     stateGroundTruthBodyFrame["jointEffort"] = np.vstack([np.array(m.joints.effort) for m in stateGroundTruthBodyFrameData])
        
    #     num_feet = len(stateGroundTruthBodyFrameData[0].feet.feet)
    #     stateGroundTruthBodyFrame["footPosition"] = []
    #     stateGroundTruthBodyFrame["footVelocity"] = []

    #     for i in range(num_feet):
    #         stateGroundTruthBodyFrame["footPosition"].append(
    #             np.array([[m.feet.feet[i].position.x, m.feet.feet[i].position.y, m.feet.feet[i].position.z] for m in stateGroundTruthBodyFrameData]))
    #         stateGroundTruthBodyFrame["footVelocity"].append(
    #             np.array([[m.feet.feet[i].velocity.x, m.feet.feet[i].velocity.y, m.feet.feet[i].velocity.z] for m in stateGroundTruthBodyFrameData]))


    # Read trajectory data
    stateTrajectoryData = []
    stateTrajectory = {}


    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/state/trajectory"]):
        stateTrajectoryData.append(ros_msg)
        
    if len(stateTrajectoryData) == 0:
        print('No data on trajectory topic')
    else:
        stateTrajectory["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in stateTrajectoryData])
        stateTrajectory["position"] = np.array([
            [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z] for m in stateTrajectoryData])
        stateTrajectory["velocity"] = np.array([
            [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z] for m in stateTrajectoryData])
        stateTrajectory["orientationQuat"] = np.array([
            [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z] for m in stateTrajectoryData])
        stateTrajectory["orientationRPY"] = np.array([R.from_quat(
            [m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z, m.body.pose.orientation.w]).as_euler("xyz") for m in stateTrajectoryData])
        stateTrajectory["angularVelocity"] = np.array([
            [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z] for m in stateTrajectoryData])
        
        # Omit joint and foot data, as it is not included in the reference trajectory
        # stateTrajectory["jointPosition"] = np.vstack([np.array(m.joints.position) for m in stateTrajectoryData])
        # stateTrajectory["jointVelocity"] = np.vstack([np.array(m.joints.velocity) for m in stateTrajectoryData])
        # stateTrajectory["jointEffort"] = np.vstack([np.array(m.joints.effort) for m in stateTrajectoryData])
        
        # for i in range(num_feet):
        #     stateTrajectory["footPosition"].append(
        #         np.array([[m.feet.feet[i].position.x, m.feet.feet[i].position.y, m.feet.feet[i].position.z] for m in stateTrajectoryData]))
        #     stateTrajectory["footVelocity"].append(
        #         np.array([[m.feet.feet[i].velocity.x, m.feet.feet[i].velocity.y, m.feet.feet[i].velocity.z] for m in stateTrajectoryData]))

    # Read control GRF data
    controlGRFsData = []
    controlGRFs = {}
    controlGRFs["vectors"] = []
    controlGRFs["points"] = []
    controlGRFs["contactStates"] = []

    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/control/grfs"]):
        controlGRFsData.append(ros_msg)

    if len(controlGRFsData) == 0:
        print('No data on grf control topic')
    else:
        controlGRFs["time"] = np.array([
            float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
            for m in controlGRFsData
        ])
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
    stateGRFsData = []
    stateGRFs = {}
    stateGRFs["vectors"] = []
    stateGRFs["points"] = []
    stateGRFs["contactStates"] = []

    for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/state/grfs"]):
        stateGRFsData.append(ros_msg)

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
        

    # # Read cmd vel data
    # cmdVelData = []
    # cmdVels = {}

    # for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/cmd_vel_stamped"]):
    #     cmdVelData.append(ros_msg)

    # if len(cmdVelData) == 0:
    #     print('No data on cmd vel topic')
    # else:
    #     cmdVels["time"] = np.array([float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9 for m in cmdVelData])
    #     cmdVels["velocity"] = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for m in cmdVelData])
    #     cmdVels["angularVelocity"] = np.array([[m.twist.angular.x, m.twist.angular.y, m.twist.angular.z] for m in cmdVelData])

    # Read local plan data
    localPlanData = []
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

    # if len(stateGroundTruthBodyFrame.keys()) != 0:
    #     stateGroundTruthBodyFrame["time"] = stateGroundTruthBodyFrame["time"] - startTime
    #     data["stateGroundTruthBodyFrame"] = stateGroundTruthBodyFrame
    # else:
    #     data["stateGroundTruthBodyFrame"] = []

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
        
    # if len(cmdVels.keys()) != 0:
    #     cmdVels["time"] = cmdVels["time"] - startTime
    #     data["cmdVels"] = cmdVels
    # else:
    #     data["cmdVels"] = []
        
    if len(localPlan.keys()) != 0:
        localPlan["time"] = localPlan["time"] - startTime
        for i in range(len(localPlan["elementTimes"])):
            shiftedElementTimes = localPlan["elementTimes"][i]+localPlan["time"][i]
            localPlan["elementTimes"][i] = shiftedElementTimes
        data["localPlan"] = localPlan
    else:
        data["localPlan"] = []

    return [data, trial_name]