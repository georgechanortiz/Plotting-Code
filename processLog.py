import matplotlib.pyplot as plt
from parseQuadBag import parseQuadBag
import numpy as np
import os
from _saveLog import saveLog
from plotState import plotState



def processLog(*args):
    # Prepare environment; close all open plots/figures
    plt.close('all')

    # Check if directory is correct; essentially make sure that script is located in quad_logger/scripts/
    if not os.getcwd().endswith("quad_logger/scripts"):
        print("This script must be run from quad_logger/scripts/")
    

    # Select a mcap file to parse through, if a trial name is provided use that
    if len(args) > 0:
        trialName = args[0]
        namespace = args[1]
    else:
        trialName = None
        namespace = ""

    # Set parameters to save and set up variables for figure titles and labels
    bSave = False
    bTitles = True
    bPlotLocalPlanInfo = True
    tWindowStates = []
    tWindowControl = []
    tWindowLocalPlan = []
    
    # Load data from parseQuadBag.py

    data, trialName = parseQuadBag(trialName, namespace)

    stateEstimate = data["stateEstimate"]
    stateGroundTruth = data["stateGroundTruth"]
    stateTrajectory = data["stateTrajectory"]
    stateGRFs = data["stateGRFs"]
    controlGRFs = data["controlGRFs"]
    localPlan = data["localPlan"]


    # Plot everything, separate by:
    # State
    stateFigs = np.array([])
    if stateGroundTruth:

        stateFigs = plotState(stateGroundTruth, tWindowStates, '-', bTitles, stateFigs)
    if stateTrajectory:
        stateFigs = plotState(stateTrajectory, tWindowStates, '--', bTitles, stateFigs)
    if stateEstimate:
        stateFigs = plotState(stateEstimate, tWindowStates, ':', bTitles, stateFigs)

    # Control
    # controlFigs = np.array([])
    # if stateGRFs:
    #     controlFigs = plotControl(stateGRFs, tWindowControl, '-', bTitles, controlFigs)
    # if controlGRFs:
    #     controlFigs = plotControl(controlGRFs, tWindowControl, '--', bTitles, controlFigs)

    # # Local Plan Information
    # localPlanFigs = np.array([])
    # if bPlotLocalPlanInfo and localPlan:
    #     localPlanFigs = plotLocalPlan(localPlan, tWindowLocalPlan, bTitles, localPlanFigs)

    # Add figs to a single folder
    figArray = stateFigs #+ controlFigs + localPlanFigs

    
    if bSave:
        logDir = saveLog(trialName, figArray)

    plt.show()

if __name__ == "__main__":
    processLog()