import matplotlib.pyplot as plt
from _parseQuadBag import parseQuadBag
import numpy as np


# Prepare environment; close all open plots/figures


# Check if directory is correct; essentially make sure that script is located in quad_logger/scripts/


# Select a mcap file to parse through, if a trial name is provided use that


# Set parameters to save and set up variables for figure titles and labels


# Load data from parseQuadBag.py

[data, trialName] = parseQuadBag(trialName=None, namespace="")

stateEstimate = data.stateEstimate
stateGroundTruth = data.stateGroundTruth
stateTrajectory = data.stateTrajectory
stateGRFs = data.stateGRFs
controlGRFs = data.controlGRFs
localPlan = data.localPlan
# Plot everything, separate by:
#       State
stateFigs = np.array([])

#       Control
controlFigs = np.array([])

#       Local Plan Information
localPlanFigs = np.array([])

# Add figs to a single folder

figArray = [stateFigs, controlFigs, localPlanFigs]