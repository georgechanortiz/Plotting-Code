import os
import matplotlib.pyplot as plt

def saveLog(trialName, figArray):
    logDir = os.path.join("logs", trialName)
    os.makedirs(logDir, exist_ok=True)

    for i, fig in enumerate(figArray):
        fig_path = os.path.join(logDir, f"figure_{i+1}.png")
        fig.savefig(fig_path)
    return logDir