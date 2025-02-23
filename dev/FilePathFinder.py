import pandas as pd

def getPath(name,csvFile="../paths.csv"):
    paths = pd.read_csv(csvFile)["path"]
    if name == "chessEngine":
        return paths[0]
    elif name == "voiceModel":
        return paths[1]
    else:
        return ""

