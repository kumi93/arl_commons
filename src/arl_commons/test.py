import os
import pandas as pd

cwd = os.path.abspath(os.path.join(__file__, "../.."))



path = os.path.dirname(cwd) + '/config/pressures_trajectory.csv'
df = pd.read_csv(path)

print path
print df