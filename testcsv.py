import matplotlib.pyplot as plt
import pandas as pd
import plotly .express as px

df = pd.read_csv('/home/will/ros-project/clone_testbed/task-allocation-testbed/data/data_log_aug.csv')

fig = px.line(df)
fig.show()
