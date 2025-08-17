import pandas as pd

# READ BEFORE USE:

# Decide whether to use the predefined gain schedule or a directly calculate the gains.
#   1. If you want to use the predefined gain schedule, use THIS FILE and make sure to have the MOST UP TO DATE 'gain_schedule.csv' file in the same directory. 
#   2. If you want to calculate the gains directly (computationally more heavy in flight but more accurate) USE THE OTHER FILE Gain_schedule_direct.py

# Note: This file only schedules gains with velocity because the aerodynamic gain is a function of velocity squared, thereby making it the most important factor.
# A future improvement could be to include altitude as well, perhaps in the form of a cumulative dynamic pressure variable, but that is not currently implemented.

# 1. PREDEFINED GAIN SCHEDULE

gain_schedule = pd.read_csv('gain_schedule.csv')

# 3. Operation
# Inputs: alt (altitude) and V (velocity)
# Outputs: KP, KI, KD (PID gains)

def get_gains_for_velocity(current_velocity):
    # Assumes gain_schedule is sorted by velocity ascending
    velocities = gain_schedule['velocity'].values
    idx = velocities.searchsorted(current_velocity, side='right') - 1
    if idx < 0:
        idx = 0
    row = gain_schedule.iloc[idx]
    return row['KP'], row['KI'], row['KD']
