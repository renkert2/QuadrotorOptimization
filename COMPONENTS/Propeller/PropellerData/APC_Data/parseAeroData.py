#%%
from io import StringIO
import pandas as pd
from pandas.io import pickle
import json

#%%
keys = []
tables = []
blank_counter = 0
go_flag = False
row_accum = ""
with open('./PERFILES_WEB/PER2_STATIC-2.DAT', "r") as aerofile:
    for line in aerofile:    
        if not line.strip():
            blank_counter += 1
        else:
            if blank_counter >= 2:
                keys.append(line.strip())
                if row_accum:
                    tables.append(row_accum)
                row_accum = ""
                go_flag = True
            elif go_flag:
                row_accum += line
            blank_counter = 0
    if row_accum:
        tables.append(row_accum) # Clean up last table

#%% Read Tables into DataFrames
data_dicts = []
for tblstring in tables:
    df = pd.read_csv(StringIO(tblstring), delim_whitespace=True) 
    df.drop([0],inplace=True)
    df = df.astype('float64')
    df.rename({"THRUST":"THRUST_LBF", "POWER":"POWER_HP", "TORQUE":"TORQUE_IN-LBF"}, inplace=True, axis=1)
    d = df.to_dict(orient='list')
    data_dicts.append(d)
# %% Combine into dictionary
aero_dict = dict(zip(keys,data_dicts))

# %% Export as Json
with open('StaticAeroData.json', 'w') as outjson:
    json.dump(aero_dict, outjson, indent=4)
# %%
