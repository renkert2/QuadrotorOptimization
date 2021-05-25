#%%
import json
import pandas as p
import re
import numpy as np

# Read Cross Reference File
with open('./PERFILES_WEB/PER2_TITLEDAT.DAT','r') as crfile:
    pairs = {}
    for line in crfile:
        pair = line.strip().split()
        if len(pair) == 2:
            pair[0] = pair[0].strip("PER3_")
            pairs[pair[1]] = pair[0]

with open('PropCrossReference.json', 'w') as outjson:
    json.dump(pairs, outjson, indent=4)
# Read Technical Data File into initial component dictionary
df = p.read_excel("APC_Technical_Data.xlsx", sheet_name="PRODUCT LIST")
df = df.iloc[:,list(map(lambda x: x-1, [1,2,3,5,10,11,12]))]
df = df.rename(columns={"Product Name":"Model", "Product Description":"Description", "Product Code (SKU)":"SKU", "Price ($)":"Price", "Weight":"Mass", "Diameter (INCHES)":"D", "Pitch (INCHES)":"P"})

# Remove Bad Columns
df.dropna(subset=["Mass","D","P"], how='any', inplace=True)

# Clean Columns
df = df.replace(to_replace="\n", value=" ", regex=True)

def clean_mass_fun(x):
    if type(x) == str:
        y = float(re.findall(r"[-+]?\d*\.\d+|\d+",str(x))[0])
    else:
        y = float(x)
    return y

df["Mass"] = (df["Mass"]).map(clean_mass_fun)

# Unit Conversions
ozTokG = lambda x: x*0.0283495
inToM = lambda x: x*0.0254

df["Mass"] = df["Mass"].map(ozTokG)
df["D"] = df["D"].map(inToM)
df["P"] = df["P"].map(inToM)

# Parameter Templates
priceTemplate = lambda x: {"Sym":"Price", "Value":x, "Unit": "USD", "Description":"Price"}
massTemplate = lambda x: {"Sym":"Mass", "Value":x, "Unit": "kg", "Description":"Mass"}
dTemplate = lambda x: {"Sym":"D", "Value":x, "Unit": "m", "Description":"Diameter"}
pTemplate = lambda x: {"Sym":"P", "Value":x, "Unit": "m", "Description":"Pitch"}

df["Price"] = df["Price"].map(priceTemplate)
df["Mass"] = df["Mass"].map(massTemplate)
df["D"] = df["D"].map(dTemplate)
df["P"] = df["P"].map(pTemplate)

# Add Component and Make information
df.insert(0,"Component", "Propeller")
df.insert(1,"Make", "APC")

# Load Aero Data
with open('StaticAeroData.json', 'r') as aero_file:
    aero_dict = json.load(aero_file)

# %% Process Aero Data
N = len(df) # Number of components from Technical Data Spreadsheet
Cp = np.full([N,1], np.nan)
Ct = np.full([N,1], np.nan)
models = list(df["Model"])
for i in range(len(df)):
    model = models[i]
    if model in pairs:
        filekey = pairs[model]
        if filekey in aero_dict:
            aero_data = aero_dict[filekey]
            cpdat = np.array(aero_data["Cp"])
            ctdat = np.array(aero_data["Ct"])
            rpmdat = np.array(aero_data["RPM"])

            rpmmax = 10000
            rpmmin = 1000
            filterindex = np.logical_and(rpmdat > max(min(rpmdat),rpmmin), rpmdat < min(max(rpmdat), rpmmax))

            Cp[i] = np.mean(cpdat[filterindex])
            Ct[i] = np.mean(ctdat[filterindex])

# %% Append Cp and Ct to df
df["Cp"] = Cp
df["Ct"] = Ct
df_clean = df.dropna()

CpTemplate = lambda x: {"Sym":"k_P", "Value":x, "Unit": "none", "Description":"Power Coefficient"}
CtTemplate = lambda x: {"Sym":"k_T", "Value":x, "Unit": "none", "Description":"Thrust Coefficient"}
df_clean["Cp"] = df_clean["Cp"].map(CpTemplate)
df_clean["Ct"] = df_clean["Ct"].map(CtTemplate)

# %% Convert to CopmonentData Format
# Combine Separate Cols with params into list
paramcols = ["Price", "Mass", "D", "P", "Cp", "Ct"]
df_clean["Data"] = df_clean[paramcols].values.tolist()
df_clean.drop(paramcols, axis=1, inplace=True)

comp_dict = df_clean.to_dict('records')
# %% Export to Json
with open("PropellerComponentDatabase.json", 'w') as outfile:
    json.dump(comp_dict, outfile, indent=4)
# %%
