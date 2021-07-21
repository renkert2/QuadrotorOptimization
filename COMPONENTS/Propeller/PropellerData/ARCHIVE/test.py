from Parser_UIUC_AeroData.propeller_data_util import prop_File_Filter
import csv
import os

dir_path = os.getcwd() + "\\"

paths = ["Propeller_Data_V1", "Propeller_Data_V2", "Propeller_Data_V3"]

propTable = [["FileName", "Diameter (m)", "Pitch (m)", "Path"]]

for p in paths:
    filenames,diameters,pitches,AbsolutePaths = prop_File_Filter(p, contains="static", metric=True, verbose=True)
    for i in range(len(filenames)):
        path = dir_path+AbsolutePaths[i]
        propTable.append([filenames[i], diameters[i], pitches[i], path])

#print(propTable)

# df = pd.DataFrame(propTable, columns = ["FileNames", "Diameters", "Pitches", "AbsPaths"])
# print(df)

with open("propDataTable.csv",'w') as myFile:
    wr = csv.writer(myFile)
    wr.writerows(propTable)