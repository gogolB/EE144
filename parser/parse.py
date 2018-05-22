import re
filename = 'output_raw1.txt'
pattern  = '(?<=position\: \[)-?\d+.\d+(e-?\d+)?, -?\d+.\d+(e-?\d+)?'
new_file = 'DataPoints.txt'
DataPoints = []
p = re.compile(pattern)


# Make sure file gets closed after being iterated
with open(filename, 'r') as f:
  # Read the file contents and generate a list with each line
  lines = f.readlines()
  listSize = len(lines)
  for x in range(0,listSize-1):
    match = re.search(pattern,lines[x])
    if match:
      newLine = match.group(0) + '\n'
      print(newLine)
      DataPoints.append(newLine)
  
with open(new_file, 'w') as f:
     # go to start of file
     f.seek(0)
     # actually write the lines
     f.writelines(DataPoints)

#print(len(lines))
#print ("The string that will be searched:",'\n')
#print (lines[2])
#match = re.search(pattern,lines[2])
#print(match)
#if match:
  #print(match.group(0))
#print(DataPoints)
