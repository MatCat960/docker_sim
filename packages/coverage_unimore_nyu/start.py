
import numpy as np
import os

ROBOTS_NUM = 16
GUI = False
# read positions from file
file = os.path.join(os.getcwd(), 'pos.txt')
data = []
with open(str(file), 'r') as f:
    lines = f.readlines()

for l in lines:
    data.append(l)

t = np.zeros((len(data), 3))

for i in range(len(data)):
  data[i] = data[i].replace('\n', '')
  t[i] = tuple(map(float, data[i].split(' ')))

cmd = "roslaunch coverage_unimore_nyu distri_supervisor_sim.launch "
for i in range(ROBOTS_NUM):
   cmd += f"x{i}:={t[i,0]}" + " " + f"y{i}:={t[i,1]}" + " " + f"th{i}:={t[i,2]}" + " "

if GUI:
  cmd += "gui:=true"
else:
  cmd += "gui:=false"

print("Sending command:")
print(cmd)

# run ros launch file
os.system(cmd)


