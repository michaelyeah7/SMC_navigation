import numpy as np
import matplotlib.pyplot as plt
filename = 'cal.log'
with open(filename) as f:
    lines = [float(line.rstrip()) for line in f]
# print(lines)
averaged_rewards = []
i = 0
step = 50
while  i < len(lines):
	averaged_reward = np.average(lines[i:i+step])
	averaged_rewards .append(averaged_reward)
        i += step
plt.plot(averaged_rewards)
print(averaged_rewards)
plt.savefig('rewards.png')

