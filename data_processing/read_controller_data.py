import matplotlib.pyplot as plt

f = open('C:/Users/shenc/OneDrive/Sync/Academics/Year 4/FYP/Trained Models/SingleRacecarBullet-FPV-30cm/controller/controller-18-fpv-cropped.txt', 'r')
f2 = open('C:/Users/shenc/OneDrive/Sync/Academics/Year 4/FYP/Trained Models/SingleRacecarBullet-TopDown-30cm/controller/controller-18-tpv-cropped.txt', 'r')

avg_performance = []
std_deviation = []

max_perf = []
min_perf = []

target_string = "('single_racecar_bullet',"

for line in f:

    if target_string in line:
        
        word_string = line.split(' ')
        idx = word_string.index(target_string)
        avg_performance.append(float(word_string[idx+3][:-1]))
        max_perf.append(float(word_string[idx+5][:-1]))
        min_perf.append(float(word_string[idx+4][:-1]))

        std_deviation.append(float(word_string[idx+6][:-1]))

f.close()

top_perf = [avg_performance[i] + std_deviation[i] for i in range(len(avg_performance))]
btm_perf = [avg_performance[i] - std_deviation[i] for i in range(len(avg_performance))]

x = [i for i in range(len(avg_performance))]

plt.plot(avg_performance, label='First-Person View Average Reward')
plt.fill_between(x, btm_perf, top_perf, alpha=0.2)

f2_avg_performance = []
f2_std_deviation = []

f2_max_perf = []
f2_min_perf = []

target_string = "('single_racecar_bullet',"

for line in f2:

    if target_string in line:
        
        word_string = line.split(' ')
        idx = word_string.index(target_string)
        f2_avg_performance.append(float(word_string[idx+3][:-1]))
        f2_max_perf.append(float(word_string[idx+5][:-1]))
        f2_min_perf.append(float(word_string[idx+4][:-1]))

        f2_std_deviation.append(float(word_string[idx+6][:-1]))

f2.close()

f2_top_perf = [f2_avg_performance[i] + f2_std_deviation[i] for i in range(len(f2_avg_performance))]
f2_btm_perf = [f2_avg_performance[i] - f2_std_deviation[i] for i in range(len(f2_avg_performance))]

x2 = [i for i in range(len(f2_avg_performance))]

plt.plot(f2_avg_performance, label='Top-Down View Average Reward')
plt.fill_between(x2, f2_btm_perf, f2_top_perf, alpha=0.2)

plt.legend(loc='lower right')
plt.title('Controller Reward Evolution')# (Top-Down View)')
plt.xlabel('Number of Generations')
plt.ylabel('Reward')
plt.show()