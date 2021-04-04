f = open('C:/Users/shenc/OneDrive/Sync/Academics/Year 4/FYP/Trained Models/SingleRacecarBullet-FPV-30cm/rnn/RNN.out', 'r')
results = open('C:/Users/shenc/OneDrive/Sync/Academics/Year 4/FYP/Trained Models/SingleRacecarBullet-FPV-30cm/rnn/loss.txt','w+')
counter = 3

target_string = 'loss:'

for line in f:

    if counter == 0:
        word_string = line.split(' ')
        idx = word_string.index(target_string)
        results.write(word_string[idx+1] + '\n')
    
    counter -= 1

    if counter < 0:
        counter = 3

f.close()
results.close()