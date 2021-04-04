import matplotlib.pyplot as plt

losses = open('C:/Users/shenc/OneDrive/Sync/Academics/Year 4/FYP/Trained Models/SingleRacecarBullet-FPV-30cm/vae/loss.txt', 'r')
reconstruction_losses = open('C:/Users/shenc/OneDrive/Sync/Academics/Year 4/FYP/Trained Models/SingleRacecarBullet-FPV-30cm/vae/reconstruction_loss.txt', 'r')
kl_losses = open('C:/Users/shenc/OneDrive/Sync/Academics/Year 4/FYP/Trained Models/SingleRacecarBullet-FPV-30cm/vae/kl_loss.txt', 'r')


data = []
reconstruction_loss_data = []
kl_loss_data = []

for loss in losses:
    data.append(float(loss))

for loss in reconstruction_losses:
    reconstruction_loss_data.append(float(loss))

for loss in kl_losses:
    kl_loss_data.append(float(loss))

plt.plot(data, label = 'Total Loss')
plt.plot(reconstruction_loss_data, label = 'Reconstruction Loss')
plt.plot(kl_loss_data, label = 'KL Loss')
plt.legend()
plt.title('VAE Loss (First-Person View)')
plt.xlabel('Number of Epochs')
plt.ylabel('Loss')
plt.show()