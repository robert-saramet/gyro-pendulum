import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

sns.set()

data = pd.read_csv('raw.csv')

plt.plot(data['time'], data['angle'])

plt.title('Grafic Pendul')

plt.xlabel('Timp')
plt.ylabel('Unghi')

plt.show()
