import numpy as np
import matplotlib.pyplot as plt

result = ['gaussNewton', 'g2oCurveFitting', 'ceresCurveFitting']
output_path = 'build/output/'

def frange(start, final, increment):
    numbers=[]
    while start<final:
        numbers.append(start)
        start = start + increment
    return numbers

for name in result:
    with open(output_path + name + '_result.txt', 'r') as f:
        x_noise = []
        y_noise = []
        param = []
        for line in f.readlines():
            data = line.strip().split()

            if len(data) == 2:
                x_noise.append(float(data[0]))
                y_noise.append(float(data[1]))
            elif len(data) == 3:
                param.append([float(e) for e in data])
            elif len(data) == 1:
                useTime = data[0]
            else:
                print('Error')
        
        x_real = frange(0, 1, 0.01)
        y_real = []
        for i in x_real:
            y_real.append(np.exp(param[0][0] * i ** 2 + param[0][1] * i + param[0][2]))

        x_est = frange(0, 1, 0.01)
        y_est = []
        for i in x_est:
            y_est.append(np.exp(param[1][0] * i ** 2 + param[1][1] * i + param[1][2]))

        plt.plot(x_real, y_real, 'b', label='Real curve')
        plt.plot(x_est, y_est, 'r', label='Estimate curve')
        plt.scatter(x_noise, y_noise, s=5, c='g', label='Noise')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title(name + ' method (used time : ' + useTime + ')')
        plt.legend()
        plt.grid()
        plt.savefig(output_path + name + '_plot.png')
        plt.clf()

print('Done')