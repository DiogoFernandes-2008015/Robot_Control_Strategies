import numpy as np
import matplotlib.pyplot as plt

# Parâmetros do Sistema
m, b, k_s = 1.0, 2.0, 10.0
dt = 0.001
T = 2.0
time = np.arange(0, T, dt)
N = len(time)

# Trajetória Desejada e Perturbação Senoidal
x_d = np.sin(np.pi * time)
dx_d = np.pi * np.cos(np.pi * time)
ddx_d = - (np.pi**2) * np.sin(np.pi * time)
d_pert = 5.0 * np.sin(2 * np.pi * time)  # Perturbação senoidal desconhecida

# Configuração do ILC (Arimoto)
num_iteracoes = 100
Gamma = 0.8  # Ganho de aprendizado (0 < Gamma < 2m)
u_ilc = np.zeros(N) # Sinal feedforward inicial (k=1)

erros_rms = []

plt.figure(figsize=(10, 5))

for k in range(num_iteracoes):
    x = np.zeros(N)
    dx = np.zeros(N)
    dx[0] = dx_d[0] # Condição inicial perfeita
    
    e = np.zeros(N)
    de = np.zeros(N)
    
    # Simulação da dinâmica do bloco no ciclo k (Euler)
    for i in range(N - 1):
        e[i] = x_d[i] - x[i]
        de[i] = dx_d[i] - dx[i]
        
        # Força total = Controle ILC + Perturbação
        F_total = u_ilc[i] + d_pert[i]
        
        # Aceleração m*ddx + b*dx + k_s*x = F_total
        ddx = (F_total - b * dx[i] - k_s * x[i]) / m
        
        # Integração
        dx[i+1] = dx[i] + ddx * dt
        x[i+1] = x[i] + dx[i] * dt
        
    e[-1] = x_d[-1] - x[-1]
    de[-1] = dx_d[-1] - dx[-1]
    
    # Erro RMS do ciclo
    rms = np.sqrt(np.mean(e**2))
    erros_rms.append(rms)
    
    # Plota algumas iterações
    if k in [0, 2, 5, 14]:
        plt.plot(time, x, label=f'Iteração {k+1}')
        
    # LEI DE ARIMOTO (Atualização Off-line para o próximo ciclo)
    u_ilc = u_ilc + Gamma * de

plt.plot(time, x_d, 'k--', linewidth=2, label='Desejado (\(x_d\))')
plt.title('Evolução da Trajetória do Bloco com a Lei de Arimoto')
plt.xlabel('Tempo (s)')
plt.ylabel('Posição (m)')
plt.legend()
plt.grid(True)
plt.show()

# Gráfico da convergência do erro
plt.figure(figsize=(6, 4))
plt.plot(range(1, num_iteracoes + 1), erros_rms, 'o-r')
plt.title('Convergência do Erro RMS por Iteração')
plt.xlabel('Iteração (k)')
plt.ylabel('Erro RMS (m)')
plt.grid(True)
plt.show()
