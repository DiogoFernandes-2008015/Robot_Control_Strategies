# Controle por Torque Computado (CTC) em Manipuladores Seriais

Este repositório contém a implementação do **Controle por Torque Computado (Computed Torque Control - CTC)** para acompanhamento de trajetória em manipuladores robóticos rígidos de 2 Graus de Liberdade (GDL), desenvolvida no ambiente **MATLAB/Simulink**.

O projeto inclui o modelo de simulação do sistema em malha fechada, o script de pós-processamento `anima_simulink_2.m` e a rotina gráfica genérica `dinanima_ref.m` para visualização animada do robô executando a trajetória de referência.

## 📌 Conteúdo do Repositório

* **`controle_ctc_2dof.slx`**: Modelo no Simulink que implementa a dinâmica inversa do manipulador, o gerador de trajetórias e a malha de controle PD em coordenadas de juntas.

* **`anima_simulink_2.m`**: Script MATLAB executado após o término da simulação para extrair os dados da estrutura `out.logsout` e invocar a função de animação.

* **`dinanima_ref.m`**: Função responsável por calcular a cinemática direta a cada instante de tempo e renderizar a animação 2D/3D do robô, sobrepondo a trajetória percorrida com a referência desejada.

* **`README.md`**: Documentação explicativa do projeto, fundamentação teórica e especificação das funções.

---

## 🧠 Fundamentação Teórica: Controle por Torque Computado (CTC)

O **Controle por Torque Computado** é uma técnica clássica baseada no princípio de **linearização por realimentação** (*feedback linearization*). Seu objetivo é cancelar rigorosamente as forças e momentos não lineares do manipulador através da injeção do modelo dinâmico inverso na malha fechada, transformando um sistema não linear acoplado em um conjunto de integradores duplos independentes.

### 1. Modelo Dinâmico no Espaço de Juntas

A equação diferencial ordinária não linear que descreve a dinâmica de um manipulador robótico rígido de $n$ GDL é dada por:

$$
\tau = H(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q)
$$

Onde:
* $q(t) \in \mathbb{R}^n$: Vetor de posições angulares/prismáticas das juntas.
* $\dot{q}(t), \ddot{q}(t) \in \mathbb{R}^n$: Vetores de velocidade e aceleração das juntas.
* $\tau(t) \in \mathbb{R}^n$: Vetor de torques ou forças aplicados pelos atuadores.
* $H(q) \in \mathbb{R}^{n \times n}$: Matriz de inércia do robô (simétrica e definida positiva).
* $C(q, \dot{q})\dot{q} \in \mathbb{R}^n$: Vetor de forças centrífugas e de Coriolis.
* $g(q) \in \mathbb{R}^n$: Vetor de torques gravitacionais.

### 2. Lei de Controle e Linearização por Realimentação

A lei de controle não linear do CTC cancela a dinâmica inerente ao robô ao definir o torque $\tau$ como:

$$
\tau := H(q)u + C(q, \dot{q})\dot{q} + g(q)
$$

Substituindo a lei de controle no modelo dinâmico real e considerando conhecimento perfeito dos parâmetros ($\hat{H} = H, \hat{C} = C, \hat{g} = g$):

$$
H(q)u + C(q, \dot{q})\dot{q} + g(q) = H(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) \implies u = \ddot{q}
$$

A variável $u(t) \in \mathbb{R}^n$ atua como uma **aceleração virtual comandada** sobre $n$ integradores duplos lineares e totalmente desacoplados.

### 3. Projeto da Malha Auxiliar Proporcional-Derivativa (PD)

Para garantir o acompanhamento de uma trajetória desejada $q_d(t)$, define-se o erro de rastreamento $e(t) = q_d(t) - q(t)$ e projeta-se a aceleração virtual $u(t)$ como:

$$
u = \ddot{q}_d + K_v \dot{e} + K_p e
$$

Onde $K_p, K_v \in \mathbb{R}^{n \times n}$ são matrizes diagonais definidas positivas. A dinâmica do erro em malha fechada resulta em:

$$
\ddot{e} + K_v \dot{e} + K_p e = 0
$$

Escolhendo os ganhos para amortecimento crítico ($\zeta_i = 1$) por junta:
* $K_{pi} = \omega_{ni}^2$
* $K_{vi} = 2\omega_{ni}$

Garante-se a alocação de polos reais duplos em $s = -\omega_{ni}$, assegurando a convergência exponencial do erro $e(t) \to 0$ quando $t \to \infty$.

---

## 🎬 Pós-Processamento e Animação

### 1. Script Principal de Animação (`anima_simulink_2.m`)

O script `anima_simulink_2.m` conecta os resultados salvos na estrutura de simulação do Simulink (`out`) com a ferramenta de renderização gráfica:

```matlab
syms th1 th2 'real'

% Parâmetros de Denavit-Hartenberg e tipo de junta (1 = Revolução)
tdh_n = [1 0 0 th1; 1 0 0 th2];
info = [1; 1];

% Extração dos dados salvos no Simulink
qout = out.logsout{1}.Values.Data;
tout = out.tout;
xd = out.logsout{2}.Values.Data;
yd = out.logsout{3}.Values.Data;

ref = [xd yd];

% Chamada da função gráfica de animação
dinanima_ref(tdh_n, info, qout, tout, 1, ref');
```

---

### 2. Descrição da Função `dinanima_ref.m`

A função `dinanima_ref` é responsável por animar a movimentação de qualquer manipulador serial genérico de $n$ GDL a partir das trajetórias temporais de junta $q_{out}(t)$, recomputando a cinemática direta numérica quadro a quadro.

#### Assinatura da Função:
```matlab
dinanima_ref(tdh0, info, qout, tout, rastro, ref)
```

#### Parâmetros de Entrada:

| Parâmetro | Tipo / Dimensão | Descrição |
| :--- | :--- | :--- |
| `tdh0` | Matriz $n \times 4$ | Tabela de Denavit-Hartenberg padrão $[a, \alpha, d, \theta]$ contendo os parâmetros geométricos fixos do manipulador. |
| `info` | Vetor $n \times 1$ | Identificador do tipo de cada junta: `1` para junta de **revolução** ($\theta_i$ variável) e `2` para junta **prismática** ($d_i$ variável). |
| `qout` | Matriz $N_t \times n$ | Posições temporais executadas das juntas obtidas da integração/simulação. |
| `tout` | Vetor $N_t \times 1$ | Vetor com os instantes discretos de tempo da simulação. |
| `rastro` | Escalar *(Opcional)* | Habilita o rastro da trajetória executada pelo efetuador final. Usar `1` para ativar (padrão) ou `0` para desativar. |
| `ref` | Matriz *(Opcional)* | Trajetória de referência a ser plotada em verde. |

#### Formatos Suportados pelo Parâmetro `ref`:
1. **Espaço de Juntas ($N_t \times n$)**: A função aplica a cinemática direta ponto a ponto sobre a referência de junta para gerar a curva correspondente no espaço cartesiano.
2. **Espaço Cartesiano 3D ($3 \times N_t$ ou $N_t \times 3$)**: Trajetória desejada dada em coordenadas $[x, y, z]^\top$.
3. **Espaço Cartesiano 2D ($2 \times N_t$ ou $N_t \times 2$)**: Trajetória desejada dada em $[x, y]^\top$; a função automaticamente preenche a coordenada $z = 0$.

#### Elementos Gráficos Renderizados:
* 🟢 **Referência Desejada**: Curva verde pontilhada (`g--`) com um marcador circular verde (`go`) indicando a posição do alvo no instante de tempo atual $t$.
* 🔴 **Rastro Real**: Trajetória contínua em vermelho (`r-`) traçada pelo efetuador final até o instante $t$.
* 🔵 **Estrutura do Robô**: Linhas azuis com marcadores pretos nas juntas representando os elos físicos e a configuração do manipulador a cada frame.

---

## 🚀 Como Executar

### Pré-requisitos
* **MATLAB** (R2020b ou superior)
* **Simulink**
* **Symbolic Math Toolbox**

### Passo a Passo

1. Clone o repositório para o seu ambiente local:
   ```bash
   git clone https://github.com/seu-usuario/controle-ctc-simulink.git
   cd controle-ctc-simulink
   ```

2. Abra o **MATLAB** e adicione o diretório do projeto ao Path.

3. Execute o modelo no Simulink (`controle_ctc_2dof.slx`).

4. Após o término da simulação, execute o script de animação no Command Window:
   ```matlab
   run('anima_simulink_2.m')
   ```

---

## 📚 Referências

* FERNANDES, Diogo Lopes. *Controle de Torque Computado em Manipuladores Robóticos - Controle de Trajetória*. Nota Didática, 2026.
* SPONG, Mark W.; HUTCHINSON, Seth; VIDYASAGAR, M. *Robot Modeling and Control*. John Wiley & Sons, 2005.
* LUH, J. Y., WALKER, M. W., & PAUL, R. P. (1980). *On-line computational scheme for mechanical manipulators*. ASME Journal of Dynamic Systems, Measurement, and Control, 102(2), 69-76.
