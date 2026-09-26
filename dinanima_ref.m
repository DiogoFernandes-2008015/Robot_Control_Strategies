function dinanima_ref(tdh0, info, qout, tout, rastro, ref)
%Função para animar o movimento de um manipulador serial genérico a
%partir da trajetória de juntas qout(t) obtida da integração numérica
%das equações de movimento (dinnum.m), recalculando a cinemática direta
%numérica a cada instante de tempo.
%
%tdh0   - tabela DH (n x 4) [a alpha d theta] com os parâmetros
%         geométricos fixos (a, alpha, d)
%info   - vetor (n x 1): 1 = junta de revolução, 2 = junta prismática
%qout   - matriz (N x n) com a posição executada de cada junta
%tout   - vetor (N x 1) com os instantes de tempo correspondentes
%rastro - (opcional) 1 para desenhar o rastro do efetuador (padrão: 1)
%ref    - (opcional) Trajetória de referência. Pode ser:
%         - Matriz Cartesiana (3 x Nt) ou (Nt x 3) [x, y, z]
%         - Matriz Cartesiana 2D (2 x Nt) ou (Nt x 2) [x, y]
%         - Matriz de Juntas de Referência q_ref (Nt x n)

if nargin < 5 || isempty(rastro)
    rastro = 1;
end
if nargin < 6
    ref = [];
end

N = size(tdh0);
n = N(1);
Nt = length(tout);

% --- Pré-processamento da Trajetória de Referência ---
r_ref = [];
if ~isempty(ref)
    [rows, cols] = size(ref);

    % Caso 1: Referência dada em coordenadas de junta (q_ref: Nt x n)
    if rows == Nt && cols == n
        r_ref = zeros(3, Nt);
        for k = 1:Nt
            A = eye(4);
            for i = 1:n
                a = tdh0(i,1); alp = tdh0(i,2); d = tdh0(i,3); tta = tdh0(i,4);
                if info(i) == 1
                    tta = ref(k,i);
                elseif info(i) == 2
                    d = ref(k,i);
                end
                A = A * [cos(tta) -sin(tta)*cos(alp)  sin(tta)*sin(alp) a*cos(tta);
                         sin(tta)  cos(tta)*cos(alp) -cos(tta)*sin(alp) a*sin(tta);
                         0         sin(alp)           cos(alp)          d;
                         0         0                  0                 1];
            end
            r_ref(:,k) = A(1:3,4);
        end

    % Caso 2: Referência dada em coordenadas cartesianas (2D ou 3D)
    else
        if cols == Nt && (rows == 2 || rows == 3)
            r_ref = ref;
        elseif rows == Nt && (cols == 2 || cols == 3)
            r_ref = ref';
        end

        % Se for 2D, preenche o eixo Z com zeros
        if size(r_ref, 1) == 2
            r_ref = [r_ref; zeros(1, Nt)];
        end
    end
end

% Alcance máximo do manipulador
alc = 1.2 * sum(abs(tdh0(:,1))) + 1e-6;

figure;
axis equal; grid on; hold on;
xlabel('x'); ylabel('y'); zlabel('z');

efetuador = zeros(3, Nt);

for k = 1:Nt
    A = eye(4);
    P = [0; 0; 0];
    
    % Cinemática Direta do robô na posição atual qout
    for i = 1:n
        a = tdh0(i,1); alp = tdh0(i,2); d = tdh0(i,3); tta = tdh0(i,4);
        if info(i) == 1
            tta = qout(k,i);
        elseif info(i) == 2
            d = qout(k,i);
        end
        A = A * [cos(tta) -sin(tta)*cos(alp)  sin(tta)*sin(alp) a*cos(tta);
                 sin(tta)  cos(tta)*cos(alp) -cos(tta)*sin(alp) a*sin(tta);
                 0         sin(alp)           cos(alp)          d;
                 0         0                  0                 1];
        P = [P A(1:3,4)];
    end
    efetuador(:,k) = P(:,end);

    cla;

    % 1. Plota a Trajetória de Referência completa e o alvo atual em tempo real
    if ~isempty(r_ref)
        plot3(r_ref(1,:), r_ref(2,:), r_ref(3,:), 'g--', 'LineWidth', 1.5); % Curva completa
        plot3(r_ref(1,k), r_ref(2,k), r_ref(3,k), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g'); % Alvo no tempo t
    end

    % 2. Plota o Rastro da trajetória real do efetuador
    if rastro == 1
        plot3(efetuador(1,1:k), efetuador(2,1:k), efetuador(3,1:k), 'r-', 'LineWidth', 1.5);
    end

    % 3. Plota os elos do manipulador
    plot3(P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2, 'MarkerSize', 6, 'Color', 'b', 'MarkerFaceColor', 'k');

    grid on;
    title(['t = ' num2str(tout(k), '%.2f') ' s']);
    drawnow;
end
end