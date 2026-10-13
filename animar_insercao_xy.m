function animar_insercao_xy(t, pos, F, x_entrada)
% ANIMAR_INSERCAO_XY Animação com enquadramento dinâmico dos eixos
%
% Exemplo de chamada:
%   animar_insercao_xy(out.tout, out.posicao.Data, out.forca.Data);
%   ou especificando a entrada do furo:
%   animar_insercao_xy(out.tout, out.posicao.Data, out.forca.Data, 1.10);

    %% 1. Tratamento e Limpeza dos Dados
    t = squeeze(t);
    pos = squeeze(pos);
    F = squeeze(F);

    if size(pos, 1) == 2 && size(pos, 2) ~= 2, pos = pos'; end
    if size(F, 1) == 2 && size(F, 2) ~= 2, F = F'; end

    [t, idx_unico] = unique(t);
    pos = pos(idx_unico, :);
    F = F(idx_unico, :);

    % Conversão mm -> m se necessário
    if max(abs(pos(:,1))) > 10
        pos = pos / 1000;
    end

    %% 2. Configuração Dinâmica do Furo com base nos seus dados
    prof_furo = 0.10;      % Profundidade do furo em metros (10 cm)
    largura_furo = 0.04;   % Abertura em Y (4 cm)
    
    % Se x_entrada não for informado, posiciona o furo no final do movimento
    if nargin < 4 || isempty(x_entrada)
        x_entrada = max(pos(:,1)) - prof_furo; % Ex: 1.19 - 0.10 = 1.09 m
    end

    largura_pino = 0.036;    % Dimensão Y do pino (3.6 cm)
    comprimento_pino = 0.10; % Dimensão X do pino (10 cm)
    escala_forca = 0.001;    % Escala para os vetores de força

    %% 3. Reamostragem em Tempo Real (30 FPS)
    fps = 30;
    dt_real = 1 / fps;
    t_anim = (t(1):dt_real:t(end))';

    pos_anim = interp1(t, pos, t_anim);
    F_anim = interp1(t, F, t_anim);

    %% 4. Criação da Janela
    fig = figure('Name', 'Simulação do Controle Híbrido', ...
                 'Color', 'w', 'Position', [100, 100, 1150, 600]);

    % Painel Principal da Animação 2D
    ax1 = subplot(2, 2, [1 3]);
    hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    xlabel(ax1, 'Posição X (m)'); ylabel(ax1, 'Posição Y (m)');
    title(ax1, 'Animação da Inserção Pino-no-Furo');

    % ENQUADRAMENTO DINÂMICO DOS EIXOS (Evita que o pino fique fora da tela)
    x_min = min([pos_anim(:,1); x_entrada]) - 0.10;
    x_max = max([pos_anim(:,1); x_entrada + prof_furo]) + 0.05;
    y_min = min(pos_anim(:,2)) - 0.12;
    y_max = max(pos_anim(:,2)) + 0.12;

    xlim(ax1, [x_min, x_max]);
    ylim(ax1, [y_min, y_max]);

    % Subplot X(t)
    ax2 = subplot(2, 2, 2);
    hold(ax2, 'on'); grid(ax2, 'on');
    plot(ax2, t, pos(:,1), 'k--', 'LineWidth', 1);
    h_mark_x = plot(ax2, t_anim(1), pos_anim(1,1), 'ro', 'MarkerFaceColor', 'r');
    xlabel(ax2, 'Tempo (s)'); ylabel(ax2, 'X (m)');
    title(ax2, 'Evolução da Posição X');

    % Subplot Forças(t)
    ax3 = subplot(2, 2, 4);
    hold(ax3, 'on'); grid(ax3, 'on');
    plot(ax3, t, F(:,1), 'b-', 'DisplayName', 'Fx');
    plot(ax3, t, F(:,2), 'r-', 'DisplayName', 'Fy');
    h_mark_f = plot(ax3, t_anim(1), F_anim(1,1), 'bo', 'MarkerFaceColor', 'b');
    legend(ax3, 'Location', 'northwest');
    xlabel(ax3, 'Tempo (s)'); ylabel(ax3, 'Força (N)');
    title(ax3, 'Forças de Contato');

    %% 5. Desenho do Furo (Estático)
    y_furo = 0; % Centro em Y
    y_sup = y_furo + largura_furo/2;
    y_inf = y_furo - largura_furo/2;

    fill(ax1, [x_entrada, x_entrada + prof_furo, x_entrada + prof_furo, x_entrada], ...
              [y_sup, y_sup, y_sup + 0.10, y_sup + 0.10], [0.5 0.5 0.5], 'EdgeColor', 'k');
    fill(ax1, [x_entrada, x_entrada + prof_furo, x_entrada + prof_furo, x_entrada], ...
              [y_inf, y_inf, y_inf - 0.10, y_inf - 0.10], [0.5 0.5 0.5], 'EdgeColor', 'k');
    line(ax1, [x_entrada + prof_furo, x_entrada + prof_furo], [y_inf, y_sup], 'Color', 'k', 'LineWidth', 2.5);

    %% 6. Elementos Dinâmicos (Pino e Força)
    h_traj = plot(ax1, pos_anim(1,1), pos_anim(1,2), 'b:', 'LineWidth', 1.5);
    
    % Retângulo do pino inicializado
    x_pino_ini = pos_anim(1,1) - comprimento_pino;
    y_pino_ini = pos_anim(1,2) - largura_pino/2;
    h_pino = rectangle(ax1, 'Position', [x_pino_ini, y_pino_ini, comprimento_pino, largura_pino], ...
                       'FaceColor', [0.2 0.7 0.3], 'EdgeColor', 'k', 'LineWidth', 1.5);

    h_forca = quiver(ax1, 0, 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    h_txt = text(ax1, x_min + 0.02, y_max - 0.03, '', 'FontSize', 10, 'FontWeight', 'bold');

    %% 7. Loop de Animação
    for k = 1:length(t_anim)
        if ~ishandle(fig), break; end

        tic;

        px = pos_anim(k, 1);
        py = pos_anim(k, 2);

        % Atualiza a posição do Pino (caixa verde)
        set(h_pino, 'Position', [px - comprimento_pino, py - largura_pino/2, comprimento_pino, largura_pino]);
        
        % Trajetória percorrida pela ponta
        set(h_traj, 'XData', pos_anim(1:k, 1), 'YData', pos_anim(1:k, 2));

        % Vetor da força de contato na ponta
        set(h_forca, 'XData', px, 'YData', py, ...
                     'UData', F_anim(k,1)*escala_forca, ...
                     'VData', F_anim(k,2)*escala_forca);

        % Marcadores de tempo nos gráficos ao lado
        set(h_mark_x, 'XData', t_anim(k), 'YData', px);
        set(h_mark_f, 'XData', t_anim(k), 'YData', F_anim(k,1));

        % Status do processo
        if px < x_entrada
            status = 'Aproximação em X'; color_txt = 'k';
        elseif px >= x_entrada && px < x_entrada + prof_furo
            status = sprintf('Inserção (Profundidade = %.3f m)', px - x_entrada); color_txt = [0 0.5 0];
        else
            status = 'Contato com Fundo / Ajuste de Força Fx'; color_txt = 'r';
        end
        set(h_txt, 'String', sprintf('Tempo: %.2f s\n%s', t_anim(k), status), 'Color', color_txt);

        drawnow;

        t_proc = toc;
        if dt_real > t_proc
            pause(dt_real - t_proc);
        end
    end
end