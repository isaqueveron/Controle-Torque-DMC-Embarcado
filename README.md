# Controlador Preditivo DMC Embarcado - Emulação Eólica

Este repositório contém a implementação de um sistema de Controle Preditivo Baseado em Modelo (DMC - *Dynamic Matrix Control*) restrito e chaveado, desenvolvido em Python 3 para o controle dinâmico de torque em uma bancada de emulação eólica. O sistema gerencia um motor síncrono de ímãs permanentes (PMSM) utilizando um inversor comercial WEG CFW-11 como atuador de velocidade.

## Estrutura do Projeto

* **`controlador_dmc.py`**: Módulo central que implementa a lógica do controlador DMC chaveado restrito. Realiza o carregamento dos modelos normalizados de subida/descida, montagem da matriz de resposta ao degrau ($G$) e otimização do esforço de controle ($\Delta u$) via algoritmo SLSQP (`scipy.optimize.minimize`).
* **`main_dmc_embarcado.py`**: Script principal responsável por executar o laço de controle em tempo real. Gerencia o sincronismo temporal, a leitura do torquímetro, a filtragem digital do sinal de processo (PV), o cálculo do DMC e o envio dos comandos de velocidade (MV) via comunicação serial RS-485.
* **`aquisicao_resposta_degrau.py`**: Script de automação para ensaios em malha aberta. É utilizado para a coleta de dados de torque e velocidade em diferentes faixas operacionais, permitindo extrair as curvas médias para a identificação dos modelos dinâmicos de subida e descida.
* **`monte_carlo_otimizador.py`**: Ferramenta de teste estatístico (Simulação de Monte Carlo) projetada para avaliar o tempo de processamento do pior caso (*Worst-Case Execution Time* - WCET) do solver, garantindo o determinismo e a viabilidade do sistema em tempo real.

## Configurações e Parâmetros de Controle

* **Período de Amostragem ($T_s$):** 50 ms (0.05 s)
* **Horizonte de Predição ($P$):** 15 passos
* **Horizonte de Controle ($M_c$):** 3 passos
* **Peso de Penalização ($\lambda$):** Ajustável no script principal (padrão: 0.005) para balancear a velocidade de resposta e a suavidade da ação de controle.
* **Filtragem Digital:** Filtro passa-baixas de primeira ordem com frequência de corte em 25 Hz para atenuação de ruídos elétricos e ressonâncias mecânicas da bancada.
* **Estratégia de Chaveamento:** Banco composto por 6 modelos lineares normalizados (3 faixas para aceleração e 3 para desaceleração entre 300 e 700 RPM) para contornar o comportamento assimétrico e não-linear da planta.
* **Restrições Operacionais de Segurança:** Restrições rígidas tratadas diretamente pelo solver numérico para isolar nativamente a zona morta do motor ($\omega_{ref} \ge 300$ RPM) e evitar sobretorques no eixo mecânico ($\tau \le 25$ N.m).

## Requisitos e Dependências

O projeto foi validado em ambiente Linux (Debian) utilizando Python 3.x com as seguintes bibliotecas:
* `numpy` (Álgebra linear e manipulação de matrizes)
* `scipy` (Módulo `optimize` para resolução do problema QP restrito)
* `matplotlib` (Geração de gráficos de desempenho e latência)
* Módulos locais de hardware: `driver_torquimetro`, `driver_inversor_cfw11` e `init_serial_devices`

## Como Executar

1. **Identificação da Planta:** Realize os ensaios de resposta ao degrau em malha aberta para gerar os arquivos `.csv` de calibração:
   ```bash
   python aquisicao_resposta_degrau.py
