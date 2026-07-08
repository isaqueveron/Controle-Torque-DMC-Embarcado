import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.signal import butter, filtfilt

def carregar_modelos_normalizados(arquivo_subida, arquivo_descida):
    dados_subida = {}
    dados_descida = {}
    
    faixas_permitidas = {

        '300->400', '500->600', '600->700',
        '400->300', '600->500', '700->600'
    
    }

    try:
        # Leitura subida
        with open(arquivo_subida, mode='r', encoding='utf-8') as f:
            for linha in csv.DictReader(f):
                faixa = linha['Faixa_RPM'].strip()
                if faixa in faixas_permitidas and float(linha['Tempo_Discreto_s']) >= 0:
                    if faixa not in dados_subida:
                        dados_subida[faixa] = []
                    dados_subida[faixa].append((float(linha['Tempo_Discreto_s']), float(linha['Torque_Filt_Medio_Disc_Nm'])))

        # Leitura descida
        with open(arquivo_descida, mode='r', encoding='utf-8') as f:
            for linha in csv.DictReader(f):
                faixa = linha['Faixa_RPM'].strip()
                if faixa in faixas_permitidas and float(linha['Tempo_Discreto_s']) >= 0:
                    if faixa not in dados_descida:
                        dados_descida[faixa] = []
                    dados_descida[faixa].append((float(linha['Tempo_Discreto_s']), float(linha['Torque_Filt_Medio_Disc_Nm'])))
    except Exception as e:
        print(f"Erro na leitura do CSV: {e}")
        return None, None

    if not dados_subida or not dados_descida:
        return None, None

    todas_curvas = list(dados_subida.values()) + list(dados_descida.values())
    pontos_minimos = min(len(curva) for curva in todas_curvas)
    banco_modelos = {}

    frequencia_corte_hz = 4.0
    frequencia_nyquist = 10.0
    b_coef, a_coef = butter(N=2, Wn=frequencia_corte_hz / frequencia_nyquist, btype='low')

    # Processamento subida (normalizacao)
    for faixa, valores in dados_subida.items():
        valores.sort(key=lambda x: x[0])
        torque_resposta = np.array([v[1] for v in valores[:pontos_minimos]])
        amplitude = 100.0 
        curva_step_bruta = (torque_resposta - torque_resposta[0]) / amplitude
        curva_step_filtrada = filtfilt(b_coef, a_coef, curva_step_bruta)
        curva_step_filtrada[0] = 0.0
        banco_modelos[faixa] = curva_step_filtrada

        # PLOT
        """plt.figure(figsize=(10, 4))
        plt.plot(curva_step_bruta, 'g:', alpha=0.4, label='Normalizado Bruto')
        plt.plot(curva_step_filtrada, 'b-', linewidth=2, label='Filtro Bidirecional Sem Atraso)')
        plt.title(f"Processamento de Modelo (Subida): Faixa {faixa} RPM")
        plt.xlabel("Amostras [k]")
        plt.ylabel("Ganho Unitario [N.m / RPM]")
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.show()"""

    # Processamento descida
    for faixa, valores in dados_descida.items():
        valores.sort(key=lambda x: x[0])
        torque_resposta = np.array([v[1] for v in valores[:pontos_minimos]])
        amplitude = -100.0
        curva_step_bruta = (torque_resposta - torque_resposta[0]) / amplitude 
        curva_step_filtrada = filtfilt(b_coef, a_coef, curva_step_bruta)
        curva_step_filtrada[0] = 0.0
        banco_modelos[faixa] = curva_step_filtrada

        # PLOT
        """plt.figure(figsize=(10, 4))
        plt.plot(curva_step_bruta, 'r:', alpha=0.4, label='Normalizado Bruto')
        plt.plot(curva_step_filtrada, 'b-', linewidth=2, label='Filtro Bidirecional Sem Atraso')
        plt.title(f"Processamento de Modelo (Descida): Faixa {faixa} RPM")
        plt.xlabel("Amostras [k]")
        plt.ylabel("Ganho Unitario [N.m / RPM]")
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.show()"""

    return banco_modelos, pontos_minimos


class DMC_Chaveado_Restrito:
    def __init__(self, banco_modelos, horizonte_predicao, horizonte_controle, peso_lambda):
        self.banco = banco_modelos
        self.horizonte_predicao = horizonte_predicao
        self.horizonte_controle = horizonte_controle
        self.peso_lambda = peso_lambda
        
        self.limite_rpm_min = 300.0
        self.limite_rpm_max = 1000.0
        self.limite_delta_rpm_max =  5
        
        self.limite_torque_min = 0.0     
        self.limite_torque_max = 25.0    
        
        self.matrizes_A = {}
        self.steps = {}
        
        for faixa, vetor_step in self.banco.items():
            step_recortado = vetor_step[:self.horizonte_predicao]
            self.steps[faixa] = step_recortado
            
            matriz_a = np.zeros((self.horizonte_predicao, self.horizonte_controle))
            for i in range(self.horizonte_predicao):
                for j in range(self.horizonte_controle):
                    if i >= j:
                        matriz_a[i, j] = step_recortado[i - j]
            self.matrizes_A[faixa] = matriz_a

        self.vetor_predicao_livre = np.zeros(self.horizonte_predicao)

    def obter_modelo_faixa(self, rpm_atual, subindo):
        rpm = float(rpm_atual)
        if subindo:
            if rpm <= 400: return '300->400'
            elif rpm <= 600: return '500->600'
            else: return '600->700'
        else:
            if rpm <= 400: return '400->300'
            elif rpm <= 600: return '600->500'
            else: return '700->600'

    def calcular_u(self, setpoint_torque, torque_atual, rpm_anterior):
        subindo = (setpoint_torque >= torque_atual)
        faixa = self.obter_modelo_faixa(rpm_anterior, subindo) # aqui deveria mudar pelo rpm medido de fato, mas por enquanto
                                                               # ele chaveia baseado no sinal de controle enviado antes
        matriz_A_ativa = self.matrizes_A[faixa]
        step_ativo = self.steps[faixa]
        
        erro_predicao = torque_atual - self.vetor_predicao_livre[0]
        vetor_corrigido = self.vetor_predicao_livre + erro_predicao
        vetor_setpoint = np.ones(self.horizonte_predicao) * setpoint_torque
        erro_futuro = vetor_setpoint - vetor_corrigido

        def funcao_custo(delta_u):
            erro_previsto = np.dot(matriz_A_ativa, delta_u) - erro_futuro
            termo_erro = np.sum(erro_previsto ** 2)
            termo_esforco = self.peso_lambda * np.sum(delta_u ** 2)
            return termo_erro + termo_esforco

        bounds_delta_u = [(-self.limite_delta_rpm_max, self.limite_delta_rpm_max) for _ in range(self.horizonte_controle)]

        matriz_soma_cumulativa = np.tril(np.ones((self.horizonte_controle, self.horizonte_controle)))

        restricoes = [
            {
                'type': 'ineq', 
                'fun': lambda delta_u: self.limite_rpm_max - (rpm_anterior + np.dot(matriz_soma_cumulativa, delta_u))
            },
            {
                'type': 'ineq', 
                'fun': lambda delta_u: (rpm_anterior + np.dot(matriz_soma_cumulativa, delta_u)) - self.limite_rpm_min
            },
            {
                'type': 'ineq', 
                'fun': lambda delta_u: self.limite_torque_max - (vetor_corrigido + np.dot(matriz_A_ativa, delta_u))
            },
            {
                'type': 'ineq', 
                'fun': lambda delta_u: (vetor_corrigido + np.dot(matriz_A_ativa, delta_u)) - self.limite_torque_min
            }
        ]

        delta_u_inicial = np.zeros(self.horizonte_controle)
        resultado = minimize(
                        funcao_custo, 
                        delta_u_inicial, 
                        method='SLSQP', 
                        bounds=bounds_delta_u, 
                        constraints=restricoes,
                        options={
                            'ftol': 1e-1, # tolerancia para determinar convergencia 
                            'eps': 1.0,   # um parametro usado para o "bound" do delta_u
                            'maxiter': 10 # nmbr max. de iteracoes
                        }
                    )

        """# ----- Restricoes no formato classico ensinado na disciplina, funciona mas demora mais para calular -----
        I_Mc = np.eye(self.horizonte_controle)
        T_L = np.tril(np.ones((self.horizonte_controle, self.horizonte_controle)))
        
        matriz_M = np.vstack([
            I_Mc,
            -I_Mc,
            T_L,
            -T_L,
            matriz_A_ativa,
            -matriz_A_ativa
        ])

        vetor_V = np.concatenate([
            self.limite_delta_rpm_max * np.ones(self.horizonte_controle),
            self.limite_delta_rpm_max * np.ones(self.horizonte_controle),
            (self.limite_rpm_max - rpm_anterior) * np.ones(self.horizonte_controle),
            (rpm_anterior - self.limite_rpm_min) * np.ones(self.horizonte_controle),
            self.limite_torque_max * np.ones(self.horizonte_predicao) - vetor_corrigido,
            vetor_corrigido - self.limite_torque_min * np.ones(self.horizonte_predicao)
        ])
        
        restricoes = [
            {
                'type': 'ineq',
                'fun': lambda delta_u: vetor_V - np.dot(matriz_M, delta_u)
            }
        ]

        delta_u_inicial = np.zeros(self.horizonte_controle)
        resultado = minimize(
                        funcao_custo, 
                        delta_u_inicial, 
                        method='SLSQP', 
                        constraints=restricoes,
                        options={
                            'ftol': 1e-1,       
                            'maxiter': 10       
                        }
                    )
        """
        
        delta_u_efetivo = resultado.x[0] if resultado.success else 0.0 # fallback
        proxima_predicao = np.zeros(self.horizonte_predicao)
        for i in range(self.horizonte_predicao - 1):
            proxima_predicao[i] = vetor_corrigido[i + 1] + step_ativo[i + 1] * delta_u_efetivo
        proxima_predicao[-1] = vetor_corrigido[-1] + step_ativo[-1] * delta_u_efetivo
        
        self.vetor_predicao_livre = proxima_predicao

        rpm_aplicada = rpm_anterior + delta_u_efetivo
        return rpm_aplicada