import time
import numpy as np
import matplotlib.pyplot as plt

from dmc import carregar_modelos_normalizados, DMC_Chaveado_Restrito
from driver_torquimetro import Torquimeter
from driver_inversor_cfw11 import Inverter
from init_serial_devices import selecionar_porta

# CONFIGURACOES
TEMPO_AMOSTRAGEM = 0.05  # Ts em segundos
TEMPO_AMOSTRAGEM_MS = TEMPO_AMOSTRAGEM * 1000.0

FREQUENCIA_CORTE_HZ = 25.0 
CONSTANTE_TEMPO = 1.0 / (2 * np.pi * FREQUENCIA_CORTE_HZ)
FILTRO_ALFA = TEMPO_AMOSTRAGEM / (CONSTANTE_TEMPO + TEMPO_AMOSTRAGEM)

HORIZONTE_CONTROLE = 3
HORIZONTE_PREDICAO = 15
PESO_LAMBDA = 0.005

RPM_INICIAL = 700.0
TEMPO_ESTABILIZACAO_SEG = 10.0
TEMPO_TOTAL_ENSAIO_SEG = 90.0

def calcular_estatisticas(vetor_tempos_ms):
    vetor = np.array(vetor_tempos_ms)
    if len(vetor) == 0:
        return {'max': 0.0, 'media': 0.0, 'p95': 0.0, 'p99': 0.0}
    return {
        'max': np.max(vetor),
        'media': np.mean(vetor),
        'p95': np.percentile(vetor, 95),
        'p99': np.percentile(vetor, 99)
    }

def main():
    banco_modelos, total_pontos = carregar_modelos_normalizados('modelo_medio_DMC.csv', 'modelo_medio_DMC_decrescente.csv')
    if banco_modelos is None: return
    
    controlador_dmc = DMC_Chaveado_Restrito(banco_modelos, HORIZONTE_PREDICAO, HORIZONTE_CONTROLE, PESO_LAMBDA)

    porta_torq = selecionar_porta("Torquimetro")
    porta_inv = selecionar_porta("Inversor")
    if not porta_torq or not porta_inv: return

    torquimetro = Torquimeter(Port=porta_torq, Baudrate=230400, Timeout=0.003)
    inversor = Inverter(Port=porta_inv, ADR=1, Baudrate=57600, Timeout=0.003)

    historico_tempo, historico_sp, historico_pv, historico_mv = [], [], [], []
    latencias_torquimetro, latencias_otimizador, latencias_inversor, latencias_ciclo_total = [], [], [], []

    try:
        inversor.ActivateMotor()
        time.sleep(0.1)
        # estabilizamos a saida no ponto de operacao inicial RPM = 300
        inversor.SendReferenceAngularVelocity(RPM_INICIAL)
        time.sleep(TEMPO_ESTABILIZACAO_SEG)

        # iniciamos o vetor de predicao livre lendo a saida 
        # como sabemos que esta em regime permanente
        # enchemos esse vetor com esse valor medido
        torquimetro.ReadRaw() # envia o comando que pede a leitura y(k)
        torque_filtrado_anterior = torquimetro.Torque_calibrated # a mais recente leitura
        controlador_dmc.vetor_predicao_livre = np.ones(controlador_dmc.horizonte_predicao) * torque_filtrado_anterior

        tempo_inicio_ensaio = time.perf_counter()
        tempo_ultimo_ciclo = tempo_inicio_ensaio
        contador_iteracao = 0
        rpm_anterior = RPM_INICIAL

        while (time.perf_counter() - tempo_inicio_ensaio) < TEMPO_TOTAL_ENSAIO_SEG:
            tempo_atual = time.perf_counter()
            
            if (tempo_atual - tempo_ultimo_ciclo) >= TEMPO_AMOSTRAGEM:
                t_inicio_ciclo = time.perf_counter()
                tempo_decorrido = tempo_atual - tempo_inicio_ensaio
                
                # Leitura da variavel y(k)
                t0 = time.perf_counter()
                torquimetro.ReadRaw()
                torque_bruto = torquimetro.Torque_calibrated
                latencias_torquimetro.append((time.perf_counter() - t0) * 1000.0)

                # Aplicacao do filtro na variavel y(k)
                torque_filtrado_atual = (FILTRO_ALFA * torque_bruto) + ((1.0 - FILTRO_ALFA) * torque_filtrado_anterior)

                # Gerador de Referencia
                ciclo_tempo = tempo_decorrido % 100
                if   ciclo_tempo < 10: setpoint_atual = 05.0
                elif ciclo_tempo < 20: setpoint_atual = 10.0
                elif ciclo_tempo < 30: setpoint_atual = 15.0
                elif ciclo_tempo < 40: setpoint_atual = 20.0
                elif ciclo_tempo < 50: setpoint_atual = 24.0
                elif ciclo_tempo < 60: setpoint_atual = 20.0
                elif ciclo_tempo < 70: setpoint_atual = 15.0
                elif ciclo_tempo < 80: setpoint_atual = 10.0
                elif ciclo_tempo < 90: setpoint_atual = 05.0

                """# simulacao de perturbacao
                setpoint_atual = 15.0
                perturbacao = 0.0
                if 10.0 <= tempo_decorrido <= 20.0:
                    perturbacao = 3.0
                torque_filtrado_atual += perturbacao"""

                """# rampa
                ciclo_tempo = tempo_decorrido % 100
                if ciclo_tempo < 50.0:
                    setpoint_atual = (24.0 / 50.0) * ciclo_tempo
                else:
                    setpoint_atual = 24.0 - (24.0 / 50.0) * (ciclo_tempo - 50.0)"""
                
                """# senoidal
                ciclo_tempo = tempo_decorrido % 100.0
                f1 = 1.0 / 100.0
                f2 = 5.0 / 100.0
                amp_secundaria = 3.0
                amp_fundamental = 12.0 - (amp_secundaria / 2.0)
                componente_fundamental = amp_fundamental - amp_fundamental * np.cos(2 * np.pi * f1 * ciclo_tempo)
                componente_rapida = amp_secundaria * np.sin(2 * np.pi * f2 * ciclo_tempo) * np.sin(np.pi * f1 * ciclo_tempo)
                setpoint_atual = componente_fundamental + componente_rapida"""

                # Calculo das predicoes e sinal de controle
                t1 = time.perf_counter()
                rpm_calculada = controlador_dmc.calcular_u(setpoint_atual, torque_filtrado_atual, rpm_anterior)
                latencias_otimizador.append((time.perf_counter() - t1) * 1000.0)

                # Aplicacao da variavel u(k)
                t2 = time.perf_counter()
                inversor.SendReferenceAngularVelocity(rpm_calculada)
                latencias_inversor.append((time.perf_counter() - t2) * 1000.0)
                
                latencias_ciclo_total.append((time.perf_counter() - t_inicio_ciclo) * 1000.0)
                
                torque_filtrado_anterior = torque_filtrado_atual
                rpm_anterior = rpm_calculada
                tempo_ultimo_ciclo = tempo_atual
                contador_iteracao += 1

                historico_tempo.append(tempo_decorrido)
                historico_sp.append(setpoint_atual)
                historico_pv.append(torque_filtrado_atual)
                historico_mv.append(rpm_calculada)

    except KeyboardInterrupt:
        pass
    
    finally:
        inversor.SendReferenceAngularVelocity(0)
        time.sleep(0.5)
        inversor.StopMotor()
        
        est_tq    = calcular_estatisticas(latencias_torquimetro)
        est_opt   = calcular_estatisticas(latencias_otimizador)
        est_inv   = calcular_estatisticas(latencias_inversor)
        est_total = calcular_estatisticas(latencias_ciclo_total)

        fig, axs = plt.subplots(2, 2, figsize=(14, 9))

        axs[0, 0].hist(latencias_torquimetro, bins=30, color='royalblue', edgecolor='black', alpha=0.7)
        axs[0, 0].axvline(est_tq['media'], color='red', linestyle='--', label=f"Média: {est_tq['media']:.1f}ms")
        axs[0, 0].axvline(est_tq['p99'], color='purple', linestyle=':', label=f"P99: {est_tq['p99']:.1f}ms")
        axs[0, 0].set_title("Latência de Comunicação: Torquímetro")
        axs[0, 0].set_xlabel("Tempo (ms)")
        axs[0, 0].grid(True, alpha=0.4)
        axs[0, 0].legend()

        axs[0, 1].hist(latencias_otimizador, bins=30, color='seagreen', edgecolor='black', alpha=0.7)
        axs[0, 1].axvline(est_opt['media'], color='red', linestyle='--', label=f"Média: {est_opt['media']:.1f}ms")
        axs[0, 1].axvline(est_opt['p99'], color='purple', linestyle=':', label=f"P99: {est_opt['p99']:.1f}ms")
        axs[0, 1].set_title("Tempo de Cálculo: Otimizador DMC")
        axs[0, 1].set_xlabel("Tempo (ms)")
        axs[0, 1].grid(True, alpha=0.4)
        axs[0, 1].legend()

        axs[1, 0].hist(latencias_inversor, bins=30, color='darkorange', edgecolor='black', alpha=0.7)
        axs[1, 0].axvline(est_inv['media'], color='red', linestyle='--', label=f"Média: {est_inv['media']:.1f}ms")
        axs[1, 0].axvline(est_inv['p99'], color='purple', linestyle=':', label=f"P99: {est_inv['p99']:.1f}ms")
        axs[1, 0].set_title("Latência de Comunicação: Inversor CFW11")
        axs[1, 0].set_xlabel("Tempo (ms)")
        axs[1, 0].grid(True, alpha=0.4)
        axs[1, 0].legend()

        axs[1, 1].hist(latencias_ciclo_total, bins=30, color='crimson', edgecolor='black', alpha=0.7)
        axs[1, 1].axvline(est_total['media'], color='black', linestyle='--', label=f"Média: {est_total['media']:.1f}ms")
        axs[1, 1].axvline(TEMPO_AMOSTRAGEM_MS, color='red', linestyle='-', linewidth=2, label=f"Limite Amostragem ({TEMPO_AMOSTRAGEM_MS:.1f}ms)")
        axs[1, 1].set_title("Tempo Total de Processamento por Ciclo")
        axs[1, 1].set_xlabel("Tempo (ms)")
        axs[1, 1].grid(True, alpha=0.4)
        axs[1, 1].legend()

        plt.tight_layout()
        plt.savefig('analise_latencia_bancada.pdf', format='pdf', bbox_inches='tight')
        
        plt.figure(figsize=(11, 7))
        
        plt.subplot(2, 1, 1)
        plt.plot(historico_tempo, historico_sp, color='crimson', linestyle='--', 
                 linewidth=2, label='Referencia de Torque (Setpoint)')
        plt.plot(historico_tempo, historico_pv, color='royalblue', linestyle='-', 
                 linewidth=1.5, label='Torque Real Medido (PV)')
        
        plt.title('Ensaio Dinamico do Controlador DMC Preditivo', fontsize=12, fontweight='bold', pad=12)
        plt.ylabel('Torque de Reacao [N.m]', fontsize=10, fontweight='bold')
        plt.legend(loc='upper right', frameon=True, shadow=True)
        plt.grid(True, linestyle=':', alpha=0.6)
        
        plt.subplot(2, 1, 2)
        plt.plot(historico_tempo, historico_mv, color='forestgreen', 
                 linestyle='-', linewidth=1.5, drawstyle='steps-post',
                 label='Acao de Controle DMC')
        
        plt.ylabel('Velocidade Angular [RPM]', fontsize=10, fontweight='bold')
        plt.xlabel('Tempo de Ensaio [Segundos]', fontsize=10, fontweight='bold')
        plt.legend(loc='upper right', frameon=True, shadow=True)
        plt.grid(True, linestyle=':', alpha=0.6)
        
        plt.tight_layout()
        nome_arquivo = f'desempenho_controlador_dmc_{PESO_LAMBDA}.pdf'
        plt.savefig(nome_arquivo, format='pdf', bbox_inches='tight')
        plt.show()

main()