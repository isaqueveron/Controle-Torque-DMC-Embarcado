# Script que automatiza os testes de resposta ao degrau

import time
import csv
import numpy as np
import matplotlib.pyplot as plt

from driver_torquimetro import Torquimeter
from driver_inversor_cfw11 import Inverter
from init_serial_devices import selecionar_porta

PASSO_HARDWARE_SEGUNDOS = 0.01  
TEMPO_AMOSTRAGEM_DMC = 0.05
TEMPO_ESTABILIZACAO_S = 5
TEMPO_POS_DEGRAU_S = 3
TEMPO_TOTAL_ENSAIO = TEMPO_ESTABILIZACAO_S + TEMPO_POS_DEGRAU_S
NUM_ENSAIOS_POR_FAIXA = 3
FREQ_CORTE_HZ = 25.0 

omega_c = 2 * np.pi * FREQ_CORTE_HZ
tau = 1.0 / omega_c
ALPHA = PASSO_HARDWARE_SEGUNDOS / (tau + PASSO_HARDWARE_SEGUNDOS)

TEMPO_PRE_DEGRAU_VISUALIZAR = 1.0 # desloco o eixo t para vizualizar melhor 

def main():
    FAIXAS_DEGRAU = [
                    (300, 400),
                    (500, 600),
                    (600, 700)
                ]
    print("--- Inicialização do Setup de Identificação para DMC ---")
    print(f"Configuração ativa: Ts = {TEMPO_AMOSTRAGEM_DMC}s | Alpha do Filtro = {ALPHA:.4f}")

    inversor.ActivateMotor()
    time.sleep(0.1)
    inversor.SendReferenceAngularVelocity(0)
    time.sleep(1.0)
    
    resultados_medios = {}

    try:
        for rpm_inicial, rpm_final in FAIXAS_DEGRAU:
            faixa = f'{rpm_inicial}->{rpm_final}'
            print(f"\n{'='*40}")
            print(f"Iniciando MÚLTIPLOS ENSAIOS: Degrau de {rpm_inicial} para {rpm_final} RPM")
            
            matriz_torques_brutos_transitorios = []     
            matriz_torques_filtrados_transitorios = []
            matriz_tempos_transitorios = []
            matriz_ref_transitorios = []

            for n_ensaio in range(NUM_ENSAIOS_POR_FAIXA):
                print(f"\n -> Rodada {n_ensaio + 1} de {NUM_ENSAIOS_POR_FAIXA}")
                
                inversor.SendReferenceAngularVelocity(rpm_inicial)
                print(f"    Aguardando estabilização inicial ({TEMPO_ESTABILIZACAO_S} segundos)...")
                time.sleep(TEMPO_ESTABILIZACAO_S) 
                
                num_pontos = int(TEMPO_TOTAL_ENSAIO / PASSO_HARDWARE_SEGUNDOS) + 1
                tempos = np.zeros(num_pontos)
                torques_brutos_rt = np.zeros(num_pontos)     
                torques_filtrados_rt = np.zeros(num_pontos)
                referencias_aplicadas = np.zeros(num_pontos)
                
                torque_anterior_filtrado = 0.0
                
                idx = 0
                idx_degrau = 0 
                tempo_inicio_ensaio = time.perf_counter()
                tempo_ultimo_controle = tempo_inicio_ensaio
                degrau_aplicado = False
                
                while idx < num_pontos:
                    tempo_atual = time.perf_counter()
                    tempo_decorrido = tempo_atual - tempo_inicio_ensaio
                    
                    if (tempo_atual - tempo_ultimo_controle) >= PASSO_HARDWARE_SEGUNDOS:
                        if tempo_decorrido >= TEMPO_ESTABILIZACAO_S and not degrau_aplicado:
                            inversor.SendReferenceAngularVelocity(rpm_final)
                            degrau_aplicado = True
                            idx_degrau = idx 
                            print(f"    [{tempo_decorrido:.2f}s] Degrau aplicado!")
                        
                        torquimetro.ReadRaw()
                        torque_lido = torquimetro.Torque_calibrated
                        
                        if idx == 0:
                            torque_f = torque_lido
                        else:
                            torque_f = (ALPHA * torque_lido) + ((1.0 - ALPHA) * torque_anterior_filtrado)
                        
                        torque_anterior_filtrado = torque_f
                        
                        tempos[idx] = tempo_decorrido
                        torques_brutos_rt[idx] = torque_lido      
                        torques_filtrados_rt[idx] = torque_f
                        
                        referencias_aplicadas[idx] = rpm_final if degrau_aplicado else rpm_inicial
                        
                        idx += 1
                        tempo_ultimo_controle = tempo_atual

                pontos_retroceder = int(TEMPO_PRE_DEGRAU_VISUALIZAR / PASSO_HARDWARE_SEGUNDOS)
                idx_inicio_recorte = max(0, idx_degrau - pontos_retroceder)

                t_trans = tempos[idx_inicio_recorte:idx] - tempos[idx_degrau]
                ref_trans = referencias_aplicadas[idx_inicio_recorte:idx]
                tq_b_trans = torques_brutos_rt[idx_inicio_recorte:idx]    
                tq_f_trans = torques_filtrados_rt[idx_inicio_recorte:idx]

                matriz_tempos_transitorios.append(t_trans)
                matriz_ref_transitorios.append(ref_trans)
                matriz_torques_brutos_transitorios.append(tq_b_trans)
                matriz_torques_filtrados_transitorios.append(tq_f_trans)
                
                print("    Ensaio concluído. Retornando a RPM inicial.")
                inversor.SendReferenceAngularVelocity(rpm_inicial)
                time.sleep(2.0) 
            
            min_len = min(len(t) for t in matriz_tempos_transitorios)
            
            matriz_tq_b_truncada = np.array([tq[:min_len] for tq in matriz_torques_brutos_transitorios])
            tq_medio_bruto = np.mean(matriz_tq_b_truncada, axis=0) 
            
            matriz_tq_f_truncada = np.array([tq[:min_len] for tq in matriz_torques_filtrados_transitorios])
            tq_medio_filtrado = np.mean(matriz_tq_f_truncada, axis=0)
            
            t_medio = matriz_tempos_transitorios[0][:min_len]
            ref_media = matriz_ref_transitorios[0][:min_len]

            resultados_medios[faixa] = {
                't_transitorio': t_medio,
                'ref_transitorio': ref_media,
                'tq_bruto_medio': tq_medio_bruto,
                'tq_filt_medio': tq_medio_filtrado
            }

    finally:
        print("\n=== Desligando sistema para segurança ===")
        inversor.SendReferenceAngularVelocity(0)
        time.sleep(0.1)
        inversor.StopMotor()
        
        if not resultados_medios:
            return

        print("-" * 50)
        ts_valido = False
        while not ts_valido:
            try:
                Ts = TEMPO_AMOSTRAGEM_DMC
                if Ts <= 0:
                    raise ValueError
                ts_valido = True
            except ValueError:
                print("Valor inválido. Digite um número positivo.")

        print(f"\nSalvando curvas médias contínuas em 'modelo_medio_continuo.csv'...")
        with open('modelo_medio_continuo.csv', mode='w', newline='') as f_cont:
            writer = csv.writer(f_cont)
            writer.writerow(['Faixa_RPM', 'Tempo_s', 'Ref_RPM', 'Torque_Bruto_Medio_Nm', 'Torque_Filt_Medio_Nm'])
            for faixa, dados in resultados_medios.items():
                for t, r, tq_b, tq_f in zip(dados['t_transitorio'], dados['ref_transitorio'], dados['tq_bruto_medio'], dados['tq_filt_medio']):
                    writer.writerow([faixa, f"{t:.4f}", f"{r:.2f}", f"{tq_b:.4f}", f"{tq_f:.4f}"])

        print(f"Salvando Matriz DMC (curvas médias discretizadas com Ts={Ts}s) em 'modelo_medio_DMC.csv'...")
        with open('modelo_medio_DMC.csv', mode='w', newline='') as f_disc:
            writer_disc = csv.writer(f_disc)
            writer_disc.writerow(['Faixa_RPM', 'Tempo_Discreto_s', 'Torque_Filt_Medio_Disc_Nm'])
            
            for faixa, dados in resultados_medios.items():
                t_orig = dados['t_transitorio']
                t_final = t_orig[-1]
                
                t_disc = np.arange(0, t_final, Ts)
                tq_disc = np.interp(t_disc, t_orig, dados['tq_filt_medio'])
                
                dados['t_disc'] = t_disc
                dados['tq_disc'] = tq_disc
                
                for t_d, tq_d in zip(t_disc, tq_disc):
                    writer_disc.writerow([faixa, f"{t_d:.4f}", f"{tq_d:.4f}"])

        print("\nGerando gráficos...")
        fig, axs = plt.subplots(len(resultados_medios), 1, figsize=(10, 3.5 * len(resultados_medios)), squeeze=False)
        fig.canvas.manager.set_window_title('Modelos Médios Filtrados para DMC')
        
        for i, (faixa, dados) in enumerate(resultados_medios.items()):
            ax = axs[i, 0]
            ax.plot(dados['t_transitorio'], dados['tq_bruto_medio'], color='lightgray', alpha=0.7, linewidth=1.0, label='Média Sinais Brutos')
            ax.plot(dados['t_transitorio'], dados['tq_filt_medio'], color='blue', linewidth=1.5, label='Curva Média (Passa-Baixas Manual)')
            ax.plot(dados['t_disc'], dados['tq_disc'], marker='o', color='red', linestyle='None', markersize=4, label=f'DMC Model (Ts={Ts}s)')
            ax.axvline(x=0, color='black', linestyle=':', linewidth=1.5, label='Instante do Degrau')
            ax_ref = ax.twinx()
            ax_ref.plot(dados['t_transitorio'], dados['ref_transitorio'], color='green', linestyle='--', alpha=0.6, label='Ref RPM (Degrau)')
            ax_ref.set_ylabel('Velocidade [RPM]', color='green')
            ax_ref.tick_params(axis='y', labelcolor='green')
            ax.set_title(f"Modelo DMC: {faixa} RPM ({NUM_ENSAIOS_POR_FAIXA} Ensaios)")
            ax.set_ylabel('Torque [N.m]')
            ax.grid(True, linestyle='--')
            ax.set_xlim(left=-TEMPO_PRE_DEGRAU_VISUALIZAR)

            lines_1, labels_1 = ax.get_legend_handles_labels()
            lines_2, labels_2 = ax_ref.get_legend_handles_labels()
            ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc='lower right')

        axs[-1, 0].set_xlabel('Tempo em relação ao Degrau [s]')
        plt.tight_layout()
        plt.savefig('modelos_medios_dmc.png', dpi=300)
        plt.show()

def main_decrescente():
    FAIXAS_DEGRAU = [
                    (400, 300),
                    (600, 500),
                    (700, 600)
                ]
    print("--- Inicialização do Setup de Identificação para DMC (Degrau Decrescente) ---")
    print(f"Configuração ativa: Ts = {TEMPO_AMOSTRAGEM_DMC}s | Alpha do Filtro = {ALPHA:.4f}")

    inversor.ActivateMotor()
    time.sleep(0.1)
    inversor.SendReferenceAngularVelocity(0)
    time.sleep(1.0)
    
    resultados_medios = {}

    try:
        for rpm_inicial, rpm_final in FAIXAS_DEGRAU:
            faixa = f'{rpm_inicial}->{rpm_final}'
            print(f"\n{'='*40}")
            print(f"Iniciando MÚLTIPLOS ENSAIOS: Degrau decrescente de {rpm_inicial} para {rpm_final} RPM")
            
            matriz_torques_brutos_transitorios = []     
            matriz_torques_filtrados_transitorios = []
            matriz_tempos_transitorios = []
            matriz_ref_transitorios = []

            for n_ensaio in range(NUM_ENSAIOS_POR_FAIXA):
                print(f"\n -> Rodada {n_ensaio + 1} de {NUM_ENSAIOS_POR_FAIXA}")
                print(f"    Levando motor até a rotação inicial de {rpm_inicial} RPM...")
                inversor.SendReferenceAngularVelocity(rpm_inicial)
                print(f"    Aguardando estabilização inicial em regime permanente ({TEMPO_ESTABILIZACAO_S} segundos)...")
                time.sleep(TEMPO_ESTABILIZACAO_S) 
                
                num_pontos = int(TEMPO_TOTAL_ENSAIO / PASSO_HARDWARE_SEGUNDOS) + 1
                tempos = np.zeros(num_pontos)
                torques_brutos_rt = np.zeros(num_pontos)     
                torques_filtrados_rt = np.zeros(num_pontos)
                referencias_aplicadas = np.zeros(num_pontos)
                
                torque_anterior_filtrado = 0.0
                
                idx = 0
                idx_degrau = 0 
                tempo_inicio_ensaio = time.perf_counter()
                tempo_ultimo_controle = tempo_inicio_ensaio
                degrau_aplicado = False
                
                while idx < num_pontos:
                    tempo_atual = time.perf_counter()
                    tempo_decorrido = tempo_atual - tempo_inicio_ensaio
                    
                    if (tempo_atual - tempo_ultimo_controle) >= PASSO_HARDWARE_SEGUNDOS:
                        # Aplica a redução (degrau para baixo) após o tempo de estabilização
                        if tempo_decorrido >= TEMPO_ESTABILIZACAO_S and not degrau_aplicado:
                            inversor.SendReferenceAngularVelocity(rpm_final)
                            degrau_aplicado = True
                            idx_degrau = idx 
                            print(f"    [{tempo_decorrido:.2f}s] Degrau decrescente aplicado!")
                        
                        torquimetro.ReadRaw()
                        torque_lido = torquimetro.Torque_calibrated
                        
                        if idx == 0:
                            torque_f = torque_lido
                        else:
                            torque_f = (ALPHA * torque_lido) + ((1.0 - ALPHA) * torque_anterior_filtrado)
                        
                        torque_anterior_filtrado = torque_f
                        
                        tempos[idx] = tempo_decorrido
                        torques_brutos_rt[idx] = torque_lido      
                        torques_filtrados_rt[idx] = torque_f
                        referencias_aplicadas[idx] = rpm_final if degrau_aplicado else rpm_inicial
                        
                        idx += 1
                        tempo_ultimo_controle = tempo_atual

                pontos_retroceder = int(TEMPO_PRE_DEGRAU_VISUALIZAR / PASSO_HARDWARE_SEGUNDOS)
                idx_inicio_recorte = max(0, idx_degrau - pontos_retroceder)
                t_trans = tempos[idx_inicio_recorte:idx] - tempos[idx_degrau]
                ref_trans = referencias_aplicadas[idx_inicio_recorte:idx]
                tq_b_trans = torques_brutos_rt[idx_inicio_recorte:idx]    
                tq_f_trans = torques_filtrados_rt[idx_inicio_recorte:idx]

                matriz_tempos_transitorios.append(t_trans)
                matriz_ref_transitorios.append(ref_trans)
                matriz_torques_brutos_transitorios.append(tq_b_trans)
                matriz_torques_filtrados_transitorios.append(tq_f_trans)
                
                print(f"    Ensaio concluído. Mantendo em {rpm_inicial} RPM para preparar próxima rodada.")
                inversor.SendReferenceAngularVelocity(rpm_inicial)
                time.sleep(2.0) 
            
            min_len = min(len(t) for t in matriz_tempos_transitorios)
            
            matriz_tq_b_truncada = np.array([tq[:min_len] for tq in matriz_torques_brutos_transitorios])
            tq_medio_bruto = np.mean(matriz_tq_b_truncada, axis=0) 
            
            matriz_tq_f_truncada = np.array([tq[:min_len] for tq in matriz_torques_filtrados_transitorios])
            tq_medio_filtrado = np.mean(matriz_tq_f_truncada, axis=0)
            
            t_medio = matriz_tempos_transitorios[0][:min_len]
            ref_media = matriz_ref_transitorios[0][:min_len]

            resultados_medios[faixa] = {
                't_transitorio': t_medio,
                'ref_transitorio': ref_media,
                'tq_bruto_medio': tq_medio_bruto,
                'tq_filt_medio': tq_medio_filtrado
            }

    finally:
        print("\n=== Desligando sistema para segurança ===")
        inversor.SendReferenceAngularVelocity(0)
        time.sleep(0.1)
        inversor.StopMotor()
        
        if not resultados_medios:
            return

        print("-" * 50)
        ts_valido = False
        while not ts_valido:
            try:
                Ts = TEMPO_AMOSTRAGEM_DMC
                if Ts <= 0:
                    raise ValueError
                ts_valido = True
            except ValueError:
                print("Valor inválido. Digite um número positivo.")

        print(f"\nSalvando curvas médias contínuas em 'modelo_medio_continuo.csv'...")
        with open('modelo_medio_continuo_decrescente.csv', mode='w', newline='') as f_cont:
            writer = csv.writer(f_cont)
            writer.writerow(['Faixa_RPM', 'Tempo_s', 'Ref_RPM', 'Torque_Bruto_Medio_Nm', 'Torque_Filt_Medio_Nm'])
            for faixa, dados in resultados_medios.items():
                for t, r, tq_b, tq_f in zip(dados['t_transitorio'], dados['ref_transitorio'], dados['tq_bruto_medio'], dados['tq_filt_medio']):
                    writer.writerow([faixa, f"{t:.4f}", f"{r:.2f}", f"{tq_b:.4f}", f"{tq_f:.4f}"])

        print(f"Salvando Matriz DMC (curvas médias discretizadas com Ts={Ts}s) em 'modelo_medio_DMC.csv'...")
        with open('modelo_medio_DMC_decrescente.csv', mode='w', newline='') as f_disc:
            writer_disc = csv.writer(f_disc)
            writer_disc.writerow(['Faixa_RPM', 'Tempo_Discreto_s', 'Torque_Filt_Medio_Disc_Nm'])
            
            for faixa, dados in resultados_medios.items():
                t_orig = dados['t_transitorio']
                t_final = t_orig[-1]
                t_disc = np.arange(0, t_final, Ts)
                tq_disc = np.interp(t_disc, t_orig, dados['tq_filt_medio'])
                
                dados['t_disc'] = t_disc
                dados['tq_disc'] = tq_disc
                
                for t_d, tq_d in zip(t_disc, tq_disc):
                    writer_disc.writerow([faixa, f"{t_d:.4f}", f"{tq_d:.4f}"])

        print("\nGerando gráficos...")
        fig, axs = plt.subplots(len(resultados_medios), 1, figsize=(10, 3.5 * len(resultados_medios)), squeeze=False)
        fig.canvas.manager.set_window_title('Modelos Médios Filtrados para DMC - Degrau Decrescente')
        
        for i, (faixa, dados) in enumerate(resultados_medios.items()):
            ax = axs[i, 0]
            
            ax.plot(dados['t_transitorio'], dados['tq_bruto_medio'], color='lightgray', alpha=0.7, linewidth=1.0, label='Média Sinais Brutos')
            ax.plot(dados['t_transitorio'], dados['tq_filt_medio'], color='blue', linewidth=1.5, label='Curva Média (Passa-Baixas Manual)')
            ax.plot(dados['t_disc'], dados['tq_disc'], marker='o', color='red', linestyle='None', markersize=4, label=f'DMC Model (Ts={Ts}s)')
            ax.axvline(x=0, color='black', linestyle=':', linewidth=1.5, label='Instante do Degrau')
            ax_ref = ax.twinx()
            ax_ref.plot(dados['t_transitorio'], dados['ref_transitorio'], color='green', linestyle='--', alpha=0.6, label='Ref RPM (Queda)')
            ax_ref.set_ylabel('Velocidade [RPM]', color='green')
            ax_ref.tick_params(axis='y', labelcolor='green')
            
            ax.set_title(f"Modelo DMC Decrescente: {faixa} RPM ({NUM_ENSAIOS_POR_FAIXA} Ensaios)")
            ax.set_ylabel('Torque [N.m]')
            ax.grid(True, linestyle='--')
            ax.set_xlim(left=-TEMPO_PRE_DEGRAU_VISUALIZAR)
            
            lines_1, labels_1 = ax.get_legend_handles_labels()
            lines_2, labels_2 = ax_ref.get_legend_handles_labels()
            ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc='lower right')

        axs[-1, 0].set_xlabel('Tempo em relação ao Degrau [s]')
        plt.tight_layout()
        plt.savefig('modelos_medios_dmc_decrescente.png', dpi=300)
        plt.show()

# ---------------------------------------------------------------

porta_torquimetro = selecionar_porta("Torquimetro")
porta_inversor = selecionar_porta("Inversor")
    
if not porta_torquimetro or not porta_inversor:
    print("Erro: Portas não selecionadas corretamente. Encerrando.")
        
else:
    torquimetro = Torquimeter(Port=porta_torquimetro, Baudrate=230400, Timeout=0.003) 
    inversor = Inverter(Port=porta_inversor, ADR=1, Baudrate=57600, Timeout=0.003)
    main()
    main_decrescente()