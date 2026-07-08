import time
import numpy as np
import matplotlib.pyplot as plt

from dmc import carregar_modelos_normalizados, DMC_Chaveado_Restrito

NUM_ITERACOES = 2000
HORIZONTE_CONTROLE = 3
HORIZONTE_PREDICAO = 15
PESO_LAMBDA = 0.005

tempo_amostragem_ms = 50.0 

SP_MIN, SP_MAX = 5.0, 15.0    
PV_MIN, PV_MAX = 5.0, 15.0    
MV_MIN, MV_MAX = 300.0, 1000.0  

def calcular_estatisticas(tempos_ms):
    tempos = np.array(tempos_ms)
    return {
        'min': np.min(tempos),
        'max': np.max(tempos),
        'media': np.mean(tempos),
        'mediana': np.median(tempos),
        'std': np.std(tempos),
        'p95': np.percentile(tempos, 95),
        'p99': np.percentile(tempos, 99)
    }

def main():
    print("Carregando banco de modelos do DMC...")
    try:
        banco_modelos, total_pontos = carregar_modelos_normalizados(
            'modelo_medio_DMC.csv', 
            'modelo_medio_DMC_decrescente.csv'
        )
        if banco_modelos is None:
            print("Erro ao inicializar os modelos.")
            return
    except Exception as e:
        print(f"Erro ao carregar CSVs: {e}")
        print("Certifique-se de estar na raiz do projeto onde os CSVs estao localizados.")
        return

    controlador_dmc = DMC_Chaveado_Restrito(
        banco_modelos, 
        HORIZONTE_PREDICAO, 
        HORIZONTE_CONTROLE, 
        PESO_LAMBDA
    )

    print(f"Iniciando simulacao Monte Carlo com {NUM_ITERACOES} calculos do DMC...")
    print("Isso pode levar alguns segundos...")
    
    tempos_execucao_ms = []

    for i in range(NUM_ITERACOES):
        setpoint_aleatorio = np.random.uniform(SP_MIN, SP_MAX)
        torque_atual_aleatorio = np.random.uniform(PV_MIN, PV_MAX)
        rpm_anterior_aleatorio = np.random.uniform(MV_MIN, MV_MAX)

        inicio = time.perf_counter()
        
        u_calculado = controlador_dmc.calcular_u(
            setpoint_aleatorio, 
            torque_atual_aleatorio, 
            rpm_anterior_aleatorio
        )
        
        fim = time.perf_counter()

        tempo_ms = (fim - inicio) * 1000.0
        tempos_execucao_ms.append(tempo_ms)

        if (i + 1) % 500 == 0:
            print(f"Progresso: {i + 1}/{NUM_ITERACOES} concluidos...")

    est = calcular_estatisticas(tempos_execucao_ms)
    
    print("\n" + "="*50)
    print(" RESULTADOS DO BENCHMARK (TEMPO DE CALCULO DMC)")
    print("="*50)
    print(f" Minimo:         {est['min']:.3f} ms")
    print(f" Media:          {est['media']:.3f} ms")
    print(f" Mediana:        {est['mediana']:.3f} ms")
    print(f" Percentil 95:   {est['p95']:.3f} ms")
    print(f" Percentil 99:   {est['p99']:.3f} ms")
    print(f" MAXIMO (WCET):  {est['max']:.3f} ms")
    print("="*50)

    margem_seguranca = tempo_amostragem_ms - est['max']
    
    if est['max'] > tempo_amostragem_ms:
        print(f"\n[ALERTA CRITICO] O tempo maximo ({est['max']:.1f}ms) superou o Ts de {tempo_amostragem_ms}ms!")
        print("Isso causara o colapso do tempo real (Jitter severo).")
    elif margem_seguranca < 20.0:
        print(f"\n[AVISO] O tempo maximo ({est['max']:.1f}ms) esta perigosamente perto do Ts de {tempo_amostragem_ms}ms.")
        print(f"Sobra pouco tempo ({margem_seguranca:.1f}ms) para ler a Serial e enviar os dados.")
    else:
        print(f"\n[OK] O solver esta rapido o suficiente. Folga de pior caso: {margem_seguranca:.1f}ms.")

    plt.figure(figsize=(10, 6))
    plt.hist(tempos_execucao_ms, bins=50, color='darkorange', alpha=0.7, edgecolor='black')
    
    plt.axvline(est['media'], color='blue', linestyle='dashed', linewidth=2, label=f"Media: {est['media']:.2f} ms")
    plt.axvline(est['p99'], color='purple', linestyle='dotted', linewidth=2, label=f"P99: {est['p99']:.2f} ms")
    plt.axvline(est['max'], color='red', linestyle='solid', linewidth=2, label=f"Max (WCET): {est['max']:.2f} ms")
    plt.axvline(tempo_amostragem_ms, color='black', linestyle='-.', linewidth=2, label=f"Ts ({tempo_amostragem_ms} ms)")
    
    plt.title("Distribuicao do Tempo de Calculo do DMC Restrito (Monte Carlo)")
    plt.xlabel("Tempo de Execucao [ms]")
    plt.ylabel("Frequencia")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()
    plt.show()

main()