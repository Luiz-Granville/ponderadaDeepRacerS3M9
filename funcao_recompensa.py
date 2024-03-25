import math
import numpy as np

def reward_function(params):
 
    
    # Parâmetros para Incentivo de Velocidade
    PASSO_FUTURO = 6
    LIMIAR_CURVA_VELOCIDADE = 6    # graus
    LIMIAR_VELOCIDADE_LENTA = 1.8  # m/s
    LIMIAR_VELOCIDADE_RAPIDA = 2    # m/s

    # Parâmetros para Incentivo de Retidão
    PASSO_FUTURO_RETA = 8
    LIMIAR_CURVA_RETA = 25    # graus
    LIMIAR_ESTERÇAMENTO = 11         # graus

    # Parâmetros para Incentivo de Progresso
    NUM_TOTAL_PASSOS = 675 # (15 passos por segundo, portanto < 45 segundos)


    def identificar_curva(waypoints, closest_waypoints, future_step):

        # Identificar próximo waypoint e um waypoint mais distante
        ponto_anterior = waypoints[closest_waypoints[0]]
        ponto_proximo = waypoints[closest_waypoints[1]]
        ponto_futuro = waypoints[min(len(waypoints) - 1,
                                     closest_waypoints[1] + future_step)]

        # Calcular direções para waypoints
        direcao_atual = math.degrees(math.atan2(ponto_anterior[1]-ponto_proximo[1], 
                                               ponto_anterior[0] - ponto_proximo[0]))
        direcao_futura = math.degrees(math.atan2(ponto_anterior[1] - ponto_futuro[1], 
                                               ponto_anterior[0] - ponto_futuro[0]))

        # Calcular a diferença entre as direções
        diff_direcao = abs(direcao_atual - direcao_futura)

        # Verificar se não escolhemos o ângulo reflexo
        if diff_direcao > 180:
            diff_direcao = 360 - diff_direcao

        # Calcular distância até waypoint mais distante
        dist_futuro = np.linalg.norm([ponto_proximo[0] - ponto_futuro[0],
                                      ponto_proximo[1] - ponto_futuro[1]])  

        return diff_direcao, dist_futuro


    def selecionar_velocidade(waypoints, closest_waypoints, future_step):

        # Identificar se há uma curva no futuro
        diff_direcao, dist_futuro = identificar_curva(waypoints,
                                                    closest_waypoints,
                                                    future_step)

        if diff_direcao < LIMIAR_CURVA_VELOCIDADE:
            # Se não houver curva, encorajar a acelerar
            ir_rapido = True
        else:
            # Se houver curva, encorajar a desacelerar
            ir_rapido = False

        return ir_rapido


    def selecionar_reta(waypoints, closest_waypoints, future_step):

        # Identificar se há uma curva no futuro
        diff_direcao, dist_futuro = identificar_curva(waypoints,
                                                    closest_waypoints,
                                                    future_step)

        if diff_direcao < LIMIAR_CURVA_RETA:
            # Se não houver curva, encorajar a ir mais reto
            ir_reto = True
        else:
            # Se houver curva, não encorajar a ir mais reto
            ir_reto = False

        return ir_reto


    # Ler parâmetros de entrada
    todas_rodas_na_pista = params['all_wheels_on_track']
    closest_waypoints = params['closest_waypoints']
    distancia_da_borda = params['distance_from_center']
    fora_da_pista = params['is_offtrack']
    progresso = params['progress']
    velocidade = params['speed']
    angulo_de_esterçamento = params['steering_angle']
    passos = params['steps']
    largura_da_pista = params['track_width']
    waypoints = params['waypoints']

    # Desencorajar fortemente sair da pista
    if fora_da_pista:
        recompensa = 1e-3
        return float(recompensa)
    
    # Dar recompensa maior se o carro estiver mais próximo da linha central e vice-versa
    # 0 se estiver na borda da pista, 1 se estiver no centro da pista
    recompensa = 1 - (distancia_da_borda/(largura_da_pista/2))**(1/4) 

    # A cada 50 passos, se estiver à frente da posição esperada, dar recompensa relativa
    # à distância que está à frente
    if (passos % 50) == 0 and progresso/100 > (passos/NUM_TOTAL_PASSOS):
        # recompensa += 2.22 para cada segundo mais rápido que 45s projetados
        recompensa += progresso - (passos/NUM_TOTAL_PASSOS)*100

    # Implementar incentivo de retidão
    permanecer_reto = selecionar_reta(waypoints, closest_waypoints, 
                                    PASSO_FUTURO_RETA)
    if permanecer_reto and abs(angulo_de_esterçamento) < LIMIAR_ESTERÇAMENTO:
        recompensa += 0.3

    # Implementar incentivo de velocidade
    ir_rapido = selecionar_velocidade(waypoints, closest_waypoints, PASSO_FUTURO)

    if ir_rapido and velocidade > LIMIAR_VELOCIDADE_RAPIDA and abs(angulo_de_esterçamento) < LIMIAR_ESTERÇAMENTO:
        recompensa += 2.0

    elif not ir_rapido and velocidade < LIMIAR_VELOCIDADE_LENTA:
        recompensa += 0.5    

    # Implementar incentivo para permanecer na pista
    if not todas_rodas_na_pista:
        recompensa -= 0.5

    recompensa = max(recompensa, 1e-3)
    return float(recompensa)
