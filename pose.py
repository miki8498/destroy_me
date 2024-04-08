import numpy as np 
import math 

def main():

   
    # Coordinate del centro fisso della circonferenza
    x_centro = -0.05

    # Raggio iniziale e finale
    raggio_iniziale = 0.15
    raggio_finale = 0.21

    # Passo di decremento del raggio
    passo_raggio = 0.01
    passo_theta = 0.314
    passo_zeta = -0.01


    # Calcolo delle coordinate x e z per ogni circonferenza
    raggio_corrente = raggio_iniziale
    theta_corrente = 1.57
    z_centro_corrente = 0.05
    while raggio_corrente <= raggio_finale:
        # Calcolo del centro della circonferenza corrente
        z_centro     = z_centro_corrente
        print(f"z_centro = {z_centro:.2f}")
        theta_centro =  theta_corrente  
        print(f"raggio_corrente = {raggio_corrente:.2f}")
 
        # Calcolo delle coordinate x e z per ciascun angolo
        
        x = x_centro + raggio_corrente * np.cos(theta_centro)
        z = z_centro + raggio_corrente * np.sin(theta_centro)
        print(f"Coordinate (x, z) = ({x:.2f}, {z:.2f})")
        
        # Incremento del raggio
        raggio_corrente += passo_raggio
        theta_corrente += passo_theta
        z_centro_corrente += passo_zeta
    

if __name__ == '__main__':
    main()