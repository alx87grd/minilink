import jax
import jax.numpy as jnp
import numpy as np

from minilink.core.framework import DynamicSystem
from minilink.core.jax_utils import get_f_jax

class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, m=1, p=2)
        self.name = "Pendulum"

    def f_jax(self, x, u, t=0, params=None):
        gravity = 9.81
        length = 1.0
        damping = 0.5
        q, dq = x[0], x[1]
        
        ddq = - (gravity / length) * jnp.sin(q) - damping * dq + u[0]
        
        return jnp.array([dq, ddq])


if __name__ == "__main__":
    sys = Pendulum()
    
    # 1. On récupère la fonction f compatible XLA
    f_jax_func = get_f_jax(sys)
    
    # Points de tests : Équilibre du pendule (q=0, dq=0)
    x0 = jnp.array([0.0, 0.0])
    u0 = jnp.array([0.0])
    
    print("====================================")
    print("  ÉVALUATION NORMALE : dx = f(x,u)")
    print("====================================")
    dx = f_jax_func(x0, u0, 0)
    print(f"Dérivée d'état (dx) au repos : {dx}")
    
    
    print("\n=======================================================")
    print("   AUTO-DIFF : Jacobiennes exactes (Matrices A et B)")
    print("=======================================================")
    
    # 2. On utilise jax.jacfwd 
    # argnums=0 dérive par rapport au premier argument f(x, ...) -> df/dx = A
    # argnums=1 dérive par rapport au deuxième argument f(x, u, ...) -> df/du = B
    
    df_dx_func = jax.jit(jax.jacfwd(f_jax_func, argnums=0))
    df_du_func = jax.jit(jax.jacfwd(f_jax_func, argnums=1))
    
    # Mesure du temps pour le fun (après le warm-up)
    _ = df_dx_func(x0, u0, 0) # compile
    
    # Matrice A de dimension (n, n)
    A = df_dx_func(x0, u0, 0)
    print("\nMatrice A = df/dx (Analytique parfaite générée par JAX) :")
    print(A)
    # Expected analytical:
    # [ 0                      ,  1       ]
    # [ -(g/l)*cos(q) = -9.81  , -damping = -0.5 ]
    
    # Matrice B de dimension (n, m)
    B = df_du_func(x0, u0, 0)
    print("\nMatrice B = df/du :")
    print(B)
    # Expected analytical:
    # [ 0 ]
    # [ 1 ]
