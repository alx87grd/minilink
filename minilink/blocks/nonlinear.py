"""Static nonlinearity blocks (placeholder — shelf decided, content planned).

Memoryless input-output maps used for actuator and sensor realism:

Planned blocks: ``Saturation``, ``DeadZone``, ``Relay``, ``RateLimiter``,
``Hysteresis`` (the last two carry internal state despite the shelf name —
they stay here because the *role* is "nonlinearity in the signal path").

All are :class:`~minilink.core.system.StaticSystem` subclasses (or small
``DynamicSystem`` ones) with the standard ``u``/``y`` ports and JAX-traceable
equations where the math allows (``xp.clip``, ``xp.where``).
"""
