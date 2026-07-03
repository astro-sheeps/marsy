import os


def load_rover():
    mode = os.getenv("MARSY_MODE", "sim")

    if mode == "sim":
        from marsy_backends.sim_backend import rover
        print("Marsy backend: SIMULATOR")
        return rover

    if mode == "real":
        from marsy_backends.real_backend import rover
        print("Marsy backend: REAL ROVER")
        return rover

    raise ValueError(f"Unknown MARSY_MODE: {mode}")