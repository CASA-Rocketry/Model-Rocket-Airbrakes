from config import Config
from controllers.controller_pid import ControllerPID
from controllers.controller_bangbang import ControllerBangBang
from controllers.controller_optimizer import ControllerOptimizer
from controllers.controller_optimizer_pid import ControllerOptimizerPID


def Control(config: Config):
    algorithm = config.control_algorithm.upper()

    if algorithm == "PID":
        return ControllerPID(config)
    elif algorithm == "BANGBANG":
        return ControllerBangBang(config)
    elif algorithm == "OPTIMIZER":
        return ControllerOptimizer(config)
    elif algorithm == "OPTIMIZERPID":
        return ControllerOptimizerPID(config)
    else:
        raise ValueError(
            f"Unknown control algorithm: '{config.control_algorithm}'. "
            f"Available options: 'PID', 'BANGBANG', 'OPTIMIZER'"
        )