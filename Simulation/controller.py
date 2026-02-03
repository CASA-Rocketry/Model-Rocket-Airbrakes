from config import Config
from controllers.controller_pid import ControllerPID
from controllers.controller_bangbang import ControllerBangBang
from controllers.controller_optimizer import ControllerOptimizer
from controllers.controller_optimizer_pid import ControllerOptimizerPID
from controllers.controller_file import ControllerFile


def Control(config: Config, deployment_file: str = None):
    """
    Create controller instance based on config

    Args:
        config: Config object
        deployment_file: Optional path to CSV file with deployment data.
                        If provided, uses ControllerFile to replay deployments.

    Returns:
        Controller instance
    """
    # If deployment file specified, use file-based controller
    if deployment_file is not None:
        return ControllerFile(
            config,
            deployment_file,
            time_col=getattr(config, 'deployment_file_time_col', 'time'),
            deployment_col=getattr(config, 'deployment_file_deployment_col', 'deployment'),
            time_unit=getattr(config, 'deployment_file_time_unit', 's')
        )

    # Check if FILE algorithm selected in config
    if hasattr(config, 'control_algorithm') and config.control_algorithm.upper() == "FILE":
        # Use file path from config
        return ControllerFile(
            config,
            config.deployment_file_path,
            time_col=config.deployment_file_time_col,
            deployment_col=config.deployment_file_deployment_col,
            time_unit=config.deployment_file_time_unit
        )

    # Otherwise use algorithm from config
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
            f"Available options: 'PID', 'BANGBANG', 'OPTIMIZER', 'OPTIMIZERPID', 'FILE'"
        )