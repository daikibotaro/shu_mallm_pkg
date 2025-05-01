from .chainlit_runner import ChainlitRunner


def create_function_wrapper(cls, method_name):
    def wrapper(*args, **kwargs):
        instance = cls(*args, **kwargs)
        method = getattr(instance, method_name)
        return method()
    return wrapper


run_chainlit = create_function_wrapper(ChainlitRunner, "run_chainlit")
