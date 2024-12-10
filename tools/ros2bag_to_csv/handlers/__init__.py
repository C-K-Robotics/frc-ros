import importlib
import pkgutil

MSG_HANDLER_MAP = {}
MSG_NAME_HANDLER_MAP = {}


def __import_submodules(package, recursive=True):
    """Import all submodules of a module, recursively, including subpackages

    :param package: package (name or actual module)
    :type package: str | module
    :rtype: dict[str, types.ModuleType]
    """
    if isinstance(package, str):
        package = importlib.import_module(package)
    results = {}
    for loader, name, is_pkg in pkgutil.walk_packages(package.__path__):
        full_name = package.__name__ + "." + name
        results[full_name] = importlib.import_module(full_name)
        can_handle = results[full_name].get_what_i_can_handle()
        can_handle_type = results[full_name].get_handle_able_type()
        print(f'Found custom message handler for "{can_handle}" in "{full_name}".')
        MSG_HANDLER_MAP[can_handle_type] = results[full_name].handle
        MSG_NAME_HANDLER_MAP[can_handle] = results[full_name].handle
        if recursive and is_pkg:
            results.update(__import_submodules(full_name))
    return results


__import_submodules(__name__)


def register_handler(can_handle: str, can_handle_type: type, handler):
    MSG_HANDLER_MAP[can_handle_type] = handler
    print(f"Registered custom message handler for {can_handle}")
