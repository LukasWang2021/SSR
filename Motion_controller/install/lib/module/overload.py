# for function overload
from innertypes import *

registry = {}

class OverloadMethod(object):
    def __init__(self, name):
        self.name = name
        self.typemap = {}

    def __call__(self, *args):
        types = tuple(arg.__class__ for arg in args) # a generator expression!
        function = self.typemap.get(types)
        if function is None:
            raise TypeError("no match function call")
        return function(*args)

    def register(self, types, function):
        if types in self.typemap:
            raise TypeError("duplicate registration")
        self.typemap[types] = function

def overload(*types):
    def register(function):
        name = function.__name__
        om = registry.get(name)
        if om is None:
            om = registry[name] = OverloadMethod(name)
        om.register(types, function)
        return om
    return register