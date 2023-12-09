"""
some nice utility decorators i made to make my life 'easier'
"""
from typing import *
import inspect
import pprint
import rospy # type: ignore

T = TypeVar('T')
U = TypeVar('U')


Deconstruct = Tuple[tuple, dict]


def debugrepr(value) -> str:
    """
    just implements some default deug representations for inbuilt types,
    namely `str` right now
    """
    if isinstance(value, float):
        return "{:.2f}".format(round(value, 2))
    return pprint.pformat(value, depth=1, width=72)


def parameter_str(args: tuple, kwargs: dict, include_type: bool = False) -> str:
    """
    create a parameter str from some args and some kwargs e.g
    ```py
    self.assertEqual(
        parameter_str((1, 2, 3), {'a': 5}),
        '1, 2, 3, a=5',
    )
    ```
    """
    return ', '.join([
        *[debugrepr(arg) + (f': {arg.__class__.__name__}' if include_type else '') for arg in args],
        *([
            f'{k}' + (f': {v.__class__.__name__}' 
            if include_type else '') + f' = {debugrepr(v)}' 
            for k, v in kwargs.items()
        ] if kwargs else []),
    ])


def __derive_from_imethod(imethod_getter: Callable[[T], Callable[[T, U], None]]) -> Callable[[T, U], Any]:
    # NOTE: Honestly, this might be some of the wors code I've ever written
    # the alternative to using imethod_getter(self)(...) would've been to use
    # __getattribute__ (or getattr) which is even more painfully slow. I honestly
    # hate python. I don't see why on earth ROS would use it. There must be better
    # lightweight statically typed and simple languages that it could use???
    # ...
    # ...
    # Rust! It's simple, fast, etc. etc.! Not exactly lightweight though with the
    # constant static dispath....
    """
    internal for deriving from imethods (e.g. __sub__ from __isub__)
    """

    def method(self, other):
        new = self.clone()
        imethod_getter(self)(new, other)
        return new
    return method


def derive(*args, **kwargs):
    """
    class decorator to derive some methods of an object, included 
    are:

    - `@derive __deconstruct__` depends on `__init__`
    - `@derive(*attributes) __repr__` a plain `@derive` will depend on `__deconstruct__`
    - `@derive clone` depends on `__deconstruct__`
    - `@derive __add__` depends on `__iadd__` and `clone`
    - `@derive __sub__` depends on `__isub__` and `clone`
    - `@derive __mul__` depends on `__imul__` and `clone`
    - `@derive __truediv__` depends on `__itruediv__` and `clone`

    You should implement a `__deconstruct__` method
    for these to work. `__deconstruct__` should not be derived as it would be 
    hideously slow
    ```py
    class Foo:
        def __init__(self):
            self.x = 1
            self.y = 2

        @derive('x', 'y')
        def __repr__(self): pass

    print(Foo())
    # Foo(x=1, y=2)
    ```
    """
    has_args = len(args) > 0 and not callable(args[0])

    def decorator(fn):
        if fn.__name__ == '__repr__':
            if has_args:
                def __repr__(self):
                    attrs = ', '.join(
                        [f'{attr}={getattr(self, attr)}' for attr in args]
                    )
                    return f'{self.__class__.__name__}({attrs})'
            else:
                def __repr__(self):
                    args, kwargs = self.__deconstruct__()
                    attrs = parameter_str(args, kwargs)
                    return f'{self.__class__.__name__}({attrs})'
            return __repr__

        if fn.__name__ == '__deconstruct__':
            def __deconstruct__(self):
                argspec = inspect.getfullargspec(self.__class__.__init__)
                kwarg_count = 0 if argspec.defaults is None else len(
                    argspec.defaults)
                ki = -kwarg_count if kwarg_count > 0 else len(argspec.args)
                arg_idents = argspec.args[1:ki]
                kwarg_idents = argspec.args[ki:]
                return (
                    tuple([self.__getattribute__(ident)
                          for ident in arg_idents]),
                    {k: v for k, v in [
                        (ident, self.__getattribute__(ident))
                        for ident in kwarg_idents
                    ]}
                )
            return __deconstruct__

        if fn.__name__ == 'clone':
            def clone(self):
                args, kwargs = self.__deconstruct__()
                return self.__class__(*args, **kwargs)
            return clone

        if fn.__name__ == '__sub__':
            return __derive_from_imethod(lambda self: self.__class__.__isub__)

        if fn.__name__ == '__add__':
            return __derive_from_imethod(lambda self: self.__class__.__iadd__)

        if fn.__name__ == '__mul__':
            return __derive_from_imethod(lambda self: self.__class__.__imul__)

        if fn.__name__ == '__truediv__':
            return __derive_from_imethod(lambda self: self.__class__.__itruediv__)

        else:
            raise ValueError(f'cannot derive function \'{fn.__name__}\'')
    if not has_args:
        return decorator(args[0])
    return decorator


def debug(print_hook: Callable[[str], Any]):
    """
    decorator for debugging a function call, using `print_hook` to
    display the data
    """
    def decorator(fn):
        def new_fn(*args, **kwargs):
            if inspect.ismethod(fn):
                usable_args = args[1:]
            else:
                usable_args = args
            print_hook(
                f'CALL {fn.__name__}({parameter_str(usable_args, kwargs, include_type=True)})')
            result = fn(*args, **kwargs)
            print_hook(f'RETURN {result.__class__.__name__}, value = {debugrepr(result)}\n')
            return result
        return new_fn
    return decorator


rosdebug = debug(rospy.loginfo)
"""
debug the input and output of a method using rospy.loginfo
"""
