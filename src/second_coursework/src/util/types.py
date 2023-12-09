from typing import *
from util.decorators import derive

T = TypeVar('T')


class IndexedTuple(Generic[T]):
    """
    Wraps a size `n` tuple, each call to `IndexedTuple.next()` moves the internal pointer
    to the next element.
    e.g.
    ```py
    >>> lt = IndexedTuple(1, 2, 3)
    >>> lt.get()
    1
    >>> lt.next().next().get()
    3
    >>> lt.next().get()
    1
    ```
    """

    def __init__(self, *values: T, start_index=None):
        """
        - `values`: the values that this tuple should contain
        - `start_index`: the index that this should start on. Capped to `len(values) -1`. `None` will
          default to `len(values) - 1`
        """
        self.__tuple = tuple(values)
        if start_index is None:
            start_index = len(self.__tuple)
        self.__i = start_index % len(self.__tuple)

    def __deconstruct__(self):
        return (self.__tuple, None)

    @derive
    def __repr__(self): pass

    def next(self) -> 'IndexedTuple[T]':
        """
        Moves the internal pointer one step forward (or loops if at the end)
        """
        self.__i = (self.__i + 1) % len(self.__tuple)
        return self

    def get(self) -> T:
        """
        Get the current value pointed to by the internal pointer
        """
        return self.__tuple[self.__i]
