import threading
from typing import *
from util.decorators import derive


T = TypeVar('T')
U = TypeVar('U')


class Mutex(Generic[T]):
    """
    Note that `T` must be clonable (you could use `@derive` for example)
    ```rs
    struct Mutex<T: Clone> {
        value: T,
        lock: threading::Lock,
    }
    ```
    """

    def __init__(self, value: T):
        self.__value = value
        self.__lock = threading.Lock()

    @derive('__value')
    def __repr__(self) -> str: pass

    def get_clone(self) -> Union[T, None]:
        """
        perform a **clone** of the underlying value, returning the result.
        the underlying value **must** have a valid clone method
        """
        
        value = None
        with self.__lock:
            value = self.__value.clone() if self.__value is not None else None
        return value

    def set(self, value: T):
        """
        Try set the underlying value of the Mutex, don't access the value you've
        passed again. That's just stupid.

        **important note**: `value` should then be considered not thread safe!
        We do not perform any kind of copy of this value. If you want to reuse `value`
        then you should do `Mutex.set(value.clone())`
        """
        
        with self.__lock:
            self.__value = value
        return value

    def get_mut(self, setter: Callable[[T], U]) -> U:
        """
        Lock the mutex and get the underlying value as the first parameter to a `setter`
        callback. If `T` is copy-only (e.g. `int`) this is just going to return a copy
        (sorry). `mut` is for `mutable` not `mutex`! `mutex` is `mtx`!!

        It's obviously a poort implemetnation for a variety of reasons (doesn't do any
        checks that you won't just return the underlying value) but whatever, it's fine

        ```py
        mutex_list = Mutex([])
        def append_and_get_length(mtx_list_mut: list) -> int:
            mtx_list_mut.append(1)
            return len(mtx_list_mut)
        length = mutex_list.get_mut(append_and_get_length)
        print(length)  # 1
        ```
        """

        with self.__lock:
            result = setter(self.__value)
        return result