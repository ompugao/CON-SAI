# -*- coding: utf-8 -*-
__author__ = "Shohei Fujii"
__version__ = "0.0.0"

from python_qt_binding import QT_BINDING_VERSION
g_PYQT_MAJOR_VERSION = int(QT_BINDING_VERSION.split('.')[0])
def get_qt_version():
    global g_PYQT_MAJOR_VERSION
    return g_PYQT_MAJOR_VERSION


# monkey patch
import types
import functools
from python_qt_binding.QtGui import QMouseEvent
def mouseevent_wrapper(func):
    global g_PYQT_MAJOR_VERSION
    if g_PYQT_MAJOR_VERSION == 5:
        @functools.wraps(func)
        def wrapper(self, event):
            def _posF(self):
                return self.localPos()
            event.posF = types.MethodType(_posF, event)
            return func(self, event)
        return wrapper
    else:
        return func

def wheelevent_wrapper(func):
    global g_PYQT_MAJOR_VERSION
    if g_PYQT_MAJOR_VERSION == 5:
        @functools.wraps(func)
        def wrapper(self, event):
            def _delta(self):
                return self.angleDelta().y()
            event.delta = types.MethodType(_delta, event)
            return func(self, event)
        return wrapper
    else:
        return func
