"""Pruebas de odom_encoders.paso (sin ROS): python3 -m pytest -q rpi/test_odom_encoders.py"""
import math
import re
from pathlib import Path

import pytest

from odom_encoders import M_PER_TICK, TRACK, paso

TELEOP = Path(__file__).resolve().parent.parent / 'ros' / 'teleop_ros.py'


def _constante(nombre):
    texto = TELEOP.read_text(encoding='utf-8')
    return re.search(rf'^{nombre}\s*=\s*([^#\n]+)', texto, re.M).group(1).strip()


def test_constantes_iguales_a_teleop():
    wheel_d = float(_constante('WHEEL_D'))
    ticks = float(_constante('TICKS_PER_REV'))
    escala = float(_constante('CAL_ESCALA'))
    assert TRACK == pytest.approx(float(_constante('TRACK')))
    assert M_PER_TICK == pytest.approx(math.pi * wheel_d / ticks * escala)


def test_recto():
    n = 1000
    x, y, th = paso((0.0, 0.0, 0.0), (n, n, n, n))
    assert x == pytest.approx(n * M_PER_TICK, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert th == pytest.approx(0.0, abs=1e-12)


def test_giro_en_el_lugar():
    n = round((math.pi / 2) * TRACK / 2 / M_PER_TICK)
    x, y, th = paso((0.0, 0.0, 0.0), (-n, -n, n, n))      # izquierda atrás, derecha adelante = giro a la izquierda
    assert th == pytest.approx(math.pi / 2, abs=0.01)
    assert math.hypot(x, y) == pytest.approx(0.0, abs=1e-9)


def test_avanza_en_la_direccion_del_rumbo():
    n = 1000
    x, y, th = paso((0.0, 0.0, math.pi / 2), (n, n, n, n))
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(n * M_PER_TICK, abs=1e-9)
