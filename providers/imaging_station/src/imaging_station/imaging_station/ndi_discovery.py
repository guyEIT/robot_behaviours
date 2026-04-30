"""NDI source discovery via libndi.so.6 (ctypes).

Mirrors experiment_collector/backend/app/services/ndi_sdk_find.py. Used at
node startup to validate that the configured `ndi_source_name` is on the
network and to log the visible source list when it isn't. Frame receiving
is delegated to the C++ bridge; this module only handles
NDIlib_find_create_v2 / wait_for_sources / get_current_sources.
"""
from __future__ import annotations

import ctypes
import logging

from .ndi_runtime import preferred_ndi_runtime_dir

logger = logging.getLogger(__name__)


class NDIlib_find_create_t(ctypes.Structure):
    _fields_ = [
        ("show_local_sources", ctypes.c_bool),
        ("p_groups", ctypes.c_char_p),
        ("p_extra_ips", ctypes.c_char_p),
    ]


class NDIlib_source_t(ctypes.Structure):
    _fields_ = [
        ("p_ndi_name", ctypes.c_char_p),
        ("p_url_address", ctypes.c_char_p),
    ]


_lib: ctypes.CDLL | None = None
_initialized: bool = False
_finder: ctypes.c_void_p | None = None


def _load_lib() -> ctypes.CDLL | None:
    path = preferred_ndi_runtime_dir() / "libndi.so.6"
    if path.is_file():
        try:
            return ctypes.CDLL(str(path))
        except OSError as e:
            logger.warning("Failed to load %s: %s", path, e)

    try:
        return ctypes.CDLL("libndi.so.6")
    except OSError as e:
        logger.warning("libndi.so.6 not found on library path: %s", e)

    return None


def _setup_signatures(lib: ctypes.CDLL) -> None:
    lib.NDIlib_initialize.argtypes = []
    lib.NDIlib_initialize.restype = ctypes.c_bool

    lib.NDIlib_destroy.argtypes = []
    lib.NDIlib_destroy.restype = None

    lib.NDIlib_find_create_v2.argtypes = [ctypes.POINTER(NDIlib_find_create_t)]
    lib.NDIlib_find_create_v2.restype = ctypes.c_void_p

    lib.NDIlib_find_wait_for_sources.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.NDIlib_find_wait_for_sources.restype = ctypes.c_bool

    lib.NDIlib_find_get_current_sources.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint32),
    ]
    lib.NDIlib_find_get_current_sources.restype = ctypes.POINTER(NDIlib_source_t)

    lib.NDIlib_find_destroy.argtypes = [ctypes.c_void_p]
    lib.NDIlib_find_destroy.restype = None


def initialize() -> bool:
    global _lib, _initialized, _finder

    if _initialized:
        return True

    _lib = _load_lib()
    if _lib is None:
        logger.error("NDI SDK discovery unavailable: libndi.so.6 not found")
        return False

    try:
        _setup_signatures(_lib)
    except AttributeError as e:
        logger.error("libndi.so.6 missing expected symbols: %s", e)
        _lib = None
        return False

    if not _lib.NDIlib_initialize():
        logger.error("NDIlib_initialize() returned false")
        _lib = None
        return False

    create_settings = NDIlib_find_create_t(
        show_local_sources=True,
        p_groups=None,
        p_extra_ips=None,
    )
    _finder = _lib.NDIlib_find_create_v2(ctypes.byref(create_settings))
    if not _finder:
        logger.error("NDIlib_find_create_v2() returned NULL")
        _lib.NDIlib_destroy()
        _lib = None
        return False

    _initialized = True
    return True


def shutdown() -> None:
    global _lib, _initialized, _finder

    if _finder and _lib:
        _lib.NDIlib_find_destroy(_finder)
        _finder = None

    if _initialized and _lib:
        _lib.NDIlib_destroy()

    _lib = None
    _initialized = False


def discover(timeout_ms: int = 5000) -> list[dict]:
    """Return [{"ndi_name": str, "url_address": str}, ...] for every visible source."""
    if not _initialized or not _lib or not _finder:
        return []

    _lib.NDIlib_find_wait_for_sources(_finder, ctypes.c_uint32(timeout_ms))

    count = ctypes.c_uint32(0)
    sources_ptr = _lib.NDIlib_find_get_current_sources(_finder, ctypes.byref(count))

    results: list[dict] = []
    for i in range(count.value):
        src = sources_ptr[i]
        ndi_name = src.p_ndi_name.decode("utf-8", errors="replace") if src.p_ndi_name else ""
        url_address = src.p_url_address.decode("utf-8", errors="replace") if src.p_url_address else ""
        if ndi_name:
            results.append({"ndi_name": ndi_name, "url_address": url_address})

    return results


def find_source(ndi_name: str, timeout_ms: int = 5000) -> dict | None:
    """Discover and return the source matching ndi_name, or None if absent."""
    for src in discover(timeout_ms=timeout_ms):
        if src["ndi_name"] == ndi_name:
            return src
    return None
