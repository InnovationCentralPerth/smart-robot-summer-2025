"""Utility helpers for working with audio devices."""
from __future__ import annotations

import contextlib
import os
import sys
from typing import Iterator


@contextlib.contextmanager
def suppress_audio_errors() -> Iterator[None]:
    """Temporarily silence ALSA/JACK noise from stderr."""

    stderr_fd = sys.stderr.fileno()
    with open(os.devnull, "w", encoding="utf-8") as devnull:
        old_stderr = os.dup(stderr_fd)
        try:
            os.dup2(devnull.fileno(), stderr_fd)
            yield
        finally:
            os.dup2(old_stderr, stderr_fd)
            os.close(old_stderr)


__all__ = ["suppress_audio_errors"]