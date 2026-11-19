"""The record every animation returns: the renderer's clip, shown and saved through it."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from minilink.graphical.common.environment import resolve_show
from minilink.graphical.common.show import hold_back

###############################################################################
# Public API
###############################################################################


@dataclass
class AnimationResult:
    """
    An animation drawn with ``renderer``. Two properties read its clip: its ``figure`` (matplotlib's or
    Plotly's figure, Meshcat's viewer; ``None`` on pygame, which draws live)
    and its ``animation`` (matplotlib's ``FuncAnimation``, Meshcat's
    ``Animation``; ``None`` on Plotly, whose frames sit in the figure).

    The clip is built on first use: a record made with ``show=False`` starts
    no viewer until it is shown or saved.
    """

    renderer: str
    clip: Any = field(default=None, repr=False)

    @property
    def figure(self):
        """The figure, or viewer, the animation plays in."""
        return None if self.clip is None else self.clip.built()[0]

    @property
    def animation(self):
        """The renderer's animation object."""
        return None if self.clip is None else self.clip.built()[1]

    def show(self):
        """Play the animation where the environment puts it, as ``show=True`` does."""
        place = resolve_show(True, self.renderer)
        if place is not None:
            self.clip.show(place)

        return self

    def save(self, file_name):
        """
        Write the animation to ``file_name``, in the format of its extension.

        Matplotlib writes a ``.gif`` (the default) or a video through ffmpeg
        (``.mp4``); Meshcat and Plotly a self-contained ``.html`` page.
        """
        return self.clip.save(Path(file_name))

    # Display

    def _ipython_display_(self):
        """Display nothing in a notebook: the animate call has already played it."""


def show_animation(figure, animation, *, fps, show, ax=None):
    """
    A matplotlib ``FuncAnimation`` a plot function built, as a record, played where ``show`` resolves.

    An animation drawn on the caller's ``ax`` is not played: the figure is its
    owner's to show.
    """
    result = AnimationResult(
        renderer="matplotlib", clip=BuiltClip(figure, animation, fps)
    )

    # Our own figure is played now, or held back from a notebook's end-of-cell display
    if ax is None:
        place = resolve_show(show, "matplotlib")
        if place is not None:
            result.clip.show(place)
        else:
            hold_back(figure, "matplotlib")

    return result


def require_matplotlib_animation(renderer):
    """Refuse a renderer other than matplotlib for an animation drawn with matplotlib alone."""
    if renderer != "matplotlib":
        raise ValueError(
            f"this animation is drawn with matplotlib alone, got renderer={renderer!r}; "
            'pass renderer="matplotlib"'
        )


###############################################################################
# Internal machinery
###############################################################################


class Clip:
    """
    The frames of a trajectory on one renderer, built into the renderer's clip
    on first use and shared by every show and save.
    """

    def __init__(self, backend, frames, schedule, *, is_3d=False, scene_title=None):
        self.backend = backend
        self.frames = frames
        self.schedule = schedule
        self.is_3d = is_3d
        self.scene_title = scene_title
        self.clip = None

    def built(self):
        """The renderer's ``(figure, animation)``, built once."""
        if self.clip is None:
            primitives = self.frames[0]["primitives"] if self.frames else []
            self.clip = self.backend.build_clip(
                primitives,
                self.frames,
                self.schedule,
                is_3d=self.is_3d,
                scene_title=self.scene_title,
            )

        return self.clip

    def show(self, place):
        """Play the clip ``"inline"`` or in a ``"window"``."""
        self.backend.show_clip(self, place)

    def save(self, path):
        """Write the clip to ``path``; the path written."""
        return self.backend.save_clip(self, path)


class BuiltClip:
    """A clip a plot function built itself: a matplotlib figure and its ``FuncAnimation``."""

    def __init__(self, figure, animation, fps):
        self.clip = (figure, animation)
        self.fps = fps

    def built(self):
        """The ``(figure, animation)``."""
        return self.clip

    def show(self, place):
        """Play the clip ``"inline"`` or in a ``"window"``."""
        # Lazy import: the matplotlib renderer loads with its first clip
        from minilink.graphical.animation.renderers.matplotlib_renderer import (
            show_matplotlib_clip,
        )

        show_matplotlib_clip(*self.clip, place)

    def save(self, path):
        """Write the clip to ``path``; the path written."""
        # Lazy import: the matplotlib renderer loads with its first clip
        from minilink.graphical.animation.renderers.matplotlib_renderer import (
            save_matplotlib_clip,
        )

        return save_matplotlib_clip(self.clip[1], path, self.fps)
