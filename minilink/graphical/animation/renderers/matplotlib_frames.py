"""Matplotlib frames for a clip's files and a live window: drawn in layers, written as a player, a GIF, a video."""

from __future__ import annotations

import matplotlib as mpl
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import matplotlib.image as image
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.layout_engine import PlaceHolderLayoutEngine
from PIL import Image

###############################################################################
# Public API
###############################################################################


class LayeredFrames:
    """
    The frames of a flat animation as images, each the same as a full draw of the figure.

    A full draw of a figure redraws what does not move: the axes with their
    grid, ticks and labels, the figure itself. The animation's axes lie under
    every shape, so while the view (the axis limits and labels) is the one of
    the frame before, they come from an image kept for that view, and only the
    primitives' artists, the frame and the title are drawn over it, in the
    order of a full draw. A frame on a new view is a full draw.

    ``rgba()`` gives the frame the figure holds, between :meth:`open` and
    :meth:`close`; ``draw_on`` draws it into a canvas the figure is shown on.
    """

    def __init__(self, fig, ax, primitives):
        self.fig = fig
        self.ax = ax
        self.primitives = primitives  # The canvas of primitives: its drawn_objects
        self.own_canvas = None  # The figure's own canvas and dpi, set back at close
        self.own_dpi = None
        self.agg = None  # The canvas drawn into
        self.view = None  # The view of the last frame
        self.title_position = None  # Where a full draw of this view puts the title
        self.background = None  # The image kept for this view

    def open(self, dpi) -> None:
        """Draw at ``dpi`` on a canvas of its own until :meth:`close`."""
        self.own_canvas = self.fig.canvas
        self.own_dpi = self.fig.dpi
        # On the canvas of its own, the resolution changes no window's size
        self.agg = QuietCanvas(self.fig)
        self.fig.set_dpi(dpi)
        self.view = None
        self.background = None

    def close(self) -> None:
        """The figure back on its own canvas, at its own resolution."""
        if self.own_canvas is None:
            return
        self.fig.set_dpi(self.own_dpi)
        self.fig.set_canvas(self.own_canvas)
        self.own_canvas = None
        self.agg = None
        self.background = None

    def rgba(self) -> np.ndarray:
        """The frame the figure holds, ``(height, width, 4)`` bytes."""
        self.draw_on(self.agg)

        return np.asarray(self.agg.buffer_rgba())

    def draw_on(self, agg) -> None:
        """Draw the frame the figure holds into the image of ``agg``, an Agg canvas of the figure."""
        self.agg = agg
        view = self.view_key()
        if not self.can_layer() or view != self.view:
            # A new view: a full draw, which also places its ticks
            self.view = view
            self.background = None
            agg.draw()
            self.title_position = self.ax.title.get_position()
            return
        if self.background is None:
            self.keep_background()
        self.draw_over_background()

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def view_key(self) -> tuple:
        """What the background depends on: the limits, the axis labels, the image's size."""
        ax = self.ax
        return (
            ax.get_xlim(),
            ax.get_ylim(),
            ax.get_xlabel(),
            ax.get_ylabel(),
            self.agg.get_width_height(),
            self.fig.dpi,
        )

    def can_layer(self) -> bool:
        """True when a full draw only draws what the layers order: axes under the shapes, no layout to redo, no other artist."""
        ax = self.ax
        if ax.get_axisbelow() is not True:
            # A grid over some of the shapes belongs between them, not on the background
            return False
        engine = self.fig.get_layout_engine()
        if engine is not None and not isinstance(engine, PlaceHolderLayoutEngine):
            # A layout engine places the axes from everything drawn, the title included
            return False
        known = {
            *(id(artist) for artist in self.primitives.drawn_objects),
            *(id(spine) for spine in ax.spines.values()),
            id(ax.xaxis),
            id(ax.yaxis),
            id(ax.title),
            id(ax.patch),
        }
        others = [
            child
            for child in ax.get_children()
            if id(child) not in known and not is_empty_text(child)
        ]
        return not others and ax.axison and ax.get_frame_on()

    def keep_background(self) -> None:
        """Draw and keep what stays on this view: everything but the layers drawn per frame."""
        over = self.layers_over()
        was_visible = [artist.get_visible() for artist in over]
        for artist in over:
            artist.set_visible(False)
        try:
            # Into the image only: a window is shown the finished frame
            FigureCanvasAgg.draw(self.agg)
            image_kept = self.agg.copy_from_bbox(self.fig.bbox)
        finally:
            for artist, visible in zip(over, was_visible):
                artist.set_visible(visible)
        self.background = image_kept

    def layers_over(self) -> list:
        """The artists drawn over the background, in the order a full draw gives them."""
        ax = self.ax
        children = [*self.primitives.drawn_objects, *ax.spines.values(), ax.title]

        return sorted(children, key=lambda artist: artist.zorder)

    def draw_over_background(self) -> None:
        renderer = self.agg.get_renderer()
        self.agg.restore_region(self.background)
        # A draw of the axes places the title; the layers put it where the
        # full draw of this view did
        self.ax.title.set_position(self.title_position)
        for artist in self.layers_over():
            artist.draw(renderer)


class LayeredSaving:
    """
    Mixin of a movie writer, listed before the writer class: its frames come from :class:`LayeredFrames`.

    For writers that take ``png`` or raw ``rgba`` frames from ``savefig``.
    """

    def __init__(self, layers, **kwargs):
        super().__init__(**kwargs)
        self.layers = layers

    def setup(self, fig, outfile, dpi=None, **kwargs):
        super().setup(fig, outfile, dpi=dpi, **kwargs)
        self.layers.open(self.dpi)
        self.fig = LayeredFigure(fig, self.layers)

    def finish(self):
        if isinstance(self.fig, LayeredFigure):
            self.fig = self.fig.fig
        self.layers.close()
        super().finish()


class LayeredHTMLWriter(LayeredSaving, animation.HTMLWriter):
    """The HTML player, its frames drawn in layers."""


class ClipPaletteWriter(animation.PillowWriter):
    """
    The GIF writer with one palette for the whole clip.

    A GIF holds 256 colours. The palette is reduced once, from the colours of
    every frame, and each colour goes to its nearest entry; a clip whose
    colours one palette cannot hold keeps a palette per frame.
    """

    def finish(self):
        # ``_frames`` is the writer's own list of grabbed frames, with no public
        # twin; the GIF tests of test_graphics.py fail if it moves.
        frames = self._frames
        indexed = None
        if all(frame.mode == "RGB" for frame in frames):
            indexed = frames_on_one_palette(frames)
        if indexed is None:
            # Frames with transparency, or too many colours for one palette
            super().finish()
            return
        indexed[0].save(
            self.outfile,
            save_all=True,
            append_images=indexed[1:],
            duration=int(1000 / self.fps),
            loop=0,
        )


class LayeredPillowWriter(LayeredSaving, ClipPaletteWriter):
    """The GIF writer, its frames drawn in layers."""


class LayeredFFMpegWriter(LayeredSaving, animation.FFMpegWriter):
    """The ffmpeg video writer, its frames drawn in layers."""


# Pixels of the image a clip's palette is reduced from: its colours, each as
# often as its share of the clip
_PALETTE_PIXELS = 1 << 21
# A colour counts when it first fills this many pixels of a frame, and one
# palette holds the clip when each such colour has an entry this near [RGB levels]
_COLOUR_PIXELS = 64
_COLOUR_TOLERANCE = 16.0
# Colours whose nearest entries are found at a time
_COLOUR_CHUNK = 4096


def frames_on_one_palette(frames):
    """
    RGB ``frames`` as palette images sharing one palette of 256 colours.

    ``None`` when no single palette holds the clip: a colour that fills part
    of a frame would land far from every entry. Uses two tables over the 2^24
    colours (32 MB) while it runs.
    """
    # The colours of the clip, each with the pixels it fills where it first appears
    seen = np.zeros(1 << 24, dtype=bool)
    colours, pixels = [], []
    for frame in frames:
        code = colour_codes(frame)
        new, count = np.unique(code[~seen[code]], return_counts=True)
        seen[new] = True
        colours.append(new)
        pixels.append(count)
    colours = np.concatenate(colours)
    pixels = np.concatenate(pixels)
    rgb = np.stack([colours >> 16, (colours >> 8) & 255, colours & 255], axis=1)
    rgb = rgb.astype(np.uint8)

    # The palette: the colours themselves when they fit, else the median cut of
    # an image holding each colour as often as its share of the pixels
    if len(colours) <= 256:
        palette = rgb
    else:
        share = pixels * (_PALETTE_PIXELS / pixels.sum())
        repeats = np.maximum(1, np.rint(share)).astype(np.int64)
        strip = np.repeat(rgb, repeats, axis=0)
        width = 2048
        height = -(-len(strip) // width)
        padded = np.concatenate(
            [strip, np.repeat(strip[-1:], width * height - len(strip), axis=0)]
        )
        reduced = Image.fromarray(padded.reshape(height, width, 3), mode="RGB")
        reduced = reduced.convert("P", palette=Image.Palette.ADAPTIVE)
        palette = np.array(reduced.getpalette(), dtype=np.uint8).reshape(-1, 3)

    # Each colour to its nearest entry
    entry = np.zeros(len(colours), dtype=np.uint8)
    distance = np.zeros(len(colours))
    for start in range(0, len(colours), _COLOUR_CHUNK):
        chunk = rgb[start : start + _COLOUR_CHUNK].astype(np.int32)
        squared = ((chunk[:, None, :] - palette[None].astype(np.int32)) ** 2).sum(
            axis=2
        )
        nearest = np.argmin(squared, axis=1)
        entry[start : start + _COLOUR_CHUNK] = nearest
        distance[start : start + _COLOUR_CHUNK] = np.sqrt(
            squared[np.arange(len(chunk)), nearest]
        )
    if np.any(distance[pixels >= _COLOUR_PIXELS] > _COLOUR_TOLERANCE):
        return None

    # The frames, each pixel the entry of its colour
    entry_of = np.zeros(1 << 24, dtype=np.uint8)
    entry_of[colours] = entry
    flat = np.zeros((256, 3), dtype=np.uint8)
    flat[: len(palette)] = palette
    flat = flat.reshape(-1).tolist()
    indexed = []
    for frame in frames:
        frame_p = Image.fromarray(entry_of[colour_codes(frame)], mode="P")
        frame_p.putpalette(flat)
        indexed.append(frame_p)

    return indexed


###############################################################################
# Internal machinery
###############################################################################


class QuietCanvas(FigureCanvasAgg):
    """An image canvas that draws only when a frame is asked for, never on a redraw request."""

    def draw_idle(self, *args, **kwargs):
        pass


def is_empty_text(artist) -> bool:
    """True for a text artist with nothing to draw (the unused side titles of an axes)."""
    return hasattr(artist, "get_text") and artist.get_text() == ""


class LayeredFigure:
    """Stands in for the figure in a movie writer: ``savefig`` writes the layered frame."""

    def __init__(self, fig, layers):
        self.fig = fig
        self.layers = layers

    def savefig(self, target, *, format, dpi, **savefig_kwargs):
        if format not in ("png", "rgba") or not keeps_figure_colours(
            self.fig, savefig_kwargs
        ):
            # A frame the layers do not write: the figure's own, a full draw
            self.fig.savefig(target, format=format, dpi=dpi, **savefig_kwargs)
            return
        rgba = self.layers.rgba()
        if format == "png":
            image.imsave(target, rgba, format="png", origin="upper", dpi=dpi)
        else:
            target.write(rgba.tobytes())

    def __getattr__(self, name):
        if name == "fig":
            raise AttributeError(name)

        return getattr(self.fig, name)


def keeps_figure_colours(fig, savefig_kwargs) -> bool:
    """True when a save with these options paints the figure in its own face and edge colours."""
    if savefig_kwargs.get("transparent", mpl.rcParams["savefig.transparent"]):
        return False
    own_colours = (
        ("facecolor", fig.get_facecolor()),
        ("edgecolor", fig.get_edgecolor()),
    )
    for side, own in own_colours:
        colour = savefig_kwargs.get(side, mpl.rcParams[f"savefig.{side}"])
        if isinstance(colour, str) and colour == "auto":
            continue
        if not mcolors.same_color(colour, own):
            return False

    return True


def colour_codes(frame) -> np.ndarray:
    """The pixels of an RGB image as one integer each, ``r·2¹⁶ + g·2⁸ + b``."""
    rgb = np.asarray(frame)
    code = (
        (rgb[..., 0].astype(np.uint32) << 16)
        | (rgb[..., 1].astype(np.uint32) << 8)
        | rgb[..., 2]
    )

    return code
