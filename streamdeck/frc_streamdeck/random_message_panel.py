from frctools.streamdeck.keys import StreamDeckKey

from pathlib import Path
from random import choice, uniform
from typing import Dict, List, Tuple

import time

from PIL import ImageDraw, ImageFont


Coord = Tuple[int, int]


def _load_font(size: int):
    # DejaVu is usually available on Linux; fallback to PIL default if missing.
    try:
        return ImageFont.truetype('DejaVuSans.ttf', size)
    except Exception:
        return ImageFont.load_default()


def _fit_font_for_text(text: str, max_width: int, max_height: int, min_size: int, max_size: int):
    if not text:
        return _load_font(min_size)

    lo, hi = min_size, max_size
    best = _load_font(min_size)

    while lo <= hi:
        mid = (lo + hi) // 2
        font = _load_font(mid)
        l, t, r, b = font.getbbox(text)
        w, h = (r - l), (b - t)

        if w <= max_width and h <= max_height:
            best = font
            lo = mid + 1
        else:
            hi = mid - 1

    return best


class _RandomMessageState:
    def __init__(self, messages_path: str, min_delay_s: float, max_delay_s: float, show_for_s: float):
        if min_delay_s < 0 or max_delay_s < min_delay_s:
            raise ValueError('Invalid delay range')
        if show_for_s <= 0:
            raise ValueError('show_for_s must be > 0')

        self._messages = self._load_messages(messages_path)
        if len(self._messages) == 0:
            raise ValueError(f'No messages found in {messages_path}')

        self._min_delay_s = min_delay_s
        self._max_delay_s = max_delay_s
        self._show_for_s = show_for_s

        self.current_text = None
        self._show_until = None
        self._next_show_at = time.monotonic() + uniform(self._min_delay_s, self._max_delay_s)

    @staticmethod
    def _load_messages(messages_path: str) -> list[str]:
        lines = Path(messages_path).read_text(encoding='utf-8').splitlines()
        return [line.strip() for line in lines if line.strip()]

    def tick(self):
        now = time.monotonic()

        if self.current_text is None:
            if now >= self._next_show_at:
                self.current_text = choice(self._messages)
                self._show_until = now + self._show_for_s
                return True
            return False

        if now >= self._show_until:
            self.current_text = None
            self._show_until = None
            self._next_show_at = now + uniform(self._min_delay_s, self._max_delay_s)
            return True

        return False


class _PanelLayout:
    def __init__(self, coords: List[Coord], h_align: str, v_align: str):
        if len(coords) == 0:
            raise ValueError('coords must not be empty')

        self.coords = list(coords)
        self.h_align = h_align
        self.v_align = v_align

        xs = sorted(set(x for x, _ in coords))
        ys = sorted(set(y for _, y in coords), reverse=True)
        self._x_to_col = {x: i for i, x in enumerate(xs)}
        self._y_to_row = {y: i for i, y in enumerate(ys)}

        self.cols = len(xs)
        self.rows = len(ys)
        self.capacity = len(coords)

        # Only support rectangular 3x3/2x2 style grids for predictable centering.
        if self.rows * self.cols != self.capacity:
            raise ValueError('coords must form a full rectangular grid')

        # coord -> linear slot index (row-major). Order-independent by design.
        self.coord_to_slot: Dict[Coord, int] = {}
        for coord in coords:
            x, y = coord
            row = self._y_to_row[y]
            col = self._x_to_col[x]
            self.coord_to_slot[coord] = row * self.cols + col

    def words_to_slot_map(self, words: List[str], panel_size: int) -> Dict[int, str]:
        if panel_size <= 0:
            return {}

        usable_slots = min(panel_size, self.capacity)
        words = words[:usable_slots]
        if len(words) == 0:
            return {}

        slot_to_word: Dict[int, str] = {}

        rows_usable = min(self.rows, (usable_slots + self.cols - 1) // self.cols)
        words_remaining = len(words)
        word_index = 0

        if self.v_align == 'top':
            start_row = 0
        elif self.v_align == 'bottom':
            start_row = self.rows - rows_usable
        else:  # center
            start_row = (self.rows - rows_usable) // 2

        for local_row in range(rows_usable):
            row = start_row + local_row
            row_capacity = min(self.cols, usable_slots - (local_row * self.cols))
            row_words = min(row_capacity, words_remaining)

            if row_words <= 0:
                break

            if self.h_align == 'left':
                start_col = 0
            elif self.h_align == 'right':
                start_col = self.cols - row_words
            else:  # center
                start_col = (self.cols - row_words) // 2

            for i in range(row_words):
                col = start_col + i
                slot = row * self.cols + col
                slot_to_word[slot] = words[word_index]
                word_index += 1
                words_remaining -= 1

                if words_remaining <= 0:
                    return slot_to_word

        return slot_to_word


class RandomMessagePanelKey(StreamDeckKey):
    def __init__(
        self,
        state: _RandomMessageState,
        layout: _PanelLayout,
        coord: Coord,
        panel_size: int,
        label: str = None,
        background: str = 'black',
        margin: int = 6,
        min_font_size: int = 10,
        max_font_size: int = 36,
    ):
        super().__init__()
        self._state = state
        self._layout = layout
        self._coord = coord
        self._slot = layout.coord_to_slot[coord]
        self._panel_size = max(1, min(panel_size, layout.capacity))

        self._label = label
        self._background = background
        self._margin = max(0, margin)
        self._min_font_size = min_font_size
        self._max_font_size = max_font_size

        self._last_text = None

    def _text_for_me(self):
        text = self._state.current_text
        if not text:
            return None

        words = text.split()
        slot_map = self._layout.words_to_slot_map(words, self._panel_size)
        return slot_map.get(self._slot)

    def update(self):
        changed = self._state.tick()
        text = self._text_for_me()

        if changed or text != self._last_text:
            self._last_text = text
            self.update_img()

        super().update()

    def generate_img(self):
        img = self.create_img(self._background, label=self._label)

        if self._last_text:
            draw = ImageDraw.Draw(img)
            max_w = max(1, img.width - (2 * self._margin))
            max_h = max(1, img.height - (2 * self._margin))
            font = _fit_font_for_text(self._last_text, max_w, max_h, self._min_font_size, self._max_font_size)
            draw.text((img.width / 2, img.height / 2), self._last_text, fill='white', font=font, anchor='mm', align='center')

        return img


def build_random_message_panel(
    messages_path: str,
    coords: List[Coord],
    panel_size: int = None,
    label: str = 'Roast',
    background: str = 'black',
    min_delay_s: float = 4.0,
    max_delay_s: float = 10.0,
    show_for_s: float = 3.0,
    margin: int = 6,
    h_align: str = 'center',
    v_align: str = 'center',
    min_font_size: int = 10,
    max_font_size: int = 36,
):
    if h_align not in ('left', 'center', 'right'):
        raise ValueError("h_align must be 'left', 'center', or 'right'")
    if v_align not in ('top', 'center', 'bottom'):
        raise ValueError("v_align must be 'top', 'center', or 'bottom'")

    layout = _PanelLayout(coords, h_align=h_align, v_align=v_align)
    if panel_size is None:
        panel_size = layout.capacity

    state = _RandomMessageState(messages_path, min_delay_s, max_delay_s, show_for_s)

    keys_by_coord: Dict[Coord, RandomMessagePanelKey] = {}
    # label only on the top-left coordinate (small visual cue)
    top_left = min(coords, key=lambda c: (c[1], c[0]))

    for coord in coords:
        panel_label = label if coord == top_left else None
        keys_by_coord[coord] = RandomMessagePanelKey(
            state,
            layout,
            coord,
            panel_size=panel_size,
            label=panel_label,
            background=background,
            margin=margin,
            min_font_size=min_font_size,
            max_font_size=max_font_size,
        )

    return keys_by_coord
