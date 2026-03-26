from frctools.streamdeck.keys import StreamDeckKey

from pathlib import Path
from random import choice, uniform
import math
import time


class _RandomMessageState:
    def __init__(
        self,
        messages_path: str,
        min_delay_s: float,
        max_delay_s: float,
        show_for_s: float,
        panel_capacity: int,
        panel_size: int,
    ):
        if min_delay_s < 0 or max_delay_s < min_delay_s:
            raise ValueError('Invalid delay range')
        if show_for_s <= 0:
            raise ValueError('show_for_s must be > 0')
        if panel_capacity <= 0:
            raise ValueError('panel_capacity must be > 0')

        self._messages = self._load_messages(messages_path)
        if len(self._messages) == 0:
            raise ValueError(f'No messages found in {messages_path}')

        self._min_delay_s = min_delay_s
        self._max_delay_s = max_delay_s
        self._show_for_s = show_for_s
        self._panel_capacity = panel_capacity

        self.active_panel_size = self._clamp_size(panel_size)

        self.current_text = None
        self._show_until = None
        self._next_show_at = time.monotonic() + uniform(self._min_delay_s, self._max_delay_s)

    @staticmethod
    def _load_messages(messages_path: str) -> list[str]:
        lines = Path(messages_path).read_text(encoding='utf-8').splitlines()
        return [line.strip() for line in lines if line.strip()]

    def _clamp_size(self, size: int) -> int:
        return max(1, min(self._panel_capacity, int(size)))

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


class RandomMessagePanelKey(StreamDeckKey):
    def __init__(self, state: _RandomMessageState, panel_index: int, label: str = None, background: str = 'black'):
        super().__init__()
        self._state = state
        self._panel_index = panel_index
        self._label = label
        self._background = background
        self._last_slice = None

    def _slice_for_panel(self):
        text = self._state.current_text
        if not text:
            return None

        active_size = self._state.active_panel_size
        if self._panel_index >= active_size:
            return None

        words = text.split()
        if len(words) == 0:
            return None

        per_panel = math.ceil(len(words) / active_size)
        start = self._panel_index * per_panel
        end = start + per_panel
        chunk = ' '.join(words[start:end]).strip()
        return chunk if chunk else None

    def update(self):
        changed = self._state.tick()
        current_slice = self._slice_for_panel()

        if changed or current_slice != self._last_slice:
            self._last_slice = current_slice
            self.update_img()

        super().update()

    def generate_img(self):
        return self.create_img(self._background, label=self._label, value=self._last_slice)


def build_random_message_panel(
    messages_path: str,
    panel_capacity: int,
    panel_size: int = None,
    label: str = 'Roast',
    background: str = 'black',
    min_delay_s: float = 4.0,
    max_delay_s: float = 10.0,
    show_for_s: float = 3.0,
):
    if panel_size is None:
        panel_size = panel_capacity

    state = _RandomMessageState(
        messages_path,
        min_delay_s,
        max_delay_s,
        show_for_s,
        panel_capacity=panel_capacity,
        panel_size=panel_size,
    )

    keys = []
    for i in range(panel_capacity):
        panel_label = label if i == 0 else None
        keys.append(RandomMessagePanelKey(state, i, label=panel_label, background=background))

    return keys
