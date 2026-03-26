from frctools.streamdeck.keys import StreamDeckKey

from pathlib import Path
from random import choice, uniform
import time


class RandomMessageKey(StreamDeckKey):
    def __init__(
        self,
        messages_path: str,
        label: str = None,
        background: str = 'black',
        min_delay_s: float = 4.0,
        max_delay_s: float = 9.0,
        show_for_s: float = 3.0,
    ):
        super().__init__()

        if min_delay_s < 0 or max_delay_s < min_delay_s:
            raise ValueError('Invalid delay range')
        if show_for_s <= 0:
            raise ValueError('show_for_s must be > 0')

        self.__label = label
        self.__background = background
        self.__min_delay_s = min_delay_s
        self.__max_delay_s = max_delay_s
        self.__show_for_s = show_for_s

        self.__messages = self.__load_messages(messages_path)
        if len(self.__messages) == 0:
            raise ValueError(f'No messages found in {messages_path}')

        self.__current_text = None
        self.__show_until = None
        self.__next_show_at = time.monotonic() + uniform(self.__min_delay_s, self.__max_delay_s)

    @staticmethod
    def __load_messages(messages_path: str) -> list[str]:
        lines = Path(messages_path).read_text(encoding='utf-8').splitlines()
        return [line.strip() for line in lines if line.strip()]

    def update(self):
        now = time.monotonic()

        if self.__current_text is None:
            if now >= self.__next_show_at:
                self.__current_text = choice(self.__messages)
                self.__show_until = now + self.__show_for_s
                self.update_img()
        else:
            if now >= self.__show_until:
                self.__current_text = None
                self.__show_until = None
                self.__next_show_at = now + uniform(self.__min_delay_s, self.__max_delay_s)
                self.update_img()

        super().update()

    def generate_img(self):
        return self.create_img(self.__background, label=self.__label, value=self.__current_text)
