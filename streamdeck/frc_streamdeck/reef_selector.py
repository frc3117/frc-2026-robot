from frctools.streamdeck.keys import StreamDeckKeyBool

from math import floor

import ntcore


class ReefSelector:
    def __init__(self, streamdeck):
        inst = ntcore.NetworkTableInstance.getDefault()
        self.__nt_entry = inst.getEntry('/selected_reef')
        inst.addListener(self.__nt_entry, ntcore.EventFlags.kValueAll, self.__int_event__())

        self.__current_reef = self.__nt_entry.getInteger(0)

        self.__reef_key = []
        for i in range(8):
            x = floor(i / 4)
            y = i % 4

            key = StreamDeckKeyBool(label=f'{"Left" if x == 0 else "Right"}: {y+1}', true_img='green', false_img='gray')
            streamdeck.set_key((x, y), key)
            key.pressed_callback = self.__on_pressed__(i)

            self.__reef_key.append(key)

        self.set_current_reef(self.__current_reef, True)

    def __on_pressed__(self, id: int):
        def callback():
            self.set_current_reef(id)

        return callback

    def __int_event__(self):
        def e(event: ntcore.Event):
            self.set_current_reef(event.data.value.getInteger())

        return e

    def set_current_reef(self, reef_id, forced=False):
        if reef_id != self.__current_reef or forced:
            self.__current_reef = reef_id

            for i in range(8):
                key = self.__reef_key[i]
                key.state = i == reef_id

                self.__nt_entry.setInteger(reef_id)