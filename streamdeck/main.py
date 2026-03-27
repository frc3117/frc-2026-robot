from frctools.streamdeck import StreamDeckBoard
from frctools.streamdeck.keys import StreamDeckKeyBool

from frc_streamdeck import ReefSelector, build_random_message_panel

import ntcore
from pathlib import Path


# Initialize the network table
nt = ntcore.NetworkTableInstance.getDefault()
nt.startClient4('streamdeck')
#nt.setServer('localhost', ntcore.NetworkTableInstance.kDefaultPort4)
nt.setServer('10.31.17.2', ntcore.NetworkTableInstance.kDefaultPort4)

# Initialize the board
board = StreamDeckBoard()

# Add the is connected key to the top right
board.set_key((7, 3), StreamDeckKeyBool(label='Is Connected', true_img='green', false_img='red').bind_nt_connected())

board.set_key((3, 2), StreamDeckKeyBool(label='Elevator at Height', true_img='red', false_img='gray').bind_nt('/SmartDashboard/Elevator/at_height'))
board.set_key((4, 2), StreamDeckKeyBool(label='Climb Prox', true_img='red', false_img='gray').bind_nt('/SmartDashboard/Climber/has_cage'))

#board.set_key((3, 2), StreamDeckKeyBool(label='conveyor prox', true_img='red', false_img='gray').bind_nt('/SmartDashboard/Conveyor/has_coral'))
#board.set_key((4, 2), StreamDeckKeyBool(label='outtake prox', true_img='red', false_img='gray').bind_nt('/SmartDashboard/CoralOuttake/has_coral'))

# Add the reef selector to the left
reef = ReefSelector(board)

# Random roast panel: 3x3 coordinate-driven layout, order-independent mapping
messages_file = Path(__file__).with_name('messages.txt')
roast_coords = [
    (5, 0), (6, 0), (7, 0),
    (5, 1), (6, 1), (7, 1),
    (5, 2), (6, 2), (7, 2),
]

roast_keys = build_random_message_panel(
    str(messages_file),
    coords=roast_coords,
    panel_size=9,      # use fewer than 9 if you want a smaller active panel
    label='Roast',
    background='black',
    min_delay_s=4.0,
    max_delay_s=10.0,
    show_for_s=3.0,
    margin=8,          # inner text margin in px
    h_align='center',  # left|center|right
    v_align='center',  # top|center|bottom
    min_font_size=10,
    max_font_size=36,
    roast_level='feral',  # light|medium|feral|all
)

for coord in roast_coords:
    board.set_key(coord, roast_keys[coord])

# Start the stream deck
board.start()
board.wait_forever()
