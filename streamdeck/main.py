from frctools.streamdeck import StreamDeckBoard
from frctools.streamdeck.keys import StreamDeckKeyBool

from frc_streamdeck import ReefSelector, RandomMessageKey

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

# Random hype key: shows a random message from messages.txt, then clears
messages_file = Path(__file__).with_name('messages.txt')
board.set_key((7, 0), RandomMessageKey(str(messages_file), label='Hype', background='black', min_delay_s=4.0, max_delay_s=10.0, show_for_s=3.0))

# Start the stream deck
board.start()
board.wait_forever()
