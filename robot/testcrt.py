from frc_ballistic_solver import CRTEncoder, CRT


encoder_a = CRTEncoder(18, 207, 360.)
encoder_b = CRTEncoder(23, 207, 360.)

crt = CRT(encoder_a, encoder_b, 207)
crt.set_turn(0.7)

#print(0.1 * (207 / 18))

#print(encoder_a.get_angle01())
#print(encoder_b.get_angle01())

angle = crt.get_turn(max_turn=1.5)
print(angle)
