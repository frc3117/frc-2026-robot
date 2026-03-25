import pytest


from frc_ballistic_solver import CRTEncoder, CRT


@pytest.mark.parametrize(
    "encoder_a_teeth,encoder_b_teeth,turret_teeth,encoder_a_val,encoder_b_val,output",
    [
        (18, 23, 207, 0., 0., 0.),                          # Current config at zero
        (18, 23, 207, 0.16, 0.67, 0.18669),
        (76, 87, 197, 0.9886, 0.3582, 1.92461),
        (11, 39, 75, 0.1591, 0.3269, 0.16999465227127075)
    ])
def test_crt_encoder(encoder_a_teeth: int,
                     encoder_b_teeth: int,
                     turret_teeth: int,
                     encoder_a_val: float,
                     encoder_b_val: float,
                     output: float):
    encoder_a = CRTEncoder(encoder_a_teeth, turret_teeth, 1.)
    encoder_b = CRTEncoder(encoder_b_teeth, turret_teeth, 1.)
    crt = CRT(encoder_a, encoder_b, turret_teeth)

    encoder_a.set_angle01(encoder_a_val)
    encoder_b.set_angle01(encoder_b_val)

    turn = crt.get_turn()
    assert abs(turn - output) <= 0.001


@pytest.mark.parametrize(
    "encoder_a_teeth,encoder_b_teeth,turret_teeth,turret_turn,output_a,output_b",
    [
        (18, 23, 207, 0., 0., 0.),  # Current config 0 turn
        (18, 23, 207, 1, 0.5, 0.0),     # Current config 1 turn
        (18, 23, 207, 1.9999, 0.9988, 0.9991),   # Current config almost 2 turn
        (43, 85, 164, 1, 0.814, 0.9294),     # Random config 1 turn
        (29, 47, 129, 0.68, 0.0248, 0.8664)     # Random config 0.68
    ]
)
def test_reversed_set_encoder(encoder_a_teeth: int,
                              encoder_b_teeth: int,
                              turret_teeth: int,
                              turret_turn: float,
                              output_a: float,
                              output_b: float):
    encoder_a = CRTEncoder(encoder_a_teeth, turret_teeth, 1.)
    encoder_b = CRTEncoder(encoder_b_teeth, turret_teeth, 1.)
    crt = CRT(encoder_a, encoder_b, turret_teeth)

    crt.set_turn(turret_turn)

    assert abs(encoder_a.get_angle01() - output_a) <= 0.001
    assert abs(encoder_b.get_angle01() - output_b) <= 0.001


@pytest.mark.parametrize(
    "encoder_a_teeth,encoder_b_teeth,turret_teeth,max_turn,output",
    [
        (18,    23,    207,   2.,   True),      # Current config
        (15,    23,    207,   2.,   False),     # Similar config to current but can't do 2 turn
        (57,    62,    108,   5.,   True),      # Totally different config but still valid
        (57,    62,    108,   50.,  False),     # Coprime config but can't do 50. turn (only 32.722)
        (16,    32,    64,    1.,   False)      # Non coprime config
    ])
def test_valid_crt(encoder_a_teeth: int,
                   encoder_b_teeth: int,
                   turret_teeth: int,
                   max_turn: float,
                   output: bool):
    encoder_a = CRTEncoder(encoder_a_teeth, turret_teeth, 1.)
    encoder_b = CRTEncoder(encoder_b_teeth, turret_teeth, 1.)
    crt = CRT(encoder_a, encoder_b, turret_teeth)

    assert crt.is_valid(max_turn) == output
