import PIL.Image


from robot2026 import RebuiltFieldZone, RebuiltField
from frc_ballistic_solver import Vector2


def add_vec2(a: Vector2, b: Vector2):
    return Vector2(a.x + b.x, a.y + b.y)


################
#    Inputs    #
################

IMG_DOWNSCALE = 5

LEFT_MARGIN = 1067
BOTTOM_MARGIN = 190

REAL_X = 7992
REAL_Y = 3240

FIELD_LENGTH = 16.540988
FIELD_WIDTH = 8.069326

ZONE_COLORS = {
    RebuiltFieldZone.DONT_SHOOT: (0, 0, 0),
    RebuiltFieldZone.SHOOT_IN_HUB: (0, 255, 0),
    RebuiltFieldZone.PASS_PREVIOUS: (255, 255, 255),
    RebuiltFieldZone.PASS_LEFT: (0, 0, 255),
    RebuiltFieldZone.PASS_RIGHT: (255, 0, 0),
    RebuiltFieldZone.PASS_CENTER_PREVIOUS: (255, 0, 255)
}


###################
#    Constants    #
###################

IMG_X = int(REAL_X / IMG_DOWNSCALE)
IMG_Y = int(REAL_Y / IMG_DOWNSCALE)

FIELD_X = int((REAL_X - (2 * LEFT_MARGIN)) / IMG_DOWNSCALE)
FIELD_Y = int((REAL_Y - (2 * BOTTOM_MARGIN)) / IMG_DOWNSCALE)

DPI_X = FIELD_LENGTH / FIELD_X
DPI_Y = FIELD_WIDTH / FIELD_Y

OFFSET_X = int(LEFT_MARGIN / IMG_DOWNSCALE) * DPI_X
OFFSET_Y = int(BOTTOM_MARGIN / IMG_DOWNSCALE) * DPI_Y
OFFSET = Vector2(OFFSET_X, OFFSET_Y)

TOP = FIELD_Y * DPI_Y
BOTTOM = 0.

LEFT = 0.
RIGHT = FIELD_X * DPI_X

FIELD = RebuiltField()


##############
#    Code    #
##############

def pixel_to_pos(pixel: Vector2) -> Vector2:
    return Vector2(pixel.x * DPI_X - OFFSET.x, (IMG_Y - pixel.y) * DPI_Y - OFFSET.y)

img = PIL.Image.open('2026_Field_Gray.png')
img = img.resize((IMG_X, IMG_Y))

for i in range(IMG_Y):
    u_i = i
    for j in range(IMG_X):
        u_j = j

        zone = FIELD.get_zone(pixel_to_pos(Vector2(u_j, u_i)))
        color = ZONE_COLORS[zone]
        current_color = img.getpixel((j, i))

        img.putpixel((u_j, u_i), (int(color[0] * 0.3 + current_color[0] * 0.7), int(color[1] * 0.3 + current_color[1] * 0.7), int(color[2] * 0.3 + current_color[2] * 0.7)))

img.show("Shooting Map")
img.save('shooting_map.png')
