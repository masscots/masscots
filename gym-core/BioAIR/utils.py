import math

def __calcVelocityVector( SQ, x_self, y_self, x_other, y_other):

    # SQ threshold
    __Rhi = 0.45  # 100 + (-65)
    __Rlo = 0.15  # 100 + (-85)
    
    field = 0
    velocityX = 0
    velocityY = 0

    if (SQ > __Rhi):
        field = (SQ - __Rhi) / (1 - __Rhi)
    elif (0 < SQ and SQ < __Rlo ):
        field = -__Rlo  / SQ
    else:
        field = 0
    print(f"Field:{field}, X_self:{x_self}, Y_self:{y_self}, x_other:{x_other}, y_other:{y_other}")
    velocityX = field * (x_self - x_other) / math.sqrt((x_self - x_other) ** 2 + (y_self - y_other) ** 2)
    velocityY = field * (y_self - y_other) / math.sqrt((x_self - x_other) ** 2 + (y_self - y_other) ** 2)
    return velocityX, velocityY


def getNextHeadingAndNextSpeed( neighbors, position_x, position_y):
    if len(neighbors) == 0:
        return False, 0, 0
    if position_x == None:
        return False, 0, 0

    heading = 0
    speed = 0
    vector_x, vector_y = 0, 0

    for key, value in neighbors.items():
        if value['Location'] == None:
            continue

        velocityX, velcoityY = __calcVelocityVector(value['SQ'], position_x, position_y, value['Location'][0], value['Location'][1])
        vector_x += velocityX
        vector_y += velcoityY

    return True, vector_x, vector_y
