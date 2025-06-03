
drive_unit_expressions = [
        (lambda gain, lx, az: gain * (lx - az)),
        (lambda gain, lx, az: gain * (lx + az))
]

def drive_unit_evaluation(linear, angular, gain, index):
    if index >= len(drive_unit_expressions):
        raise IndexError(f"Index {index} exceeds the number of drive unit expressions")
    return drive_unit_expressions[index](gain, linear['x'], angular['z'])

def get_expression_length():
    return len(drive_unit_expressions)

