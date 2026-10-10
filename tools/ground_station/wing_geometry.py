"""Equal-span, tip-connected wings in a rear view; positive roll lowers right tip."""
import math

ORDERS = {1: ("A",), 3: ("B", "A", "C"), 4: ("D", "B", "A", "C"),
          5: ("D", "B", "A", "C", "E"), 7: ("F", "D", "B", "A", "C", "E", "G")}


def wing_segments(configuration):
    """Return (name, left endpoint, right endpoint, absolute roll) in span units."""
    count = configuration["count"]
    order = ORDERS[count]
    base = configuration["roll_a"]
    angles = {"A": base, "B": base + configuration["ab"], "C": base + configuration["ac"],
              "D": base + configuration["ab"] + configuration["bd"],
              "E": base + configuration["ac"] + configuration["ce"],
              "F": base + configuration["ab"] + configuration["bd"] + configuration["df"],
              "G": base + configuration["ac"] + configuration["ce"] + configuration["eg"]}
    def vector(name):
        radians = math.radians(angles[name])
        return math.cos(radians), math.sin(radians)
    ax, ay = vector("A")
    segments = {"A": ((-ax / 2, -ay / 2), (ax / 2, ay / 2))}
    tip = segments["A"][0]
    for name in reversed(order[:order.index("A")]):
        dx, dy = vector(name)
        other = tip[0] - dx, tip[1] - dy
        segments[name] = other, tip
        tip = other
    tip = segments["A"][1]
    for name in order[order.index("A")+1:]:
        dx, dy = vector(name)
        other = tip[0] + dx, tip[1] + dy
        segments[name] = tip, other
        tip = other
    return [(name, *segments[name], angles[name]) for name in order]


def configuration_ready(configuration):
    return bool(configuration and configuration["master"] == 1 and
                (configuration["count"] == 1 or
                 configuration["left_valid"] == 1 and configuration["right_valid"] == 1))
