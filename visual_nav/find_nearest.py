from sympy import solve as sympy_solve
from sympy import symbols as sympy_symbols
from sympy import Point, Line 

x, y = sympy_symbols('x, y')

def _coordinates_to_point(c):
    return Point(c[x],c[y])

def _point_to_coordinates(p):
    return {x:float(p.x), y:float(p.y)}

def _get_ordered_coords(my_coords, coords):
    rez = sorted([ _coordinates_to_point(c) for c in coords ], key=lambda p : p.distance(_coordinates_to_point(my_coords)))
    return [ _point_to_coordinates(p) for p in rez ]

def find_nearest(my_coords, coords, n=3):
    return _get_ordered_coords(my_coords, coords)[:n]

if __name__ == '__main__':

    coords = [
        {x: 4251.31393799202, y: 11962.4721191489}, 
        {x: 5178.53442188298, y: 9180.81066747607}, 
        {x: 4435.00740706984, y: 12408.1013573155}, 
        {x: 5592.86674004516, y: 8934.52335838951}, 
        {x: 4501.96652887774, y: 12319.1479230918}, 
        {x: 5593.06197695726, y: 9045.86157885321}, 
        {x: 4181.35695726841, y: 12060.4191301198}, 
        {x: 5181.33704388659, y: 9060.47887026523}
    ]

    my_coords = {x:0, y:0}
    print(find_nearest(my_coords, coords))
    my_coords = {x:6000, y:12000}
    print(find_nearest(my_coords, coords))