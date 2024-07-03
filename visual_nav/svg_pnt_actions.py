from bs4 import BeautifulSoup as BS
from pprint import pprint

SCALE_COEFF = 20

def svg_to_points(svg_file):
    '''2: {
        'file':'./samples/point_2_0_1470_1470.jpeg',
        'w': 1740,
        'h': 1740,
        'x':    0,
        'y': 9000,
    },'''
    points = {}
    svg = BS(open(svg_file, 'r').read(), features='html.parser')
    pnt_nums = set()
    for img in svg.findAll('image'):
        pnt_nums |= {int(img['pnt_num'])}
    for pnt_num in pnt_nums:
        cameras = {}
        for img in svg.findAll('image', pnt_num=pnt_num):
            img_name = img['xlink:href'].split('/')[-1].split('.')[0]
            w, h = img['w_mm'], img['h_mm']
            cam_num = int(img['cam_num'])
            path = '/'.join(img['xlink:href'].split('/')[-2:])
            # TODO: how to create new {} when pnt_num changes

            cameras.update({cam_num: 
                {
                    'file': f'./{path}',
                    'w': w,
                    'h': h,
                    # TODO: x mixed with y
                    'x': int(float(img['y']))*SCALE_COEFF,
                    'y': int(float(img['x']))*SCALE_COEFF,
                }
            })
        points.update({pnt_num: cameras})

    return points



def points_to_svg(points):
    lines = []
    for p_k, p_v in points.items():
        for k,v in p_v.items():
            # TODO: x mixed with y
            svg_line = f'<image pnt_num="{p_k}" cam_num="{k}" width="40" height="40" w_mm="{v["w"]}" h_mm="{v["h"]}" xlink:href="./static/dist/img/{v["file"]}" x="{v["y"]/SCALE_COEFF}" y="{v["x"]/SCALE_COEFF}"/>'
            lines += [svg_line]
    svg = '\n'.join(lines)

    return svg

if __name__ == '__main__':

    pprint(svg_to_points('../test_map.svg'))


        # img_name = img['xlink:href'].split('/')[-1].split('.')[0]
        # w, h = img['w_mm'], img['h_mm']
        # cam_num = int(img['cam_num'])
        # pnt_num = int(img['pnt_num'])
        # # print(pnt_num)
        # path = '/'.join(img['xlink:href'].split('/')[-2:])
        # # TODO: how to create new {} when pnt_num changes

        # res.update({cam_num : 
        #     {
        #         'file': f'./{path}',
        #         'w': w,
        #         'h': h,
        #         # TODO: x mixed with y
        #         'x': int(float(img['y']))*SCALE_COEFF,
        #         'y': int(float(img['x']))*SCALE_COEFF,
        #     }
        # })