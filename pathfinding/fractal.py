from PIL import Image, ImageDraw
from random import randint

_for_draw = None
_count = 0
path = []

def dir(x1, x2):
	return 1 if x2 > x1 else -1

def distance(p1, p2):
	x1, y1, x2, y2 = p1 + p2
	d = (((max(x1,x2) - min(x1,x2))**2 + (max(y1,y2) - min(y1,y2))**2)**0.5)

	return d

def get_passability(image, p):
	if 0 < p[0] < image.width and 0 < p[1] < image.height:
		return 1 if image.getpixel(p) == (0, 255, 0, 255) else -1
	else:
		return -1

def normal(k, b, p):
	try:
		k1 = - 1/k
	except ZeroDivisionError:
		k1 = 100
	x1, y1 = p
	b1 = -((k1 * x1) - y1)
	return k1, b1

def line(A, B):
	x1, x2, y1, y2 = A[0], B[0], A[1], B[1]
	try:
		k = (y2 - y1)/(x2 - x1)
	except ZeroDivisionError:
		k = 100
	b = -((k * x1) - y1)
	return k, b

def middle(a, b):
	x0 = min(a[0], b[0])
	x1 = max(a[0], b[0])
	y0 = min(a[1], b[1])
	y1 = max(a[1], b[1])
	return int(x0 + (x1 - x0) / 2), int(y0 + (y1 - y0) / 2)

def find_start_end_points(image, w, h):
	p = []
	for x in range(w):
		for y in range(h):
			c = image.getpixel((x,y))
			if c == (255, 0, 0, 255):
				p += [(x,y)]
	return p.pop(randint(0,len(p)-1)), p.pop(randint(0,len(p)-1))

def find_new_point(image, mp, k, b):
	def _search_up_dn(mp, k, b):
		if abs(k) < 1:
			x0, y0 = [ int(_) for _ in mp ]
			for dx in range(image.width):
				x1, x2 =    (x0 + dx, x0 - dx)
				y1, y2 = (k * x1 + b, k * x2 + b)
				draw.ellipse((x1-2, y1-2, x1+2, y1+2), fill=(255,0,0,255))
				draw.ellipse((x2-2, y2-2, x2+2, y2+2), fill=(255,255,0,255))
				if get_passability(image, (x1, y1)) > 0:
					return (x1, y1)
				elif get_passability(image, (x2, y2)) > 0:
					return (x2, y2)
			return mp
		else:
			return _search_lt_rt(mp, k, b)

	def _search_lt_rt(mp, k, b):
		if abs(k) >= 1:
			x0, y0 = [ int(_) for _ in mp ]
			for dy in range(image.width):
				y1, y2 =     (y0 + dy, y0 - dy)
				x1, x2 = (-(b - y1)/k, -(b - y2)/k)
				draw.ellipse((x1-2, y1-2, x1+2, y1+2), fill=(0,255,255,255))
				draw.ellipse((x2-2, y2-2, x2+2, y2+2), fill=(0,0,255,255))
				if get_passability(image, (x1, y1)) > 0:
					return (x1, y1)
				elif get_passability(image, (x2, y2)) > 0:
					return (x2, y2)
			return mp
		else:
			return _search_up_dn(mp, k, b)

	global _for_draw
	draw = ImageDraw.Draw(_for_draw)
	return mp if get_passability(image, mp) > 0 else _search_up_dn(mp, k, b)

def split(image, sp, ep, level = 0):
	global _for_draw
	global path
	global _count

	if level > 50 or distance(sp, ep) < 5:
		return 0
	else:
		level += 1
	mp = middle(sp, ep)
	k, b = line(sp, ep)
	k1, b1 = normal(k, b, mp)
	p = find_new_point(image, mp, k1, b1)
	path.insert(path.index(sp)+1,p)
	if _count % 1000 == 0:
		draw = ImageDraw.Draw(_for_draw)
		draw.line(sp + p, fill = (  0,  0,255,255))
		draw.line(p + ep, fill = (255,  0,255,255))
		_for_draw.save('_%s_.png' % (_count))
	_count += 1
	split(image, p, ep, level)
	split(image, sp, p, level)

def optimize(path = []):
	to_delete = (10,10)
	for i in range(len(path)-1):
		p0 = path[i]
		if p0 != to_delete:
			for j in range(i+2, len(path)):
				p1 = path[j]
				d = distance(p0,p1)
				print(i,j,d)
				if d < 25:
					print(j-1, i+2, -1)
					for k in range(j-1, i+2, -1):
						path[k] = to_delete
					break
	return path

	return [_ for _ in path if _ != to_delete]

def main(filename):
	global _for_draw
	global path
	i = Image.open(filename)
	_for_draw = Image.open(filename)
	w, h = i.width, i.height
	sp, ep = find_start_end_points(i, w, h)
	path = [sp, ep]
	split(i, sp, ep)

	_for_draw = Image.open(filename)
	draw = ImageDraw.Draw(_for_draw)
	# path = optimize(path)
	for p in path:
		x,y = p
		draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill = (255,0,0,255))
	_for_draw.save('res.png')

if __name__ == '__main__':
	import sys
	main(sys.argv[1])
