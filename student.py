from math import sqrt, pow

ZKS = {
 1:{ 0.90:6.31, 0.95:12.70, 0.98:31.80, 0.99:63.60},
 2:{ 0.90:2.92, 0.95: 4.30, 0.98: 6.97, 0.99: 9.93},
 3:{ 0.90:2.35, 0.95: 3.18, 0.98: 4.54, 0.99: 5.84},
 4:{ 0.90:2.13, 0.95: 2.78, 0.98: 3.75, 0.99: 4.60},
 5:{ 0.90:2.02, 0.95: 2.57, 0.98: 3.37, 0.99: 4.03},
 6:{ 0.90:1.94, 0.95: 2.45, 0.98: 3.14, 0.99: 3.71},
 7:{ 0.90:1.90, 0.95: 2.36, 0.98: 3.00, 0.99: 3.50},
 8:{ 0.90:1.86, 0.95: 2.31, 0.98: 2.90, 0.99: 3.36},
 9:{ 0.90:1.83, 0.95: 2.26, 0.98: 2.82, 0.99: 3.25},
10:{ 0.90:1.81, 0.95: 2.23, 0.98: 2.76, 0.99: 3.17},
11:{ 0.90:1.80, 0.95: 2.20, 0.98: 2.72, 0.99: 3.11},
12:{ 0.90:1.78, 0.95: 2.18, 0.98: 2.68, 0.99: 3.05},
}

class NotEnoughValuesException(Exception):
	pass

class Student(object):
	"""
	Students coefficient
	P in [0.90 0.95 0.98 0.99]
	"""

	def __init__(self, rez, P = 0.95, prc = 2): 
		self.prc = prc
		n = len(rez)
		if n < 2:
			raise NotEnoughValuesException
		else :
			x_ = sum(rez) / n
			s = sqrt(
				sum([pow(x - x_,2) for x in rez]) / 
				(n - 1)
			)
			
			N = n - 1 if n < 13 else 12
			Dx = (ZKS[N][P] * s) / sqrt(n)

			self.X = round(x_, self.prc)
			self.Dx = round(Dx, self.prc)

	def __str__(self):
		return "(%s +/- %s)" % (self.X, self.Dx)


if __name__ == "__main__":
    S = Student([1,2,3,4,3,2,1,2,3,4,5,4,3,2,1,1000],0.95)
    print(S)
    print(S.X)
    print(S.Dx)