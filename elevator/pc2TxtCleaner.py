

import sys


def removeNan(args):
	print args

	if len(args) != 2:
		print "python pc2txtCleaner.py <filename>"
		return

	orig = open(args[1], 'r')
	new = open("clean" + args[1], 'w')

	for line in orig:
		if "nan" in line:
			continue
		new.write(line)

	orig.close()

	new.close()


def switchxyz(args):
	print args

	if len(args) != 2:
		print "python pc2txtCleaner.py <filename>"
		return

	orig = open(args[1], 'r')

	points = []

	for line in orig:
		x, y, z = 0
		splitLine = line.split("")
		for i in range(3): 
			x = splitLine[2]
			y = splitLine[1]
			z = splitLine[0]



	orig.close()

	new.close()


def main(args):
	switchxyz(args)



if __name__ == '__main__':
	main(sys.argv) 