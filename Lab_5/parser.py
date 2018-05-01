import re

def readAndParse():
	fname = 'run.txt'
	p = re.compile(r'/(?<=position\: \[)-?\d+.\d+(e-?\d+)?, -?\d+.\d+(e-?\d+)?');
	with open(fname) as f:
		content = f.readlines()
		print(content);
		content = [p.match(x) for x in content]
		
		print(content);