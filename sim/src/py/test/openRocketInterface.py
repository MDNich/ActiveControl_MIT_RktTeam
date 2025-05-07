#!/usr/local/python3

# MUST BE RUN FROM TOP LEVEL WORKING DIRECTORY: 'sim'.

import os
import numpy as np

#os.chdir('../../../')

import src.py.orhelper._orhelper as orhelper
from src.py.orhelper._enums import FlightDataType as dT
from src.py.etc.logcoloring import ColorHandler

from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['font.sans-serif'] = ['Computer Modern']
rcParams['text.usetex'] = True


import logging
LOG = logging.getLogger()
LOG.setLevel(logging.DEBUG)
for handler in LOG.handlers:
	LOG.removeHandler(handler)

logging.getLogger().addHandler(ColorHandler())

# match your system as needed.
os.environ['JAVA_HOME'] = '/opt/homebrew/Cellar/openjdk/23.0.2'
os.environ['CLASSPATH'] = './out/OpenRocket.jar'
with orhelper.OpenRocketInstance(os.environ['CLASSPATH'], 'OFF') as instance:
	orh = orhelper.Helper(instance)
	doc = orh.load_doc('dat/ork/canard1.ork')
	sim = doc.getSimulation(0)
	print("loaded document + simulation")

	orh.run_simulation(sim)

	data = orh.get_timeseries(sim, [dT.TYPE_PITCH_RATE,dT.TYPE_VELOCITY_TOTAL,dT.TYPE_TIME])

	t = np.array(data[dT.TYPE_TIME].tolist())
	pR = np.array(data[dT.TYPE_PITCH_RATE].tolist())
	vel = np.array(data[dT.TYPE_VELOCITY_TOTAL].tolist())

	writePath = 'dat/simResults/canard1_out.txt'
	figPath = 'dat/simResults/canard1_out.pdf'
	try:
		f = open(writePath,"x")
	except:
		f = open(writePath,"w")
	f.write(str(t.tolist()))
	f.write("\n")
	f.write(str(pR.tolist()))
	f.write("\n")
	f.write(str(vel.tolist()))
	f.close()
	orhelper.logger.info("File Saved at {}".format(writePath))


	import matplotlib.pyplot as plt


	fig, ax = plt.subplots()
	ax.plot(t,pR,label="Pitch Rate",color='blue')
	ax2 = ax.twinx()
	ax2.plot(t,vel,label="Velocity",color='red')
	ax.legend(loc='upper left')
	ax2.legend(loc='upper right')
	plt.savefig(figPath)















