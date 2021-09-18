import sys     #ввод необходимых библиотек для полета и записи
import time
import _thread
from datetime import datetime,timedelta
import SoapySDR
from SoapySDR import *
import numpy
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fftpack import fftshift
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
rospy.init_node('flight') #инициализация полётного нода
#настройка полетных параметров
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)  
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)В
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger) #триггер посадки
#конфиг записи SDR со списком спутников
SDR_CONFIGS = {
	'NOAA 18': {'freq': 1707.0e6, 'rssi_freq': [1707.3e6,1707.7e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0},
	'NOAA 19': {'freq': 1698.0e6, 'rssi_freq': [1698.3e6,1698.8e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0},
	'METEOR-M 2': {'freq': 1700.0e6, 'rssi_freq': [1700.3e6,1700.8e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0},
	'METEOR-M2 2': {'freq': 1700.0e6, 'rssi_freq': [1700.3e6,1700.8e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0},
	'SARAL': {'freq': 1698.4e6, 'rssi_freq': [1698.3e6,1698.5e6 ], "sample_rate" : 6.0e6, 'bw': 1e6, 'gain':12.0},
	'METOP-A': {'freq': 1701.3e6, 'rssi_freq': [1700.7e6,1701.9e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':12.0},
	'METOP-B': {'freq': 1701.3e6, 'rssi_freq': [1700.7e6,1701.9e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':12.0},
	'FENGYUN 3A': {'freq': 1704.5e6, 'rssi_freq': [1704.0e6,1705.0e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':12.0},
	'FENGYUN 3B': {'freq': 1704.5e6, 'rssi_freq': [1704.0e6,1705.0e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':12.0},
	'FENGYUN 3C': {'freq': 1701.4e6, 'rssi_freq': [1701.0e6,1701.8e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':12.0},
}
DEF_GAIN = 13.0
DEF_BW = 4000000.0

SPECTR_PLOT = None
def plot_spectr(samples,sample_rate,center_freq):
	global SPECTR_PLOT
	f, Pxx = fft(samples,sample_rate,center_freq)
	plt.semilogy(f/1e6, Pxx)
	plt.xlabel('f [MHz]')
	plt.ylabel('PSD [Power/Hz]')
	plt.grid()
	plt.show()
		
def fft(nperseg ,samples,sample_rate,center_freq):
	f, Pxx = signal.welch(samples, sample_rate, nperseg =nperseg , scaling='spectrum',return_onesided=False)
	f, Pxx = fftshift(f), fftshift(Pxx)
	f = f + center_freq
	return f,10*numpy.log10(Pxx)
#настройка SDR модуля
class OSMO_SDR:   ## Soapy based SDR class
	def __init__(self, device_str=""):
		self.device_str = device_str
		self.verbose = True
		self.conf = None
		self.th = None
		self.stream_break = False
		self.state='init'
		self.config_name=''
		self.rssi = 0.0
		self.rssi_log = []
		#
		self.spectr_F = None
		self.spectr_PSD = None
		self.FFT_PERIOD = 0.5 #set to 0 to disable FFT calculation
		self.FFT_SIZE=1024  
		self.FFT_AVERAGE = 4 #defines number of FFT_SIZE samples passed to fft()
		self.fft_samples = numpy.array([0]*self.FFT_SIZE, numpy.complex64)
		#
		try:
			self.sdr = SoapySDR.Device(device_str)
		except Exception as e:
			print ("ERROR: OSMO_SDR failed to init device: {} ( {} )".format(device_str,e))
			self.state='NO DEVICE'
			raise Exception(e)
			
		if self.verbose : 
			print("Init SDR device [{}]".format(device_str))
			print("|-sample rates: "+str(self.sdr.listSampleRates(SOAPY_SDR_RX, 0)))
			print("|-BW:"+str(self.sdr.getBandwidthRange(SOAPY_SDR_RX, 0)))
			print("|-Gains:"+str(self.sdr.listGains(SOAPY_SDR_RX, 0)))
		
	def __del__(self):
		self.sdr = None
		if self.verbose : print("SDR destroy")
		pass
		
	def stream_thread(self,data_file='',log_file='',spectr_file=''):
		buff = numpy.array([0]*1024*32, numpy.complex64)
		# start streaming
		self.state='streaming'
		#--prepare IQ data stream file
		data_fid = None
		if data_file != '':
			data_fid = open(data_file, "wb")
			if self.verbose :print("streaming to [{}]".format(data_file))
		#prepare spectrum file
		spectr_fid = None
		spectr_write_heading =True
		if spectr_file != '':
			spectr_fid = open(spectr_file, "wt")
			if self.verbose : print("spectrum to [{}]".format(spectr_file))
		#--activate streaming
		rxStream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
		self.sdr.activateStream(rxStream) 
		#
		rssi_f0ind=None
		rssi_f1ind=None
		t0 = time.time()
		prev_shot_time = time.time()
		while not self.stream_break:
			sr = self.sdr.readStream(rxStream, [buff], len(buff))
			if data_fid is not None: buff.tofile(data_fid)
			if self.FFT_PERIOD > 0 :
				if time.time() - prev_shot_time > self.FFT_PERIOD: #fetch a FFT shot
					#self.fft_samples = buff[0:self.FFT_SIZE]
					self.spectr_F,self.spectr_PSD=fft(self.FFT_SIZE,buff[0: self.FFT_AVERAGE*self.FFT_SIZE],self.conf["sample_rate"],self.conf['freq'])
					prev_shot_time = time.time()
					#-- calc rssi by frequency subrange
					if 'rssi_freq' in self.conf:
						if rssi_f0ind is None: #find range indexes, do once
							for i in range(0,len(self.spectr_F)):
								if rssi_f0ind is None:
									if self.spectr_F[i]>self.conf['rssi_freq'][0]: 
										rssi_f0ind=i
										rssi_f1ind=i+1
								else:
									if self.spectr_F[i]<self.conf['rssi_freq'][1]: rssi_f1ind=i
							if rssi_f0ind is None: #not yet found
								print("SDR WARNING: not found RSSI range {}-{} in {}-{}".format(self.conf['rssi_freq'][0],self.conf['rssi_freq'][1],self.spectr_F[0],self.spectr_F[-1]))
							else:
								print("SDR RSSI range {}[{}]-{}[{}] in {}-{}".format(self.conf['rssi_freq'][0],rssi_f0ind,self.conf['rssi_freq'][1],rssi_f1ind,self.spectr_F[0],self.spectr_F[-1]))
						if rssi_f0ind is not None:
							self.rssi = numpy.mean(self.spectr_PSD[rssi_f0ind:rssi_f1ind])
							self.rssi_log.append([datetime.utcnow(),self.rssi])
							if self.verbose:
								print("{}\t{:.1f}".format(datetime.utcnow(),self.rssi))
					#-- save spectrum
					if spectr_fid is not None: #dump spectr to file
						if spectr_write_heading: #write file header
							spectr_write_heading = False
							numpy.set_printoptions(threshold=numpy.inf, suppress=True)
							spectr_fid.write("Start: {}\nSDR Config:{}\nFFT Size: {}\nSample rate: {}\nFrequency: {}\n".format(datetime.utcnow(),self.config_name,self.FFT_SIZE,self.conf["sample_rate"],self.conf['freq']))
							spectr_fid.write("SDR RSSI range {}[{}]-{}[{}] in {}-{}\n".format(self.conf['rssi_freq'][0],rssi_f0ind,self.conf['rssi_freq'][1],rssi_f1ind,self.spectr_F[0],self.spectr_F[-1]))
							spectr_fid.write("Fbins: {}\n\n".format(numpy.array2string((self.spectr_F)/1e6,max_line_width=numpy.inf, precision=3)))
						spectr_fid.write("{}\t{:.1f}\t{}\n".format(datetime.utcnow(),self.rssi,numpy.array2string(self.spectr_PSD, max_line_width=numpy.inf, precision=2))) #,separator ='\t'
				#
		#shutdown the stream and close files
		self.sdr.deactivateStream(rxStream) #stop streaming
		self.sdr.closeStream(rxStream)
		if data_fid is not None: data_fid.close()
		if spectr_fid is not None: spectr_fid.close()
		self.state='idle'
		print("==========SDR streaming end===========")
		
	def load_config(self, conf_file=None, config_id=0): 
		if self.verbose : print("|-Loading SDR config {}//{}".format(conf_file,config_id))
		if not conf_file in SDR_CONFIGS:
			print("OSMO_SDR ERROR: invalid config [{}]".format(conf_file))
			return False
		#
		self.config_name=conf_file
		conf=SDR_CONFIGS[conf_file]
		self.conf = conf
		self.sdr.setSampleRate(SOAPY_SDR_RX, 0, conf["sample_rate"])
		if self.verbose: print("Actual Rx Rate %f Msps"%(self.sdr.getSampleRate(SOAPY_SDR_RX, 0)/1e6))
		self.sdr.setFrequency(SOAPY_SDR_RX, 0, conf['freq'])
		self.sdr.setBandwidth(SOAPY_SDR_RX, 0, conf['bw'])
		if 'airspy' in self.device_str:
			self.sdr.setGain(SOAPY_SDR_RX, 0, 'LNA',conf['gain'])
			self.sdr.setGain(SOAPY_SDR_RX, 0, 'MIX',conf['gain'])
			self.sdr.setGain(SOAPY_SDR_RX, 0, 'VGA',conf['gain'])
		time.sleep(1)
		self.state='configured'
		if self.verbose :
			print("|-Freq:" + str(self.sdr.getFrequency(SOAPY_SDR_RX, 0)))
		return True
			
	def start(self,data_file='',log_file='',spectr_file=''):
		if self.verbose : print("Starting SDR...")
		if self.conf is None: 
			print("ERROR: SDR is not properly configured before start")
			return False
		try:
			self.stream_break = False
			self.th = _thread.start_new_thread(self.stream_thread,(data_file,log_file,spectr_file))
		except Exception as e:
			print("SDR thread start exception: {}".format(e))
		return True
		
	def stop(self):
		self.stream_break = True
		time.sleep(0.2)
		if self.verbose : print("////Stop SDR//// {}".format(self.state))
		self.save_log_file(self.rssi_log, self.config_name)
		
	def log_telemetry(self, passid='',satellite='', config='', duration = None, stop_time=None, log_file=None):
		""" monitor and log telemetry """
		t_start = time.time()
		try:
			fh = open(log_file, "w") 
			fh.write("#Pass ID: {}\n#Satellite: {}\n#Configuration: {}\n#Start time: {:%Y-%m-%d %H:%M:%S}\n\n".format(
				passid,satellite,config,datetime.datetime.utcnow()))
			fh.write("#Time\tLevel\tLevel2\tSNR\tBER\n")
		except Exception as e:
			print("")
			return False
		
		if stop_time is None and duration is None : duration = 300 #default
		logging_running = True
		while logging_running :
			#read
			#state = self.get_state(keep_alive=True)
			logln = "{}\t{:.1f}\t{:.1f}\t{:.1f}\t{}".format("{:%H:%M:%S.%f}".format(datetime.datetime.utcnow())[:-4],
				state['level'],state['level_out_dbm'],state['SNR'],	state['BER'])
			print(logln)
			fh.write(logln + "\n")
			#write
			time.sleep(0.1)
			#
			if (( stop_time is not None and stop_time < datetime.datetime.utcnow())
				or ( duration is not None and (time.time() - t_start > duration ) ) ) :
				logging_running = False
				print("stop {}".format(time.time() - t_start ))
		fh.write("\n#Closed at: {:%Y-%m-%d %H:%M:%S}".format(datetime.datetime.utcnow()))
		fh.close()
	
	def log_telemetry_threaded(self, passid='',satellite='', config='', duration = None, stop_time=None,  log_file=None):
		""" run logging in a separate thread """
		th = _thread.start_new_thread(self.log_telemetry,(passid,satellite,config, duration,stop_time, log_file))

	def save_log_file(self, rssi, sat_name):
		rssi_file_name = sat_name + rssi[0][0].strftime('%Y%M%d') + 'rec.log'
		with open(rssi_file_name, "wt") as rssi_f:
			for r in rssi:
				t = r[0]
				rssi_f.write("{}\t{:.1f}\n".format(t,r[1]))
			rssi_f.close()

def listSoapyDevices():
	results = SoapySDR.Device.enumerate()
	for result in results: print(result)

def test_sdr():
	print("Starting SDR test....")
	sdr = OSMO_SDR("airspy") #название модуля SDR
	if sdr.load_config("NOAA 18"): #Строка для выбора принимаемого спутника
		sdr.start("record6M.iq",'',"record6M.iq.txt") #название файла сохранения записи
		#начало кода полёта
 		time.sleep(3)
		navigate(x=0, y=0, z=0.9, frame_id='body', auto_arm=True)
		rospy.sleep(3)
		rospy.sleep(2)
		navigate(x=0, y=0, z=0.9, frame_id='aruco_map')
		time.sleep(180) #необходимое время висения
		#конец кода полёта


		sdr.stop() #отключение записи
	sdr=None

if __name__ == '__main__':
	test_sdr() #запуск программы




land() #посадка

